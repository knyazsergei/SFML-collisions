#include <SFML/Graphics.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <iostream>

// We need these two static and global to be able to modify them from our handle-function!
static sf::ConvexShape B;
// Velocity of moveable rectangle (B)
static sf::Vector2f vel;

/*
FUNCTIONS SHARED BY GJK&EPA BEGINS HERE
*/

float dot(const sf::Vector2f& v1, const sf::Vector2f& v2) {
	return (v1.x * v2.x + v1.y * v2.y);
}

// A simplex is just a list of points. It contains 2-3 points for GJK algorithm and 3+ for EPA.
typedef std::vector<sf::Vector2f> Simplex;

// Find point furthest away in a particular direction in a shape
size_t GetFurthestInDirection(const sf::ConvexShape& shape, const sf::Vector2f& direction) {
	// transform is needed to get correct position from rectangleshapes, otherwise you just end up with points at the origin
	// if the algorithm is modified to use another form of shapes which, this can probably be removed
	const sf::Transform& shapeTrans = shape.getTransform();

	float furthestDot = dot(shapeTrans.transformPoint(shape.getPoint(0)), direction);
	size_t furthestIndex = 0;

	for (size_t i = 0; i < shape.getPointCount(); ++i) {
		// if the dot product between the point and the diretion is larger than any other point in the shape, it's furthest away
		float curDot = dot(shapeTrans.transformPoint(shape.getPoint(i)), direction);
		if (curDot > furthestDot) {
			furthestDot = curDot;
			furthestIndex = i;
		}
	}

	return furthestIndex;
}

// This returns a single vector in the minkowski sum which also is furthest away in the specified direction (this avoids using points which aren't part of the minkowski difference hull). Used by both EPA & GJK algorithm
sf::Vector2f Support(const sf::ConvexShape& shape1, const sf::ConvexShape& shape2, const sf::Vector2f& direction) {
	sf::Vector2f p1 = shape1.getTransform().transformPoint(shape1.getPoint(GetFurthestInDirection(shape1, direction)));
	sf::Vector2f p2 = shape2.getTransform().transformPoint(shape2.getPoint(GetFurthestInDirection(shape2, -direction)));

	sf::Vector2f p3 = p1 - p2;

	return p3;
}

/*
FUNCTIONS SHARED BY GJK&EPA ENDS HERE
*/

/*
EPA ALGORITHM BEGINS HERE
epa begins with the simplex from the GJK-algorithm which was used to find the collision. It then expands it until it finds the edge of the polygon!
*/

// A edge described using a normal, used by EPA algorithm
struct Edge {
	sf::Vector2f normal;
	float distance;
	size_t index;
};

// Finds the edge closest to the origin in the simplex
Edge FindClosestEdge(const Simplex& simplex) {
	Edge closest;

	// set distance to maximum of a float
	closest.distance = FLT_MAX;

	for (size_t i = 0; i < simplex.size(); ++i) {
		size_t j = i + 1;
		if (j == simplex.size())
			j = 0;

		// i is the current point in the simplex and j is the next

		const sf::Vector2f& a = simplex.at(i);
		const sf::Vector2f& b = simplex.at(j);

		// find our edge
		sf::Vector2f e = b - a;

		// find the normal of the edge
		sf::Vector2f n(-e.y, e.x);
		if (dot(n, -a) >= 0) { // new direction might "wrong way", check against other side
			n = -n;
		}

		n *= 1 / sqrt(dot(n, n)); // normalize!

		float d = dot(a, n); // distance between orgin and normal (a is the vector between origin and a since: a - origin = a - [0,0] = a)

							 // if d is less than the previously closest
		if (d < closest.distance) {
			closest.distance = d;
			closest.normal = n;
			closest.index = j; // we use an index for performance reasons, so we can just look it up in the simplex later
		}
	}

	return closest;
}

// Main function for finding the penetration depth of the collision. The shapes passed to this function MUST BE COLLIDING. The simplex is the simplex which was used to find the collision, that is the terminating simplex of the GJK-algorithm
sf::Vector2f FindPenetrationDistance(const sf::ConvexShape& shape1, const sf::ConvexShape& shape2, Simplex& simplex) {
	int i = 0; // i is just for security, preventing infinit loop (shouldn't happen though)
	while (++i < 100) {
		// find the edge closest to the origin in the simplex
		Edge e = FindClosestEdge(simplex);

		// find the furthest minkowski difference point in the direction of the normal
		const sf::Vector2f& p = Support(shape1, shape2, e.normal);

		// find the distance between the point and the edge
		float d = dot(p, e.normal);
		std::cout << "HIT" << std::endl;
		// find if we've hit the border of the minkowski difference!
		if (fabs(d - e.distance) < 0.01f) {
			return sf::Vector2f((d + 0.01f) * e.normal);
		}
		else {
			// add the point inbetween the points where it was found (due to the need of correct winding)
			simplex.insert(simplex.begin() + e.index, p);
		}
	}

	return sf::Vector2f();
}

/*
END OF EPA ALGORITHM
*/

/*
GJK ALGORITHM BEGINS HERE
GJK tries to find a tringle in the minkowski difference of two convex shapes which includes the origin.
*/

// finds out if our simplex contains the origin or not! also modifies the search direction (the direction in which we should further expand our simplex)
// note that the parameters are references, making it possible to modify them
bool ContainsOrigin(std::vector<sf::Vector2f>& simplex, sf::Vector2f& direction) {
	// a is the newest point
	// b&c are the "old points"

	const sf::Vector2f& a = simplex.back();
	sf::Vector2f ao = -a; // vector from newest point in simplex to the origin

						  // triangular simplex, will check if the triangle includes the origin. If it does not it removes the "last" points from the simplex
	if (simplex.size() == 3) {
		const sf::Vector2f& b = simplex.at(0);
		const sf::Vector2f& c = simplex.at(1);

		sf::Vector2f ab = b - a; // vector between a and b
		sf::Vector2f abPerp = sf::Vector2f(-ab.y, ab.x); // the perpendicular vector to ab, possibly becomes new search direction if collision is not found
		if (dot(abPerp, c) >= 0) { // new direction might "wrong way", check against other side
			abPerp = -abPerp;
		}

		if (dot(abPerp, ao) > 0) {
			simplex.erase(std::find(simplex.begin(), simplex.end(), c)); // remove point in simplex, making it a line

			direction = abPerp; // set a new search direction, where we should look for an new third point
		}
		else { // we are in area of direction acPerp

			sf::Vector2f ac = c - a; // sme as for ab and abPerp but between a and c
			sf::Vector2f acPerp = sf::Vector2f(-ac.y, ac.x);
			if (dot(acPerp, b) >= 0) {
				acPerp = -acPerp;
			}

			if (dot(acPerp, ao) <= 0) // the origin isn't in the direction of acPerp, we just detected a collision! This is true since we already know that it isn't int the direction of abPerp
				return true; // fast exit

							 // does the following only if collision is not found. Erases a point, making the simplex a line and sets a new serach direction in the direction perpendicular to the one between a and c.
			simplex.erase(std::find(simplex.begin(), simplex.end(), b));
			direction = acPerp;
		}
	}
	else if (simplex.size() == 2) { // simplex is a line
		const sf::Vector2f& b = simplex.at(0);

		// find the vector between the two points
		sf::Vector2f ab = b - a;

		// find the perpendicular vector in the direction of the origin
		sf::Vector2f abPerp = sf::Vector2f(-ab.y, ab.x);
		if (dot(abPerp, ao) < 0) {
			abPerp = -abPerp;
		}

		// set direction to the perpendicular vector, this is the direction where we want to look for a third point
		direction = abPerp;
	}

	// no collision this iteration
	return false;
}

// Main GJK functionm, returns true if it finds a collision, feed EPA with simplex for penetration distance. parameter simplex should be empty
bool AreColliding(const sf::ConvexShape& shape1, const sf::ConvexShape& shape2, Simplex& simplex) {
	// look in any direction, (1,1) is just an example
	sf::Vector2f direction = sf::Vector2f(1, 1);
	// find a point in minkowski difference in the direction
	simplex.push_back(Support(shape1, shape2, direction));
	// look in the opposite direction
	direction = -direction;

	int i = 0;
	while (++i < 100) {
		// find a new vector in the search direction
		sf::Vector2f newVec = Support(shape1, shape2, direction);
		if (dot(newVec, direction) < 0) { // if we couldn't find a new point in the direction, we stop
			return false;
		}
		else { // we found a new point!
			simplex.push_back(newVec); // add to simplex

									   // the simplex contains the origin, collision!
			if (ContainsOrigin(simplex, direction)) {
				return true;
			}
		}
	}

	return false;
}

/*
END OF GJK ALGORITHM
*/


// test program, renders two boxes, makes one of them movable with arrow keys, makes them rotatable with I & O-keys and then uses the GJK&EPA algorithms on them
int main() {
	sf::RenderWindow window(sf::VideoMode(1024, 768), "GJK&EPA test!");
	sf::Event evt;
	window.setFramerateLimit(30);

	// the static Shape (can only be rotated, not moved)
	sf::ConvexShape A(4);

	// Declaration of B is at the top of this file
	B.setPointCount(4);
	B.setPoint(0, sf::Vector2f(0, 0));
	B.setPoint(1, sf::Vector2f(50, 0));
	B.setPoint(2, sf::Vector2f(50, 80));
	B.setPoint(3, sf::Vector2f(0, 80));
	B.setFillColor(sf::Color::Green);

	A.setPoint(0, sf::Vector2f(0, 0));
	A.setPoint(1, sf::Vector2f(200, -40));
	A.setPoint(2, sf::Vector2f(200, 180));
	A.setPoint(3, sf::Vector2f(0, 120));
	A.setPosition(300, 300);
	A.setFillColor(sf::Color::Red);

	while (window.isOpen()) {
		while (window.pollEvent(evt)) {
			if (evt.type == sf::Event::Closed)
				window.close();
		}

		// Movement stuff
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
			vel.x -= 1;
		}
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
			vel.x += 1;
		}
		else {
			vel.x *= 0.2f;
		}

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
			vel.y -= 1;
		}
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
			vel.y += 1;
		}
		else {
			vel.y *= 0.2f;
		}

		if (vel.x > 1) {
			vel.x = 4;
		}

		if (vel.x < -1) {
			vel.x = -4;
		}

		if (vel.y > 1) {
			vel.y = 4;
		}

		if (vel.y < -1) {
			vel.y = -4;
		}

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::O))
			A.rotate(0.25f);

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::I))
			B.rotate(0.25f);

		B.move(vel);

		// Do GJK & possibly EPA
		Simplex simplex;
		bool colliding = AreColliding(A, B, simplex);
		if (colliding)
			B.move(FindPenetrationDistance(A, B, simplex));

		window.clear();

		window.draw(B);
		window.draw(A);

		window.display();
	}
}