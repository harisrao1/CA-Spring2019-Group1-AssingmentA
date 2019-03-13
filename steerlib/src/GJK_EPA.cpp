#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions


int CheckOrigin(Util::Vector& vec, std::vector<Util::Vector>& simplex) {
	Util::Vector simplex1 = simplex.back();
	Util::Vector Negsimplex1 = simplex1 * (-1);

	if (simplex.size() == 3) {  //if simplex is a trinagle
		Util::Vector simplex2 = simplex[1];
		Util::Vector simplex3 = simplex[0];

		Util::Vector s2 = simplex2 - simplex1;
		Util::Vector s3 = simplex3 - simplex1;

		vec = Util::Vector(s2.z, 0, (-1)*s2.x);
		if (vec*simplex3 > 0) { // checking if towards the origin.
			vec = (-1) * vec;
		}
		if (vec*Negsimplex1 > 0) {
			simplex.erase(simplex.begin() + 0);
			return false;
		}

		vec = Util::Vector(s3.z, 0, s3.x * (-1));
		if (vec*Negsimplex1 > 0) {
			simplex.erase(simplex.begin() + 1);
			return false;
		}
		return true;
	}
	else {
		Util::Vector side = simplex[1] - simplex[0];

		vec = Util::Vector(side.z, 0, side.x * (-1));

		if (vec * Negsimplex1 < 0) {
			vec = vec * (-1);
		}
		return false;
	}
	return false;
}

Util::Vector support(const std::vector<Util::Vector>& mShape, const Util::Vector& v) {
	float temphigh = std::numeric_limits<float>::min();
	int index = 0;

	for (int i = 0; i < mShape.size(); i++) {
		if (i == 0) {
			index = 0;
		}
		Util::Vector t = mShape[i];
		float dot = t * v;
		if (dot > temphigh) { // Keep updating  until we find the max number.
			temphigh = dot;
			index = i;
		}
	}
	return mShape[index];
}

Util::Vector minkowsky(const std::vector<Util::Vector>& poly1, const std::vector<Util::Vector>& poly2, Util::Vector vec)
{
	Util::Vector poly1V;
	Util::Vector poly2V;
	Util::Vector line;

	poly1V = support(poly1, vec);
	poly2V = support(poly2, vec * (-1));
	line = poly1V - poly2V;

	return line;
}
bool gjkAlgorithm(const std::vector<Util::Vector>& poly1, const std::vector<Util::Vector>& poly2, std::vector<Util::Vector>& simp)
{
	Util::Vector dVec(1, 0, -1);
	simp.push_back(minkowsky(poly1, poly2, dVec));
	Util::Vector dVecN = dVec * (-1);

	while (true)
	{
		simp.push_back(minkowsky(poly1, poly2, dVecN));

		if (simp.back() * dVecN <= 0)
		{
			return false;
		}
		else
		{
			if (CheckOrigin(dVecN, simp))
			{
				return true;
			}
		}
	}
}



void closestEdge(std::vector<Util::Vector>& simp, float& dist, Util::Vector& norm, int& i)
{
	dist = std::numeric_limits<float>::max();
	Util::Vector vec1;
	Util::Vector vec2;
	Util::Vector e;
	Util::Vector x;

	for (int j = 0; j < simp.size(); j++)
	{
		int n;

		if (j + 1 == simp.size()) {
			n = 0;
		}
		else {
			n = j + 1;
		}

		vec1 = simp[j];
		vec2 = simp[n];
		e = vec2 - vec1;
		x = vec1 * (e*e) - e * (e*vec1);
		x = x / x.norm();

		if ((x * vec1) < dist)
		{
			dist = x * vec1;
			i = n;
			norm = x;
		}

	}

}

bool epaAlgorithm(const std::vector<Util::Vector>& poly1, const std::vector<Util::Vector>& poly2, std::vector<Util::Vector>& simp, float& dep, Util::Vector& vec)
{
	float dist;
	int i;
	Util::Vector vecNorm;
	Util::Vector s;
	float d;
	while (true)
	{
		closestEdge(simp, dist, vecNorm, i);
		s = minkowsky(poly1, poly2, vecNorm);
		d = s * vecNorm;

		if (d - dist <= 0) {
			vec = vecNorm;
			dep = dist;
			return true;
		}
		else {
			simp.insert(simp.begin() + i, s);
		}
	}

}
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	bool intersect;
	std::vector<Util::Vector> simp;
	intersect = gjkAlgorithm(_shapeA, _shapeB, simp);

	if (intersect == true)
	{
		epaAlgorithm(_shapeA, _shapeB, simp, return_penetration_depth, return_penetration_vector);
		simp.clear();
		return true;
	}
	else
	{
		simp.clear();
		return false;
	}

}




