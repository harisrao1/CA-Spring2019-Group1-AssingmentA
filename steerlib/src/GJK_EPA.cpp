#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions


int CheckOrigin(Util::Vector& vec, std::vector<Util::Vector>& simp) {
	Util::Vector simp1;
	Util::Vector simp1N;

	Util::Vector simp2;
	Util::Vector simp3;

	Util::Vector simpLine1;
	Util::Vector simpLine2;

	Util::Vector vec1;
	Util::Vector vec2;
	Util::Vector vec3;

	simp1 = simp.back();
	simp1N = simp1 * (-1);

	if (simp.size() > 2) {  //if simplex is a trinagle
		simp2 = simp[1];
		simp3 = simp[0];

		simpLine1 = simp2 - simp1;
		simpLine2 = simp3 - simp1;

		vec1 = Util::Vector(simpLine1.z, 0, simpLine1.x * (-1));
		vec2 = Util::Vector(simpLine2.z, 0, simpLine2.x * (-1));
		if (vec1*simp3 > 0) { // checking if towards the origin.
			vec1 = vec1 * (-1);
		}
		if (vec1*simp1N > 0) {
			simp.erase(simp.begin() + 0);
			vec = vec1;
			return false;
		}
		if (vec2*simp1N > 0) {
			simp.erase(simp.begin() + 1);
			vec = vec2;
			return false;
		}
		vec = vec1;
		return true;
	}
	else {
		simp2 = simp[1];
		simp3 = simp[0];
			Util::Vector simpLine3 = simp2 - simp3;

		vec3 = Util::Vector(simpLine3.z, 0, simpLine3.x * (-1));

		if (vec3 * simp1N < 0) {
			vec3 = vec3 * (-1);
		}
		vec = vec3;
		return false;
	}
	return false;
}

Util::Vector support(const std::vector<Util::Vector>& poly, const Util::Vector& vec) {
	int index;
	float currentMax;
	if (poly.size() == 0)
	{
		index = 0;
	}
	for (int i = 0; i < poly.size(); i++) {
		if (i == 0)
		{
			index = 0;
			currentMax = std::numeric_limits<float>::lowest();
		}
		Util::Vector t = poly[i];
		float dot = t * vec;
		if (dot > currentMax) {
			currentMax = dot;
			index = i;
		}
	}
	return poly[index];
}

Util::Vector minkowskyDiff(const std::vector<Util::Vector>& poly1, const std::vector<Util::Vector>& poly2, Util::Vector vec)
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
	simp.push_back(minkowskyDiff(poly1, poly2, dVec));
	Util::Vector dVecN = dVec * (-1);

	while (true)
	{
		simp.push_back(minkowskyDiff(poly1, poly2, dVecN));

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
		s = minkowskyDiff(poly1, poly2, vecNorm);
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
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& poly1, const std::vector<Util::Vector>& poly2)
{
	bool intersect;
	std::vector<Util::Vector> simp;
	intersect = gjkAlgorithm(poly1, poly2, simp);

	if (intersect == true)
	{
		epaAlgorithm(poly1, poly2, simp, return_penetration_depth, return_penetration_vector);
		simp.clear();
		return true;
	}
	else
	{
		simp.clear();
		return false;
	}

}




