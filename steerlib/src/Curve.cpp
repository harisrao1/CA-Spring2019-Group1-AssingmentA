//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	std::cout << "hey in addcontrolpoint (not points)\n";
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	if (!checkRobust())
	{
		return;
	}

	float currentTime = controlPoints[0].time;
	float endTime = controlPoints[controlPoints.size() - 1].time;

	Point initialPos = controlPoints[0].position;
	Point nextPos;

	while (calculatePoint(nextPos, currentTime))
	{
		DrawLib::drawLine(initialPos, nextPos, curveColor, curveThickness);
		initialPos = nextPos;
		currentTime = currentTime + (float)window;
	}
	nextPos = controlPoints[controlPoints.size() - 1].position;
	DrawLib::drawLine(initialPos, nextPos, curveColor, curveThickness);

	return;

#endif
}


// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	CurvePoint temp = controlPoints[0];
	for (int i = 0; i < controlPoints.size(); i++)
	{
		int j = 0;
		while(j < controlPoints.size() && controlPoints[i].time < controlPoints[j].time)
		{
			temp = controlPoints[i];
			controlPoints[i] = controlPoints[j];
			controlPoints[j] = temp;
			j++;
		}
	}
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (type == hermiteCurve && controlPoints.size() > 1)
	{
		return true;
	}
	else if (type == catmullCurve && controlPoints.size() > 3)
	{
		return true;
	}
	return false;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for (int i = 0; i < controlPoints.size(); i++)
	{
		if (time < controlPoints[i].time) {
			nextPoint = i;
			return true;
		}
	}
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	float t = time;
	float ti = controlPoints[nextPoint - 1].time;
	float ti1 = controlPoints[nextPoint].time;

	float num = t - ti;
	float denom = ti1 - ti;

	float num3 = num * num*num;
	float num2 = num * num;
	float denom3 = denom * denom*denom;
	float denom2 = denom * denom;

	Point yi = controlPoints[nextPoint - 1].position;
	Point yi1 = controlPoints[nextPoint].position;

	float six = controlPoints[nextPoint - 1].tangent.x;
	float siy = controlPoints[nextPoint - 1].tangent.y;
	float siz = controlPoints[nextPoint - 1].tangent.z;
	Point si = Point(six, siy, siz);

	float si1x = controlPoints[nextPoint].tangent.x;
	float si1y = controlPoints[nextPoint].tangent.y;
	float si1z = controlPoints[nextPoint].tangent.z;
	Point si1 = Point(si1x, si1y, si1z);

	Point np1, np2, np3, np4;

	np1 = yi*(2*num3/denom3-3*num2/denom2+1);
	np2 = yi1*(-2*(num3)/(denom3)+3*(num2)/(denom2));
	np3 = si*((num3)/(denom2) + -2*(num2)/(denom)+num);
	np4 = si1 * ((num3)/(denom2) - (num2)/(denom));

	newPosition = np1 + np2 + np3 + np4;

	return newPosition;
	
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Calculate position at t = time on Catmull-Rom curve

	// Return result
	return newPosition;
}