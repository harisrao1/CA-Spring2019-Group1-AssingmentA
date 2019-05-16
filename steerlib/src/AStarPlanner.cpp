//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//

#include <limits>
#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"

#define eHEURISTIC true
#define EPSILON 1.0f
#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
//1 = weighted, 2 = ARA*, 3 = AD*
#define ASTARTYPE 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}
	int time_limit;
	int time_cost;
	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}




	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	float AStarPlanner::Heuristic(Util::Point x, Util::Point y, float epsilon) {
		//euclidean
		if (eHEURISTIC) {
			return ((x - y)).length() * epsilon;
		}
		else {
			//manhattan
			return (abs(x.x - y.x) + abs(x.z - y.z) * epsilon);

		}
	}
	SteerLib::AStarPlannerNode* AStarPlanner::makeNeighbor(Util::Point point, Util::Point goal, SteerLib::AStarPlannerNode* current, bool dpad, float epsilon)
	{
		float g, f, h;

		if (dpad)
			g = 1 + current->g;
		else {
			if (eHEURISTIC)
				g = sqrt(2) + current->g; // move by 1.414 in euclidean or 2 in manhattan
			else
				g = current->g + 2;
		}

		h = Heuristic(point, goal, epsilon);

		f = g + h;
		SteerLib::AStarPlannerNode* temp = new SteerLib::AStarPlannerNode(point, g, f, current);
		return new SteerLib::AStarPlannerNode(point, g, f, current);
	}

	std::vector<SteerLib::AStarPlannerNode*> AStarPlanner::getNeighbors(SteerLib::AStarPlannerNode* current, Util::Point goal, float epsilon) {

		std::vector<SteerLib::AStarPlannerNode*> neighbors;
		std::vector<Util::Point> points;
		int x = current->point.x;
		int z = current->point.z;

		neighbors.clear();

		//defines directions adjacent

		points.push_back(Util::Point(x, 0, z - 1));
		points.push_back(Util::Point(x, 0, z + 1));
		points.push_back(Util::Point(x - 1, 0, z));
		points.push_back(Util::Point(x + 1, 0, z));

		//defines directions diagonal to

		points.push_back(Util::Point(x + 1, 0, z - 1));
		points.push_back(Util::Point(x + 1, 0, z + 1));
		points.push_back(Util::Point(x - 1, 0, z - 1));
		points.push_back(Util::Point(x - 1, 0, z + 1));

		for (int i = 0; i < points.size(); i++) {
			if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i]))) {
				if (i < 4)
					neighbors.push_back(makeNeighbor(points[i], goal, current, true, epsilon));
				else
					neighbors.push_back(makeNeighbor(points[i], goal, current, false, epsilon));
			}
		}

		return neighbors;
	}

	int AStarPlanner::lowestF(std::vector<SteerLib::AStarPlannerNode*>& tempSet) {
		//defining a temp node to store the node with the lowest f score
		int lowest = 0;
		//initiating to the first node

		//iterating through the set to find the lowest f_score
		for (int i = 0; i < tempSet.size(); i++) {
			if (*tempSet[i] < *tempSet[lowest]) {
				lowest = i;
			}
		}
		return lowest;
	}

	std::vector<Util::Point> AStarPlanner::pathMake(SteerLib::AStarPlannerNode* goal, Util::Point start) {
		std::vector<Util::Point> totalPath;

		SteerLib::AStarPlannerNode* current = goal;
		totalPath.clear(); // clear data.

		while (current != nullptr) {
			//std::cout << "at this neighbor: " << current->6point.x << " and z: " << current->point.z << "\n";
			totalPath.push_back(current->point);
			current = current->parent;
		}
		std::reverse(totalPath.begin(), totalPath.end());
		return totalPath;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		switch (ASTARTYPE) {
		case 1:
			//weighted A star
			return weightedAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
			break;
		case 2:
			//ARA*
			return anytimeRepetitive(agent_path, start, goal, _gSpatialDatabase, append_to_path);
			break;
		case 3:
			//AD*
			break;
		}
		//if it is just A* with an epsilon weight


		return false;
	}


	//All of the three different types will be given their separate methods here
	bool AStarPlanner::weightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//Defining sets as vectors
		std::vector<SteerLib::AStarPlannerNode*> openSet, closedSet, neighbors;
		//std::vector<SteerLib::AStarPlannerNode> neighbors;
		SteerLib::AStarPlannerNode* current;

		//Define starting node
		SteerLib::AStarPlannerNode _start(start, 0, Heuristic(start, goal, EPSILON), nullptr);

		//starting off in openset
		openSet.push_back(&_start);
		while (!openSet.empty()) {
			//Defining temporary current node
			int a = lowestF(openSet);
			if (openSet[a]->point == goal) {
				//std::cout << "at this neighbor: " << current.point.x << " and z: " << current.point.z << "\n";
				//_start.parent->g = 0;  //force failure to debug
				std::cout << "found and done" << "\n";
				agent_path = pathMake(openSet[a], start);
				return true;
			}
			closedSet.push_back(openSet[a]);
			//std::cout << "at this neighbor: " << closedSet[closedSet.size()-1].point.x << " and z: " << closedSet[closedSet.size() - 1].point.z << "\n";
			neighbors = getNeighbors(openSet[a], goal, EPSILON);
			openSet.erase(openSet.begin() + a);
			for (int i = 0; i < neighbors.size(); i++) {
				//std::cout << "at this neighbor: " << neighbors[i].point.x << " and z: " << neighbors[i].point.z  << "\n";
				//std::cout << "at this neighbor parent: " << neighbors[i].parent->point.x << " and z: " << neighbors[i].parent->point.z << "\n";
				if (!(std::find_if(closedSet.begin(), closedSet.end(), [&](const SteerLib::AStarPlannerNode* node) {return node->point == neighbors.at(i)->point; }) != closedSet.end())) {
					//not in closed set, look if in openset
					std::vector<SteerLib::AStarPlannerNode*>::iterator temp = std::find_if(openSet.begin(), openSet.end(), [&](const SteerLib::AStarPlannerNode* node) {return node->point == neighbors.at(i)->point; });

					if (!(temp != openSet.end())) {
						openSet.push_back(neighbors[i]);
					}
					else {
						if (neighbors[i]->g < (*temp)->g) {
							//std::cout << "at this neighbor: " << neighbors[i].point.x << " and z: " << neighbors[i].point.z << " g " << neighbors[i].g << "\n";
							//std::cout << "at this neighbor parent: " << neighbors[i].parent->point.x << " and z: " << neighbors[i].parent->point.z << "\n";
							//std::cout << "at this temp: " << temp->point.x << " and z: " << temp->point.z << "\n";
							//std::cout << "at this temp: " << temp->parent->point.x << " and z: " << temp->parent->point.z << "g" << temp->g << "\n";
							(*temp)->g = neighbors[i]->g;
							(*temp)->parent = neighbors[i]->parent;
							(*temp)->f = (*temp)->g + Heuristic((*temp)->point, goal, EPSILON);
							//std::cout << "at this temp2: " << temp->point.x << " and z: " << temp->point.z << "\n";
							//std::cout << "at this temp2: " << temp->parent->point.x << " and z: " << temp->parent->point.z << "\n";
						}
					}
				}
			}
		}

		//TODO
		//std::cout << "\nIn A*";

		return false;
	}

	bool AStarPlanner::anytimeRepetitive(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{

		gSpatialDatabase = _gSpatialDatabase;
		float eps = 10;
		time_limit = 500;
		time_cost = 0;
		//Defining sets as vectors
		std::vector<SteerLib::AStarPlannerNode*> openSet, closedSet, inconSet;
		float denom;
		//Defining temporary current node
		SteerLib::AStarPlannerNode* current;
		//Define starting node
		SteerLib::AStarPlannerNode _start(start, 0, Heuristic(start, goal, eps), nullptr);

		vnodes.insert(std::make_pair(start, &_start));
		double inf = 10000;
		//Defining goal node
		SteerLib::AStarPlannerNode _goal(goal, inf, inf, nullptr);
		//starting off in openset
		openSet.push_back(&_start);
		//int goalF = 0 + Heuristic(start, goal);
		//first call of improvePath
		improvePath(eps, _start, _goal, agent_path, openSet, closedSet, inconSet, vnodes);
		//std::cout << "at this neighbor: \n";

		if (openSet[lowestF(openSet)]->f < inconSet[lowestF(inconSet)]->f) {
			denom = openSet[lowestF(openSet)]->f;
		}
		else {
			denom = inconSet[lowestF(inconSet)]->f;
		}
		float epsPrime = min(eps, _goal.f / denom);

		//looping through
		while (epsPrime > 1) {
			eps--;

			//moving states from inconSet to openSet
			while (!inconSet.empty()) {
				openSet.push_back(inconSet.back());
				inconSet.pop_back();
			}

			//reset closed to empty?
			while (!closedSet.empty()) {
				closedSet.pop_back();
			}

			improvePath(eps, _start, _goal, agent_path, openSet, closedSet, inconSet, vnodes);
			denom = openSet[lowestF(openSet)]->f;
			if (openSet[lowestF(openSet)]->f < inconSet[lowestF(inconSet)]->f) {
				denom = openSet[lowestF(openSet)]->f;
			}
			else {
				denom = inconSet[lowestF(inconSet)]->f;
			}
			float epsPrime = min(eps, _goal.f / denom);
		}


		//TODO
		//std::cout << "\nIn A*";

		return true;
	}

	//imrpove path needs work
	float cost(AStarPlannerNode * n, AStarPlannerNode * neighbor) {
		return (n->point - neighbor->point).length();
	}
	bool AStarPlanner::improvePath(float eps, SteerLib::AStarPlannerNode& start, SteerLib::AStarPlannerNode& goal, std::vector<Util::Point>& agent_path, std::vector<SteerLib::AStarPlannerNode*>& openSet, std::vector<SteerLib::AStarPlannerNode*>& closedSet, std::vector<SteerLib::AStarPlannerNode*>& inconSet, std::map<Util::Point, SteerLib::AStarPlannerNode*>& vnodes)
	{

		SteerLib::AStarPlannerNode* current;
		std::vector<SteerLib::AStarPlannerNode*> neighbors;
		//calculate fscore of goal?
		int a = lowestF(openSet);

		while (goal.f > openSet[a]->f && !openSet.empty() && (time_cost++ < time_limit)) {
			a = lowestF(openSet);
			current = openSet[a];
			closedSet.push_back(openSet[a]);
			neighbors = getNeighbors(openSet[a], goal.point, eps);
			openSet.erase(openSet.begin() + a);
			for (int i = 0; i < neighbors.size(); i++) {
				//std::cout << "at this neighbor: " << neighbors[i]->point.x << " and z: " << neighbors[i]->point.z << "\n";
				float c = cost(neighbors[i], current) + current->g;

				if (vnodes.find(neighbors[i]->point) == vnodes.end()) {
					//neighbors[i]->g = 10000;
					vnodes.insert(std::make_pair(neighbors[i]->point, neighbors[i]));
				}
				SteerLib::AStarPlannerNode* neighbor = vnodes[neighbors[i]->point];
				if (!(std::find_if(closedSet.begin(), closedSet.end(), [&](const SteerLib::AStarPlannerNode* node) {return node->point == neighbors.at(i)->point; }) != closedSet.end())) {
					openSet.push_back(neighbors[i]);
				}
				else {
					inconSet.push_back(neighbors[i]);
				}


			}
		}
		return time_cost < time_limit;
	}


}
