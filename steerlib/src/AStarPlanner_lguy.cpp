//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


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


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}

	std::vector<int> AStarPlanner::getNeighbors(int id)
	{
		std::vector<int> neighbors;
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
				if (onGrid(i, j))
				{
					int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					if (canBeTraversed(index) && index != id)
					{
						neighbors.push_back(index);
					}
				}
			}
		}

		return neighbors;
	}

	bool AStarPlanner::onGrid(unsigned int x, unsigned int z)
	{
		float xMin = 0;
		float zMin = 0;
		float xMax = 200;
		float zMax = 200;

		if (x < xMin || x > xMax)
		{
			return false;
		}
		else if (z < zMin || z > zMax)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	int AStarPlanner::getGridIndexFromPoint(Util::Point p)
	{
		int id;
		id = gSpatialDatabase->getCellIndexFromLocation(p);
		return id;
	}

	unsigned int parents[100000];

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";
		
		//std::vector<AStarPlannerNode> closedSet, openSet;
		//std::map<> cameFrom;

		start.x = start.x + 0.5;
		goal.x = goal.x + 0.5;
		start.z = start.z + 0.5;
		goal.z = goal.z + 0.5;
		std::cout << "test" << std::endl;
		gSpatialDatabase = _gSpatialDatabase;

		return AStarWeighted(agent_path, start, goal, _gSpatialDatabase, append_to_path);

		return false;
	}

	double AStarPlanner::calculateHeuristicValue(Util::Point current, Util::Point end, double W, int h)
	{
		if (h == 1)
		{
			return W * (abs(current.x - end.x) + abs(current.z - end.z));
		}
		else if (h == 2)
		{
			return W * sqrt((current.x - end.x) * (current.x - end.x) + (current.z - end.z) * (current.z - end.z));
		}
	}

	double AStarPlanner::fvalue(AStarPlannerNode node, double e, Util::Point goal, double weight, int heuristic)
	{
		return node.g + e * calculateHeuristicValue(node.point, goal, weight, heuristic);
	}

	bool AStarPlanner::AStarWeighted(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* _gSpatialDatabase, bool append_to_path)
	{
		BinaryHeap fringe = BinaryHeap();

		double weight = 1.1;
		int heuristic = 1;
		AStarPlannerNode startNode(start, 0, calculateHeuristicValue(start, goal, weight, heuristic), NULL);
		startNode.parent = &startNode;
		std::vector<Util::Point> closed;
		std::vector<AStarPlannerNode> open;
		open.push_back(startNode);
		bool flag;
		bool flag2;

		fringe.insert(startNode);

		while (fringe.size > 0)
		{
			AStarPlannerNode node = fringe.popMin();
			std::cout << "current node = " << node.point << std::endl;
			for (int i = 0; i < open.size(); i++)
			{
				if (open[i].point == node.point)
				{
					open.erase(open.begin() + i);
					break;
				}
			}
			if (node.point == goal)
			{
				std::cout << "find the goal" << std::endl;
				std::cout << "goal's parent = " << node.parent->point << std::endl;

				int pID = getGridIndexFromPoint(node.parent->point);
				int ppID = parents[pID];
				std::cout << "goal's parent.parent is " << getPointFromGridIndex(ppID) << std::endl;
				std::vector<Util::Point> tempPath;
				tempPath.push_back(node.point);

				int count = 0;
				while (getPointFromGridIndex(pID) != start)
				{
					tempPath.push_back(getPointFromGridIndex(pID));
					pID = parents[pID];
					std::cout << "node is " << getPointFromGridIndex(pID) << std::endl;
				}
				tempPath.push_back(start);

				for (int i = tempPath.size() - 1; i >= tempPath.size() / 2; i--)
				{
					std::swap(tempPath[i], tempPath[tempPath.size() - 1 - i]);
				}
				for (int j = 0; j < tempPath.size(); j++)
				{
					tempPath[j].x += 0.5;
					tempPath[j].z += 0.5;
					agent_path.push_back(tempPath[j]);
				}
				return true;
			}

			closed.push_back(node.point);
			
			int id = getGridIndexFromPoint(node.point);
			std::vector<int> neighbors = getNeighbors(id);

			for (int i = 0; i < neighbors.size(); i++)
			{
				flag = true;
				flag2 = true;
				Util::Point nPoint = getPointFromGridIndex(neighbors[i]);
				for (int j = 0; j < closed.size(); j++)
				{
					if ((nPoint.x == closed[j].x) && (nPoint.z == closed[j].z))
					{
						flag = false;
						std::cout << nPoint << " is in the closed" << std::endl;
						break;
					}
				}
				if (flag)
				{
					AStarPlannerNode nNode(node.point, 0, 0, NULL);
					for (int j = 0; j < open.size(); j++)
					{
						if (nPoint == open[j].point)
						{
							nNode = open[j];
							flag2 = false;
							break;
						}
					}
					if (flag2)
					{
						nNode = AStarPlannerNode(nPoint, INFINITY, 0, NULL);
						open.push_back(nNode);
					}
					else
					{
						nNode = fringe.find(nPoint);
					}
					AStarPlannerNode neighborNode(nPoint, INFINITY, 0, NULL);
					neighborNode = updateNode(node, nNode, goal, flag2, fringe, open, weight, heuristic);
					nNode = neighborNode;
				}
			}
		}
		return false;
	}
	AStarPlannerNode AStarPlanner::updateNode(AStarPlannerNode node, AStarPlannerNode nNode, Util::Point goal, bool flag2, BinaryHeap& fringe, std::vector<AStarPlannerNode>& open, double weight, int heuristic)
	{
		int index = getGridIndexFromPoint(nNode.point);
		double traversal_cost = gSpatialDatabase->getTraversalCost(index);
		if (node.g + traversal_cost < nNode.g)
		{
			nNode.g = node.g + traversal_cost;
			nNode.f = nNode.g + calculateHeuristicValue(nNode.point, goal, weight, heuristic);
			nNode.parent = &node;
			parents[index] = getGridIndexFromPoint(node.point);
			if (!flag2)
			{
				fringe.update(nNode);
			}
			else
			{
				fringe.insert(nNode);
			}
		}
		return nNode;
	}
}