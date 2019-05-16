//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{
	class STEERLIB_API AStarPlannerNode {
	public:
		double f;
		double g;
		Util::Point point;
		AStarPlannerNode* parent;
		AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
		{
			f = _f;
			point = _point;
			g = _g;
			parent = _parent;
		}
		bool operator<(AStarPlannerNode other) const
		{
			if (this->f == other.f) {
				return this->g < other.g;
			}
			return this->f < other.f;
		}
		bool operator>(AStarPlannerNode other) const
		{
			if (this->f == other.f) {
				return this->g > other.g;
			}
			return this->f > other.f;
		}
		bool operator==(AStarPlannerNode other) const
		{
			return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		}

	};
	class STEERLIB_API AStarPlanner {
	public:
		AStarPlanner();
		~AStarPlanner();

		bool canBeTraversed(int id);

		Util::Point getPointFromGridIndex(int id);

		bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);

		float AStarPlanner::Heuristic(Util::Point x, Util::Point y, float epsilon);

		SteerLib::AStarPlannerNode* AStarPlanner::makeNeighbor(Util::Point point, Util::Point goal, SteerLib::AStarPlannerNode* current, bool dpad, float epsilon);
		std::vector<SteerLib::AStarPlannerNode*> AStarPlanner::getNeighbors(SteerLib::AStarPlannerNode* current, Util::Point goal, float epsilon);

		int AStarPlanner::lowestF(std::vector<SteerLib::AStarPlannerNode*>& tempSet);
		std::vector<Util::Point> AStarPlanner::pathMake(SteerLib::AStarPlannerNode* goal, Util::Point start);

		bool AStarPlanner::weightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path);
		bool AStarPlanner::anytimeRepetitive(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path);

		bool AStarPlanner::improvePath(float eps, SteerLib::AStarPlannerNode& start, SteerLib::AStarPlannerNode& goal, std::vector<Util::Point>& agent_path, std::vector<SteerLib::AStarPlannerNode*>& openSet, std::vector<SteerLib::AStarPlannerNode*>& closedSet, std::vector<SteerLib::AStarPlannerNode*>& inconSet, std::map<Util::Point, SteerLib::AStarPlannerNode*>& vnodes);

	private:
		SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
		std::map<Util::Point, SteerLib::AStarPlannerNode*> vnodes;
	};


}


#endif
