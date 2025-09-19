
#include <queue>


#ifndef ASTARPP_
#define ASTARPP_

#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <ctime>

#include <opencv2/opencv.hpp>

namespace hjk_samsung
{
	namespace pathplanning
	{
		class Astar
		{
		private:
			int* m_map;
			int* m_closed_nodes_map;
			int* m_open_nodes_map;
			int* m_dir_map;
			int m_dx[8];
			int m_dy[8];
			cv::Size m_mapsize;
		public:
			Astar(cv::Size mapsize);
			~Astar();
			void setGridmap(cv::Mat gridMap);
			//std::vector<cv::Point2d> pathFind(cv::Point2d start, cv::Point2d finish);
			std::vector<cv::Point> pathFind(cv::Point start, cv::Point finish);
            std::vector<cv::Point> pathFind_postpro(cv::Point start, cv::Point finish);
		};

		

		class node
		{
			// current position
			int xPos;
			int yPos;
			// total distance already travelled to reach the node
			int level;
			// priority=level+remaining distance estimate
			int priority;  // smaller: higher priority

		public:
			node(int xp, int yp, int d, int p) 
			{xPos=xp; yPos=yp; level=d; priority=p;}

			int getxPos() const {return xPos;}
			int getyPos() const {return yPos;}
			int getLevel() const {return level;}
			int getPriority() const {return priority;}

			void updatePriority(const int & xDest, const int & yDest)
			{
				priority=level+estimate(xDest, yDest)*10; //A*
			}

			// give better priority to going strait instead of diagonally
			void nextLevel(const int & i) // i: direction
			{
				//level+=(dir==8?(i%2==0?10:14):10);
				level+=(i%2==0?10:14);
			}

			// Estimation function for the remaining distance to the goal.
			const int & estimate(const int & xDest, const int & yDest) const
			{
				static int xd, yd, d;
				xd=xDest-xPos;
				yd=yDest-yPos;         

				// Euclidian Distance
				d=static_cast<int>(sqrt((double)xd*xd+yd*yd));

				// Manhattan distance
				//d=abs(xd)+abs(yd);

				// Chebyshev distance
				//d=max(abs(xd), abs(yd));

				return(d);
			}
		};

		// Determine priority (in the priority queue)
		inline bool operator<(const node & a, const node & b)
		{
			return a.getPriority() > b.getPriority();
		}
	}
}

#endif
