#include "Astarpp.h"

using namespace std;
using namespace hjk_samsung::pathplanning;

Astar::Astar(cv::Size mapsize)
{
	m_mapsize = mapsize;

	m_map = new int[m_mapsize.width * m_mapsize.height];
	m_closed_nodes_map = new int[m_mapsize.width * m_mapsize.height];
	m_open_nodes_map = new int[m_mapsize.width * m_mapsize.height];
	m_dir_map = new int[m_mapsize.width * m_mapsize.height];

	m_dx[0] = 1;
	m_dx[1] = 1;
	m_dx[2] = 0;
	m_dx[3] = -1;
	m_dx[4] = -1;
	m_dx[5] = -1;
	m_dx[6] = 0;
	m_dx[7] = 1;

	m_dy[0] = 0;
	m_dy[1] = 1;
	m_dy[2] = 1;
	m_dy[3] = 1;
	m_dy[4] = 0;
	m_dy[5] = -1;
	m_dy[6] = -1;
	m_dy[7] = -1;
}

Astar::~Astar()
{
	delete m_map;
	delete m_closed_nodes_map;
	delete m_open_nodes_map;
	delete m_dir_map;
}

void Astar::setGridmap(cv::Mat gridMap)
{
	if(gridMap.type() != CV_8UC1)
	{
		std::cout << "ERROR : Astar gridmap type error" << std::endl;
		return;
	}

	if(gridMap.size().width != m_mapsize.width || gridMap.size().height != m_mapsize.height)
	{
		std::cout << "Error : Astar gridmap size error" << std::endl;
		return;
	}
	for(int y = 0; y < gridMap.size().height; y++)
	{
		for(int x = 0; x < gridMap.size().width; x++)
		{
            //if(gridMap.at<unsigned char>(y,x) == 0)
            //if(gridMap.at<unsigned char>(y,x) < 127)
            if(gridMap.at<unsigned char>(y,x) > 127)
				m_map[y * m_mapsize.width + x] = 0;
			else
				m_map[y * m_mapsize.width + x] = 1;
		}
	}
}

//std::vector<cv::Point2d> KAI_LOC::pathplanning::Astar::pathFind(cv::Point2d start, cv::Point2d finish)
std::vector<cv::Point> Astar::pathFind(cv::Point start, cv::Point finish)
{
	std::vector<cv::Point> result;

	priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
	int pqi; // pq index
	node* n0;
	node* m0;
	int i, j, x, y, xdx, ydy;
	char c;
	pqi=0;
	int dir = 8;

	int xStart = start.x;
	int yStart = start.y;
	int xFinish = finish.x;
	int yFinish = finish.y;

	// reset the node maps
	for(y=0;y<m_mapsize.height;y++)
	{
		for(x=0;x<m_mapsize.width;x++)
		{
			m_closed_nodes_map[y * m_mapsize.width + x]=0;
			m_open_nodes_map[y * m_mapsize.width + x]=0;
		}
	}

	// create the start node and push into list of open nodes
	n0=new node(xStart, yStart, 0, 0);
	n0->updatePriority(finish.x, finish.y);
	pq[pqi].push(*n0);
	//open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

	// A* search
	while(!pq[pqi].empty())
	{
		// get the current node w/ the highest priority
		// from the list of open nodes
		n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
			pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

		x=n0->getxPos(); y=n0->getyPos();

		pq[pqi].pop(); // remove the node from the open list
		m_open_nodes_map[y * m_mapsize.width + x]=0;
		// mark it on the closed nodes map
		m_closed_nodes_map[y * m_mapsize.width + x]=1;

		// quit searching when the goal state is reached
		//if((*n0).estimate(xFinish, yFinish) == 0)
		if(x==xFinish && y==yFinish) 
		{
			// generate the path from finish to start
			// by following the directions
			string path="";
			while(!(x==xStart && y==yStart))
			{
				j=m_dir_map[y * m_mapsize.width + x];
				c='0'+(j+dir/2)%dir;
				path=c+path;
				x+=m_dx[j];
				y+=m_dy[j];
				result.push_back(cv::Point2d(x,y));
			}

			// garbage collection
			delete n0;
			// empty the leftover nodes
			while(!pq[pqi].empty()) pq[pqi].pop();           

			//
            cv::approxPolyDP(result,result,2,false);

			return result;
		}

		// generate moves (child nodes) in all possible directions
		for(i=0;i<8;i++)
		{
			xdx=x+m_dx[i]; ydy=y+m_dy[i];

			if(!(xdx<0 || xdx> m_mapsize.width - 1 || ydy<0 || ydy>m_mapsize.height -1 || m_map[ydy * m_mapsize.width + xdx]==1 
				|| m_closed_nodes_map[ydy * m_mapsize.width + xdx]==1))
			{
				// generate a child node
				m0=new node( xdx, ydy, n0->getLevel(), 
					n0->getPriority());
				m0->nextLevel(i);
				m0->updatePriority(xFinish, yFinish);

				// if it is not in the open list then add into that
				if(m_open_nodes_map[ydy * m_mapsize.width + xdx]==0)
				{
					m_open_nodes_map[ydy * m_mapsize.width + xdx]=m0->getPriority();
					pq[pqi].push(*m0);
					// mark its parent node direction
					m_dir_map[ydy * m_mapsize.width + xdx]=(i+dir/2)%dir;
				}
				else if(m_open_nodes_map[ydy * m_mapsize.width + xdx]>m0->getPriority())
				{
					// update the priority info
					m_open_nodes_map[ydy * m_mapsize.width + xdx]=m0->getPriority();
					// update the parent direction info
					m_dir_map[ydy * m_mapsize.width + xdx]=(i+dir/2)%dir;

					// replace the node
					// by emptying one pq to the other one
					// except the node to be replaced will be ignored
					// and the new node will be pushed in instead
					while(!(pq[pqi].top().getxPos()==xdx && 
						pq[pqi].top().getyPos()==ydy))
					{                
						pq[1-pqi].push(pq[pqi].top());
						pq[pqi].pop();       
					}
					pq[pqi].pop(); // remove the wanted node

					// empty the larger size pq to the smaller one
					if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
					while(!pq[pqi].empty())
					{                
						pq[1-pqi].push(pq[pqi].top());
						pq[pqi].pop();       
					}
					pqi=1-pqi;
					pq[pqi].push(*m0); // add the better node instead
				}
				else delete m0; // garbage collection
			}
		}
		delete n0; // garbage collection
    }
	return result;
	//return ""; // no route found
}

std::vector<cv::Point> Astar::pathFind_postpro(cv::Point start, cv::Point finish)
{
    std::vector<cv::Point> result;

    priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    int pqi; // pq index
    node* n0;
    node* m0;
    int i, j, x, y, xdx, ydy;
    char c;
    pqi=0;
    int dir = 8;

    int xStart = start.x;
    int yStart = start.y;
    int xFinish = finish.x;
    int yFinish = finish.y;

    // reset the node maps
    for(y=0;y<m_mapsize.height;y++)
    {
        for(x=0;x<m_mapsize.width;x++)
        {
            m_closed_nodes_map[y * m_mapsize.width + x]=0;
            m_open_nodes_map[y * m_mapsize.width + x]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(finish.x, finish.y);
    pq[pqi].push(*n0);
    //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
            pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        m_open_nodes_map[y * m_mapsize.width + x]=0;
        // mark it on the closed nodes map
        m_closed_nodes_map[y * m_mapsize.width + x]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=m_dir_map[y * m_mapsize.width + x];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=m_dx[j];
                y+=m_dy[j];
                result.push_back(cv::Point2d(x,y));
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();


            //
            //cv::approxPolyDP(result,result,2,false);

            if(result.size() > 2)
            {
                bool obj = false;
                for(int ii = 0; ii < result.size(); ii++)
                {
                    if(result.size() <= ii + 2)
                        break;
                    cv::Point p1 = cv::Point(result[ii+2].x, result[ii+2].y);
                    cv::Point p2 = cv::Point(result[ii].x, result[ii].y);

                    double dist = sqrt(pow((double)p1.x - p2.x,2) + pow((double)p1.y - p2.y,2));

                    obj = false;
                    for(int i = 0; i < dist; i++)
                    {
                        cv::Point md_pt = cv::Point( (dist - (double)i) / dist * p1.x + (double)i / dist * p2.x, (dist - (double)i) / dist * p1.y + (double)i / dist * p2.y);

                        if(m_map[md_pt.y * m_mapsize.width + md_pt.x] == 1) // object
                        {
                            obj = true;
                            break;
                        }
                    }
                    if(!obj)
                    {
                        vector<cv::Point>::iterator erase_i = result.begin();
                        result.erase(erase_i + ii + 1);
                        ii--;
                    }


                }
            }

            return result;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<8;i++)
        {
            xdx=x+m_dx[i]; ydy=y+m_dy[i];

            if(!(xdx<0 || xdx> m_mapsize.width - 1 || ydy<0 || ydy>m_mapsize.height -1 || m_map[ydy * m_mapsize.width + xdx]==1
                || m_closed_nodes_map[ydy * m_mapsize.width + xdx]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                    n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(m_open_nodes_map[ydy * m_mapsize.width + xdx]==0)
                {
                    m_open_nodes_map[ydy * m_mapsize.width + xdx]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    m_dir_map[ydy * m_mapsize.width + xdx]=(i+dir/2)%dir;
                }
                else if(m_open_nodes_map[ydy * m_mapsize.width + xdx]>m0->getPriority())
                {
                    // update the priority info
                    m_open_nodes_map[ydy * m_mapsize.width + xdx]=m0->getPriority();
                    // update the parent direction info
                    m_dir_map[ydy * m_mapsize.width + xdx]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                        pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }


    return result;
    //return ""; // no route found
}
