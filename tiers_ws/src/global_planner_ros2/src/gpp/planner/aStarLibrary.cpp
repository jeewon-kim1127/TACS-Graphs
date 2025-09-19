//
//  aStarLibrary.cpp
//  PathPlanning
//
//  Created by 윤필립 on 2016. 11. 22..
//  Copyright (c) 2016년 윤필립. All rights reserved.
//

/*
 ;===================================================================
 ;A* Pathfinder (Version 1.71a) by Patrick Lester. Used by permission.
 ;===================================================================
 ;Last updated 06/16/03 -- Visual C++ version
 */

#include "aStarLibrary.h"
#include "gridlinetraversal.h"

#ifdef _WIN32_WCE
#define DEBUG_MSG RemoteLog0
//#define DEBUG_MSG ((void)0)
#else
#define DEBUG_MSG TRACE
#endif
//-----------------------------------------------------------------------------
// Name: InitializePathfinder
// Desc: Allocates memory for the pathfinder.
//-----------------------------------------------------------------------------
CAStar::CAStar()
{
    pathLength = 0;
    pathStatus = notStarted;
    onClosedList = 10;
    pathLocation = 0;
    openList  = NULL;
    openX	  = NULL;
    openY	  = NULL;
    Fcost	  = NULL;
    Hcost	  = NULL;
    pathBank  = NULL;
    
    whichList = NULL;
    parentX   = NULL;
    parentY   = NULL;
    Gcost     = NULL;
    
    mapWidthMax  = mapHeightMax = 0;
    nTotalBufSize = 0;
}

int CAStar::GetMemorySize()
{
    return nTotalBufSize;
}

char *CAStar::GetInternalBuffer()
{
    return pTotalBuffer;
}

int CAStar::CalcMemorySize(int width, int height)
{
    int shortStep = (mapWidthMax+1)*sizeof(short);
    
    int size2DBuf = shortStep*(mapHeightMax+1);
    int sizeOpenL = ( mapWidthMax*mapHeightMax+2 ) * sizeof(int  );
    int sizeRest  = ( mapWidthMax*mapHeightMax+2 ) * sizeof(short);
    
    return
    size2DBuf * 4 +
    sizeOpenL * sizeof(int  ) +
    sizeRest  * sizeof(short) * 4;
}

template <class T>
T ** vipMake2DPtr(T *src, int step, int width, int height)
{
    T **ppRet = new T* [height];
    
    for (int y = 0 ; y < height ; y++, (char*&)src += step)
        ppRet[y] = src;
    
    return ppRet;
}

bool CAStar::Initialize(int width, int height)
{
    if (mapWidthMax<width || mapHeightMax<height)
    {
        Release();
        
        mapWidthMax  = std::max(mapWidthMax , width );
        mapHeightMax = std::max(mapHeightMax, height);
        
        //2 dimensional array used to record
        //	whether a cell is on the open list or on the closed list.
        
        int shortStep = (mapWidthMax+1)*sizeof(short);
        
        int size2DBuf = shortStep*(mapHeightMax+1);
        int sizeOpenL = ( mapWidthMax*mapHeightMax+2 ) * sizeof(int  );
        int sizeRest  = ( mapWidthMax*mapHeightMax+2 ) * sizeof(short);
        
        nTotalBufSize =
        size2DBuf * 4 +
        sizeOpenL * sizeof(int  ) +
        sizeRest  * sizeof(short) * 4;
        
        pTotalBuffer = new char[nTotalBufSize];
        if (!pTotalBuffer)
        {
            nTotalBufSize = 0;
            return false;
        }
        
        char *buf = pTotalBuffer;
        
        ushort * buf_whichList = (ushort*)buf; buf += size2DBuf;
        ushort * buf_parentX   = (ushort*)buf; buf += size2DBuf;
        ushort * buf_parentY   = (ushort*)buf; buf += size2DBuf;
        ushort * buf_Gcost     = (ushort*)buf; buf += size2DBuf;
        
        openList = (unsigned int  *)buf; buf += sizeOpenL; //1 dimensional array holding ID# of open list items
        openX    = (ushort*)buf; buf += sizeRest ; //1d array stores the x location of an item on the open list
        openY    = (ushort*)buf; buf += sizeRest ; //1d array stores the y location of an item on the open list
        Fcost    = (ushort*)buf; buf += sizeRest ; //1d array to store F cost of a cell on the open list
        Hcost    = (ushort*)buf; buf += sizeRest ; //1d array to store H cost of a cell on the open list
        
        whichList = vipMake2DPtr(buf_whichList, shortStep, mapWidthMax+1, mapHeightMax+1); // short
        parentX   = vipMake2DPtr(buf_parentX  , shortStep, mapWidthMax+1, mapHeightMax+1); // short
        parentY   = vipMake2DPtr(buf_parentY  , shortStep, mapWidthMax+1, mapHeightMax+1); // short
        Gcost     = vipMake2DPtr(buf_Gcost    , shortStep, mapWidthMax+1, mapHeightMax+1); // short

        memset(Fcost, 0, (mapWidthMax*mapHeightMax+2)*sizeof(short));
        
        pathBank = (int*) malloc(4);
    }
    
    _ResetWhichListBuf();
    pathLength = 0;
    pathStatus = notStarted;
    onClosedList = 10;
    pathLocation = 0;
    
    return true;
}

void CAStar::_ResetWhichListBuf()
{
    memset(whichList[0], 0, (mapHeightMax+1)*(mapWidthMax+1)*sizeof(whichList[0][0]));
}

//-----------------------------------------------------------------------------
// Name: EndPathfinder
// Desc: Frees memory used by the pathfinder.
//-----------------------------------------------------------------------------
void CAStar::Release()
{
    if (nTotalBufSize)
    {
        nTotalBufSize = 0;
        delete [] pTotalBuffer;
        
        delete [] whichList;
        delete [] parentX  ;
        delete [] parentY  ;
        delete [] Gcost    ;
    }
    
    if(pathBank) free (pathBank)   ; pathBank = NULL;
    mapWidthMax  = mapHeightMax = 0;
}

int CAStar::GetCost()
{
    return m_nCost;
}

/** A*∑Œ robot->goal±Ó¡ˆ¿« Shortest Path∏¶ √£¥¬¥Ÿ.
 *
 *- iPoint robot: robot¿« «ˆ¿Á ¿ßƒ° (x, y)
 *- iPoint goal: ∏Ò¿˚¡ˆ¿« ¿ßƒ° (x, y)
 *- iPoint *viaPt: √£æ∆¡¯ via Point array
 *- int &size: √£æ∆¡¯ via Point¿« ∞πºˆ
 *- bool bPreventCross: ¥Î∞¢º±¿∏∑Œ ∞•∂ß ¡¬/øÏ≥™ ªÛ/«œ∞° ∏∑«Ù¿÷¿ª ∂ß ∞°¡ˆ ∏¯«œ∞‘ «‘
 *- bool bShortPath: √£æ∆¡¯ via PointµÈ ∞£ø° ¿Âæ÷π∞¿Ã æ¯¿ªΩ√ ¿ÃæÓ¡‹.
 */
static int sq_sum(iPoint &src) { return src.x*src.x + src.y*src.y; }
inline int isqrt(int y)
{
    int     x_old, x_new;
    int		testy;
    int     nbits;
    int     i;
    
    if (y <= 0)
    {
        if (y != 0)
        {
            printf("Domain error in lsqrt().\n");
            return -1;
        }
        return 0;
    }
    // select a good starting value using binary logarithms:
    nbits = (sizeof(y) * 8) - 1;    // subtract 1 for sign bit
    for (i = 4, testy = 16;; i += 2, testy <<= 2)
    {
        if (i >= nbits || y <= testy)
        {
            x_old = (1 << (i / 2));       // x_old = sqrt(testy)
            break;
        }
    }
    // x_old >= sqrt(y)
    // use the Babylonian method to arrive at the integer square root:
    for (;;)
    {
        x_new = (y / x_old + x_old) / 2;
        if (x_old <= x_new)
            break;
        x_old = x_new;
    }
    // make sure that the answer is right:
    if ((unsigned int) x_old * x_old > y || ((unsigned int) x_old + 1) * ((unsigned int) x_old + 1) <= y)
    {
        printf("Error in lsqrt().\n");
        return -1;
    }
    return x_old;
}

bool CAStar::FindAStarPath(iPoint robot, iPoint goal, iPointArray& viaPt, int &size, bool bPreventCross, bool bShortPath)
{
    pathStatus = FindPath(robot.x, robot.y, goal.x, goal.y, bPreventCross);
    m_nCost*= 10;
    
    if (pathLength > size || pathStatus==nonexistent)
        return false;
    
    size = pathLength;
    
    
    for(int i=0; i<pathLength; i++)
    {
        viaPt[i].x = pathBank[2*i  ];
        viaPt[i].y = pathBank[2*i+1];
        //printf("A* via[%d]=%d %d\n", i, viaPt[i].x, viaPt[i].y);
    }
    
    // Modified by MH 2007.12.13.
    if (bShortPath)
    {
        for(int i=pathLength; i>0; i--)
        {
            viaPt[i].x = viaPt[i-1].x;
            viaPt[i].y = viaPt[i-1].y;
        }
        viaPt[0].x = robot.x;
        viaPt[0].y = robot.y;
        pathLength++;
        pathLength = GetShortestPath(gridMap, gridStep, mapWidth, mapHeight, viaPt, pathLength);
        pathLength--;
        size = pathLength;
        m_nCost = 0;
        for(int i=0; i<pathLength; i++)
        {
            iPoint temp;
            temp.x = viaPt[i].x - viaPt[i+1].x;
            temp.y = viaPt[i].y - viaPt[i+1].y;
            
            m_nCost+= isqrt(10000*sq_sum(temp));
            viaPt[i].x = viaPt[i+1].x;
            viaPt[i].y = viaPt[i+1].y;
            printf("SA* via[%d]=%d %d\n", i, viaPt[i].x, viaPt[i].y);
        }
        printf("Cost=%d\n", m_nCost);
    }
    
    return true;
}

int CAStar::GetShortPath(iPointArray& viaPt)
{
    pathLength = GetShortestPath(gridMap, gridStep, mapWidth, mapHeight, viaPt, pathLength);
    
    return pathLength;
}

/** ∏Ò¿˚¡ˆ∞° ¡÷æÓ¡ˆ¡ˆ æ æ“¿ª ∂ß robot ¿ßƒ°ø°º≠ ¿œ¡§ πÆ≈Œƒ° ∞≈∏Æ∏¶ ≥—æÓ∞°¥¬ √÷¿˚ path∏¶ ª˝º∫
 *
 *- iPoint robot: ∑Œ∫ø¿« «ˆ¿Á ¿ßƒ° (x, y)
 *- int dist: πÆ≈Œƒ° ∞≈∏Æ (cost∑Œ ¡÷æÓ¡¸. 1grid=10, ¥Î∞¢º±=14)
 *- iPoint *viaPt: √£æ∆¡¯ via Point array
 *- int &size: √£æ∆¡¯ via Point¿« ∞πºˆ
 *- bool bPreventCross: ¥Î∞¢º±¿∏∑Œ ∞•∂ß ¡¬/øÏ≥™ ªÛ/«œ∞° ∏∑«Ù¿÷¿ª ∂ß ∞°¡ˆ ∏¯«œ∞‘ «‘
 *- bool bShortPath: √£æ∆¡¯ via PointµÈ ∞£ø° ¿Âæ÷π∞¿Ã æ¯¿ªΩ√ ¿ÃæÓ¡‹.
 */
bool CAStar::FindAStarFrontier(iPoint robot, int dist, iPoint *viaPt, int &size, bool bPreventCross, bool bShortPath)
{
    pathStatus = FindFrontier(robot.x, robot.y, dist, bPreventCross);
    m_nCost*= 10;
    
    if (pathLength > size || pathStatus==nonexistent)
        return false;
    
    size = pathLength;
    
    
    for(int i=0; i<pathLength; i++)
    {
        viaPt[i].x = pathBank[2*i  ];
        viaPt[i].y = pathBank[2*i+1];
    }
    
    // Modified by MH 2007.12.13.
    if (bShortPath)
    {
        for(int i=pathLength; i>0; i--)
        {
            viaPt[i].x = viaPt[i-1].x;
            viaPt[i].y = viaPt[i-1].y;
        }
        viaPt[0].x = robot.x;
        viaPt[0].y = robot.y;
        pathLength++;
        //pathLength = GetShortestPath(gridMap, gridStep, mapWidth, mapHeight, viaPt, pathLength);
        pathLength--;
        size = pathLength;
        m_nCost = 0;
        for(int i=0; i<pathLength; i++)
        {
            iPoint temp;
            temp.x = viaPt[i].x - viaPt[i+1].x;
            temp.y = viaPt[i].y - viaPt[i+1].y;
            
            m_nCost+= isqrt(10000*sq_sum(temp));
            viaPt[i].x = viaPt[i+1].x;
            viaPt[i].y = viaPt[i+1].y;
            printf("SA* via[%d]=%d %d\n", i, viaPt[i].x, viaPt[i].y);
        }
        printf("Cost=%d\n", m_nCost);
    }
    
    return true;
}


int GetShortestPath(const BYTE *csMap, int csStep, int width, int height, iPointArray& viaPt, int viaSize)
{
    //-----------------------------------------------------------------------------//
    // date : 2006/8/27
    //
    // input :
    //
    // output :
    //		- √÷º“ ∞πºˆ¿« via point
    // function :
    //		- CheckObstacle
    // used global :
    //		- m_numShortViaPt
    //		- m_numViaPt
    //		- m_pNodeGrids
    //		- m_shortViaPtArray
    //		- m_numShortViaPt
    // ±‚¥… :
    //		- ¿Âæ÷π∞ø° ∞°∏Æ¡ˆ æ»»§ √÷¥‹ ∞≈∏Æ∑Œ ∞• ºˆ ¿÷∞‘ «œ¥¬ via pointµÈ¿ª ªÃ¥¬¥Ÿ.
    //-----------------------------------------------------------------------------//
    
    int i,j,k, tmp;
    
    int numShortViaPt = viaSize;
    iPoint *shortViaPt = new iPoint[viaSize];
    
    // ∏µÁ via pointø° ¥Î«ÿ m_shortViaPtArrayø° m_viaPtArray∞™¿ª ¿˙¿Â«—¥Ÿ.
    ///*
    for ( int i = 0; i < numShortViaPt; i++ )
        shortViaPt[i].x = viaPt[i].x, shortViaPt[i].y = viaPt[i].y;
    //memcpy(shortViaPt, viaPt.data(), numShortViaPt*sizeof(iPoint));
    
    //*/
    
    // ∏µÁ via pointø° ¥Î«ÿ
    // ex) m_numShortViaPt == 10¿Ã∏È 0~9±Ó¡ˆ
    for(i=0	; i<numShortViaPt ; i++)
    {
        // ex) m_numShortViaPt == 10¿Ã∞Ì, i == 3¿Ã∏È 9~4±Ó¡ˆ
        for(j=numShortViaPt-1 ; j>i ; j--)
        {
            // µŒ via point ªÁ¿Ãø° ¿Âæ÷π∞¿Ã æ¯¥Ÿ∏È
            if (!CheckObstacle(csMap, csStep, width, height, shortViaPt[i], shortViaPt[j]))
            {
                // µŒ via point ªÁ¿Ã¿« via pointµÈ¿ª m_shortViaPtArrayø°º≠ ªË¡¶«ÿ ¡ÿ¥Ÿ.
                tmp = j;
                
                for(k = i+1 ; k<numShortViaPt && tmp<numShortViaPt; k++)
                {
                    shortViaPt[k] = viaPt[tmp];
                    tmp++;
                }
                
                // ªË¡¶µ» via point ∞≥ºˆ∏∏≈≠ m_numShortViaPt∏¶ ∞®º“Ω√≈≤¥Ÿ.
                numShortViaPt -= (j-i-1);
                
                for ( int k = 0; k < numShortViaPt; k++ )
                    viaPt[k].x = shortViaPt[k].x, viaPt[k].y = shortViaPt[k].y;
                
                //memcpy(viaPt.data(), shortViaPt, numShortViaPt*sizeof(iPoint));
                break;
            }
        }
    }
    
    for ( int i = 0; i < numShortViaPt; i++ )
        viaPt[i].x = shortViaPt[i].x, viaPt[i].y = shortViaPt[i].y;
//    memcpy(viaPt.data(), shortViaPt, numShortViaPt*sizeof(iPoint));
    
    delete[] shortViaPt;
    return numShortViaPt;
    
}
//========================================================================================================================================//

// GetShortestPath()ø°º≠
//

// 2007.03.21, by MH (corner∞° ∏∑«Ù¿÷¿∏∏È ∏¯ ¡ˆ≥™∞°µµ∑œ...)
bool CheckObstacle(const BYTE *csMap, int csStep, int width, int height, iPoint p0, iPoint p1)
{
    GridLineTraversalLine line;
    line.points = new iPoint[1000];
    GridLineTraversal::gridLine(p0, p1, &line);
    
    if (line.num_points > 900)
        printf("Critical Error line.num_points = %d\n", line.num_points);
    
    int x, y, parentXval, parentYval, corner;
    
    bool bRet = false;
    
    /*FILE *out = fopen("output.out","w");
    for ( int i = 0; i < height; i++ )
    {
        for ( int j = 0; j < width; j++ )
        {
            if ( !(*(csMap + (i * width) + j)))
                fprintf(stdout,"%d %d\n", i, j);
        }
    }*/
    
    //printf("step : %d width : %d\n", csStep, width);
    for (int i=0; i<line.num_points; i++)
    {
        x = line.points[i].x;
        y = line.points[i].y;
        //printf("%d %d %d\n", x, y, (csMap+y*width)[x]);
        
        if(!(csMap+y*width)[x])
        {
            bRet = true;
            break;
        }
        
        corner = walkable;
        
        if (!i)
        {
            parentXval = x;
            parentYval = y;
            continue;
        }
        
        if (corner == unwalkable)
        {
            bRet = true;
            break;
        }
        parentXval = x;
        parentYval = y;
    }
    
    delete [] line.points;
    return bRet;
}

/** Configuration Space Map¿ª ºº∆√«‘
 *
 *- const BYTE *csMap: Byte ¥‹¿ß¿« gridmap image
 *- int csStep: gridmap image step
 *- int width, int height: gridmap image¿« width, height
 */
bool CAStar::SetCSMap (const BYTE *csMap, int csStep, int width, int height)
{
    printf("[Astar]SetCSMap [%d %d]max [%d %d]\n", mapWidthMax, mapHeightMax, width, height);
    if (width==0 || height==0) return false;
    if (width  > mapWidthMax ) return false;
    if (height > mapHeightMax) return false;
    
    gridMap   = csMap;
    gridStep  = csStep;
    
    mapWidth  = width;
    mapHeight = height;
    
    printf("[SetCSMap] %d %d\n", width, height);
    
    return true;
}


//-----------------------------------------------------------------------------
// Name: FindPath
// Desc: Finds a path using A*
//-----------------------------------------------------------------------------
int CAStar::FindPath (int startingX, int startingY, int targetX, int targetY, bool bPreventCross)
{
    int onOpenList=0, parentXval=0, parentYval=0,
    a=0, b=0, m=0, u=0, v=0, temp=0, numberOfOpenListItems=0,
    addedGCost=0, tempGcost = 0, path = 0,
    tempx, pathX, pathY, cellPosition,
    newOpenListItemID=0;
    
    //1. Convert location data (in pixels) to coordinates in the gridMap array.
    int startX = startingX/tileSize;
    int startY = startingY/tileSize;
    targetX /= tileSize;
    targetY /= tileSize;
    
    //2.Quick Path Checks: Under the some circumstances no path needs to
    //	be generated ...
    
    //	If starting location and target are in the same location...
    if (startX == targetX && startY == targetY && pathLocation >  0) return founded;
    if (startX == targetX && startY == targetY && pathLocation == 0) return nonexistent;
    
    //	If target square is unwalkable, return that it's a nonexistent path.
    
    printf("%d %d %d %d\n", targetX, targetY, mapWidth, mapHeight);
    if ((gridMap+targetY*gridStep)[targetX] == unwalkable)
        goto noPath;
    
    //3.Reset some variables that need to be cleared
    //if (onClosedList >= USHRT_MAX) //reset whichList occasionally
    {
        _ResetWhichListBuf();
        //onClosedList = 10;
    }
    onClosedList	= 2;//onClosedList+2; //changing the values of onOpenList and onClosed list is faster than redimming whichList() array
    onOpenList		= 1;//onClosedList-1;
    pathLength		= notStarted;//i.e, = 0
    pathLocation	= notStarted;//i.e, = 0
    Gcost[startY][startX] = 0; //reset starting square's G value to 0
    
    //4.Add the starting location to the open list of squares to be checked.
    numberOfOpenListItems = 1;
    openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
    openX[1] = startX ; openY[1] = startY;
    
    //5.Do the following until a path is founded or deemed nonexistent.
    do
    {
        //6.If the open list is not empty, take the first cell off of the list.
        //	This is the lowest F cost cell on the open list.
        if (numberOfOpenListItems != 0)
        {
            //7. Pop the first item off the open list.
            parentXval = openX[openList[1]];
            parentYval = openY[openList[1]]; //record cell coordinates of the item
            whichList[parentYval][parentXval] = onClosedList;//add the item to the closed list
            //printf("(%d, %d)\n", parentXval, parentYval);
            
            //	Open List = Binary Heap: Delete this item from the open list, which
            //  is maintained as a binary heap. For more information on binary heaps, see:
            //	http://www.policyalmanac.org/games/binaryHeaps.htm
            numberOfOpenListItems--;//reduce number of open list items by 1
            
            //	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
            openList[1] = openList[numberOfOpenListItems+1];//move the last item in the heap up to slot #1
            v = 1;
            
            //	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.
            do
            {
                u = v;
                if (2*u+1 <= numberOfOpenListItems) //if both children exist
                {
                    //Check if the F cost of the parent is greater than each child.
                    //Select the lowest of the two children.
                    if (Fcost[openList[u]] >= Fcost[openList[2*u  ]]) v = 2*u  ;
                    if (Fcost[openList[v]] >= Fcost[openList[2*u+1]]) v = 2*u+1;
                }
                else
                {
                    if (2*u <= numberOfOpenListItems) //if only child #1 exists
                    {
                        //Check if the F cost of the parent is greater than child #1
                        if (Fcost[openList[u]] >= Fcost[openList[2*u]])
                            v = 2*u;
                    }
                }
                
                if (u != v) //if parent's F is > one of its children, swap them
                {
                    temp = openList[u];
                    openList[u] = openList[v];
                    openList[v] = temp;
                }
                else
                    break; //otherwise, exit loop
                
            }
            while(1); //while (!KeyDown(27));//reorder the binary heap
            
            
            //7.Check the adjacent squares. (Its "children" -- these path children
            //	are similar, conceptually, to the binary heap children mentioned
            //	above, but don't confuse them. They are different. Path children
            //	are portrayed in Demo 1 with grey pointers pointing toward
            //	their parents.) Add these adjacent child squares to the open list
            //	for later consideration if appropriate (see various if statements
            //	below).
            for (b = parentYval-1; b <= parentYval+1; b++) // -1 0 1
            {
                if ( b == -1 || b == mapHeight ) continue;
                
                const BYTE   *pGrid    = gridMap+b*gridStep;
                unsigned short *pWhich   = whichList[b];
                unsigned short *pParentX = parentX  [b];
                unsigned short *pParentY = parentY  [b];
                unsigned short *pGCost   = Gcost    [b];
                
                for (a = parentXval-1; a <= parentXval+1; a++) // -1 0 1
                {
                    if ( a == -1 || a == mapWidth ) continue; // roi check
                    
                    //	If not already on the closed list (items on the closed list have
                    //	already been considered and can now be ignored).
                    if (pWhich[a] == onClosedList) continue;
                    
                    //	If not a wall/obstacle square.
                    if (pGrid[a] == unwalkable) continue;
                    
                    /*
                    if (bPreventCross)
                    {
                        //	Don't cut across corners
                        if (a == parentXval-1)
                        {
                            if      (b == parentYval-1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval-1, parentYval  ) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval-1) == unwalkable)
                                    continue;
                            }
                            else if (b == parentYval+1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval+1) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval-1, parentYval  ) == unwalkable)
                                    continue;
                            }
                        }
                        else if (a == parentXval+1)
                        {
                            if (b == parentYval-1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval-1) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval+1, parentYval  ) == unwalkable)
                                    continue;
                            }
                            else if (b == parentYval+1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval+1, parentYval  ) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval+1) == unwalkable)
                                    continue;
                            }
                        }
                    }
                     */
                    
                    //	If not already on the open list, add it to the open list.
                    if (pWhich[a] != onOpenList)
                    {
                        
                        //Create a new open list item in the binary heap.
                        newOpenListItemID++; //each new item has a unique ID #
                        m = numberOfOpenListItems+1;
                        openList[m] = newOpenListItemID;//place the new open list item (actually, its ID#) at the bottom of the heap
                        openX[newOpenListItemID] = a;
                        openY[newOpenListItemID] = b;//record the x and y coordinates of the new item
                        
                        //Figure out its G cost
                        if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
                            addedGCost = 14;//cost of going to diagonal squares
                        else
                            addedGCost = 10;//cost of going to non-diagonal squares
                        pGCost[a] = Gcost[parentYval][parentXval] + addedGCost;
                        
                        //Figure out its H and F costs and parent
                        Hcost[openList[m]] = 10*(abs(a - targetX) + abs(b - targetY)); // 4?
                        Fcost[openList[m]] = pGCost[a] + Hcost[openList[m]];
                        pParentX[a] = parentXval;
                        pParentY[a] = parentYval;
                        
                        //Move the new open list item to the proper place in the binary heap.
                        //Starting at the bottom, successively compare to parent items,
                        //swapping as needed until the item finds its place in the heap
                        //or bubbles all the way to the top (if it has the lowest F cost).
                        while (m != 1) //While item hasn't bubbled to the top (m=1)
                        {
                            //Check if child's F cost is < parent's F cost. If so, swap them.
                            if (Fcost[openList[m]] <= Fcost[openList[m/2]])
                            {
                                temp = openList[m/2];
                                openList[m/2] = openList[m];
                                openList[m] = temp;
                                m = m/2;
                            }
                            else
                                break;
                        }
                        numberOfOpenListItems++;//add one to the number of items in the heap
                        
                        //Change whichList to show that the new item is on the open list.
                        pWhich[a] = onOpenList;
                    }
                    
                    //8.If adjacent cell is already on the open list, check to see if this
                    //	path to that cell from the starting location is a better one.
                    //	If so, change the parent of the cell and its G and F costs.
                    else //If whichList(b,a) = onOpenList
                    {
                        
                        //Figure out the G cost of this possible new path
                        if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
                            addedGCost = 14;//cost of going to diagonal tiles
                        else
                            addedGCost = 10;//cost of going to non-diagonal tiles
                        tempGcost = Gcost[parentYval][parentXval] + addedGCost;
                        
                        //If this path is shorter (G cost is lower) then change
                        //the parent cell, G cost and F cost.
                        if (tempGcost < pGCost[a]) //if G cost is less,
                        {
                            pParentX[a] = parentXval; //change the square's parent
                            pParentY[a] = parentYval;
                            pGCost  [a] = tempGcost;//change the G cost
                            
                            //Because changing the G cost also changes the F cost, if
                            //the item is on the open list we need to change the item's
                            //recorded F cost and its position on the open list to make
                            //sure that we maintain a properly ordered open list.
                            for (int x = 1; x <= numberOfOpenListItems; x++) //look for the item in the heap
                            {
                                if (openX[openList[x]] == a && openY[openList[x]] == b) //item founded
                                {
                                    Fcost[openList[x]] = pGCost[a] + Hcost[openList[x]];//change the F cost
                                    
                                    //See if changing the F score bubbles the item up from it's current location in the heap
                                    m = x;
                                    while (m != 1) //While item hasn't bubbled to the top (m=1)
                                    {
                                        //Check if child is < parent. If so, swap them.
                                        if (Fcost[openList[m]] < Fcost[openList[m/2]])
                                        {
                                            temp = openList[m/2];
                                            openList[m/2] = openList[m];
                                            openList[m] = temp;
                                            m = m/2;
                                        }
                                        else
                                            break;
                                    }
                                    break; //exit for x = loop
                                } //If openX(openList(x)) = a
                            } //For x = 1 To numberOfOpenListItems
                        }//If tempGcost < Gcost(a,b)
                    }//else If whichList(b,a) = onOpenList
                    
                    
                } //for (a = parentXval-1; a <= parentXval+1; a++){
            } //for (b = parentYval-1; b <= parentYval+1; b++){
            
        }//if (numberOfOpenListItems != 0)
        
        //9.If open list is empty then there is no path.
        else
        {
            path = nonexistent; break;
        }
        
        //If target is added to open list then path has been founded.
        if (whichList[targetY][targetX] == onOpenList)
        {
            m_nCost = Fcost[openList[1]];
            path = founded; break;
        }
        
    }
    while (1);//Do until path is founded or deemed nonexistent
    
    //10.Save the path if it exists.
    if (path == founded)
    {
        pathStatus = founded;
        //a.Working backwards from the target to the starting location by checking
        //	each cell's parent, figure out the length of the path.
        pathX = targetX; pathY = targetY;
        do
        {
            //Look up the parent of the current cell.
            tempx = parentX[pathY][pathX];
            pathY = parentY[pathY][pathX];
            pathX = tempx;
            
            //Figure out the path length
            pathLength = pathLength + 1;
        }
        while (pathX != startX || pathY != startY);
        
        //b.Resize the data bank to the right size in bytes
        pathBank = (int*) realloc (pathBank, pathLength*8);
        
        //c. Now copy the path information over to the databank. Since we are
        //	working backwards from the target to the start location, we copy
        //	the information to the data bank in reverse order. The result is
        //	a properly ordered set of path data, from the first step to the
        //	last.
        pathX = targetX ; pathY = targetY;
        cellPosition = pathLength*2;//start at the end
        do
        {
            cellPosition = cellPosition - 2;//work backwards 2 integers
            pathBank  [cellPosition] = pathX;
            pathBank[cellPosition+1] = pathY;
            
            //d.Look up the parent of the current cell.
            tempx = parentX[pathY][pathX];
            pathY = parentY[pathY][pathX];
            pathX = tempx;
            
            //e.If we have reached the starting square, exit the loop.
        }
        while (pathX != startX || pathY != startY);
        
        //11.Read the first path step into xPath/yPath arrays
        ReadPath(startingX,startingY,1);
        
    }
    return path;
    
    
    //13.If there is no path to the selected target, set the pathfinder's
    //	xPath and yPath equal to its current location and return that the
    //	path is nonexistent.
noPath:
    xPath = startingX;
    yPath = startingY;
    return nonexistent;
}


int CAStar::FindFrontier (int startingX, int startingY, int dist, bool bPreventCross)
{
    int onOpenList=0, parentXval=0, parentYval=0,
    a=0, b=0, m=0, u=0, v=0, temp=0, numberOfOpenListItems=0,
    addedGCost=0, tempGcost = 0, path = 0,
    tempx, pathX, pathY, cellPosition,
    newOpenListItemID=0;
    
    int targetX, targetY;
    
    //1. Convert location data (in pixels) to coordinates in the gridMap array.
    int startX = startingX/tileSize;
    int startY = startingY/tileSize;
    
    //2.Quick Path Checks: Under the some circumstances no path needs to
    //	be generated ...
    
    //	If starting location and target are in the same location...
    if (dist == 0) return founded;
    
    //	If target square is unwalkable, return that it's a nonexistent path.
    
    //3.Reset some variables that need to be cleared
    //if (onClosedList >= USHRT_MAX) //reset whichList occasionally
    {
        _ResetWhichListBuf();
        //onClosedList = 10;
    }
    onClosedList	= 2;//onClosedList+2; //changing the values of onOpenList and onClosed list is faster than redimming whichList() array
    onOpenList		= 1;//onClosedList-1;
    pathLength		= notStarted;//i.e, = 0
    pathLocation	= notStarted;//i.e, = 0
    Gcost[startY][startX] = 0; //reset starting square's G value to 0
    
    //4.Add the starting location to the open list of squares to be checked.
    numberOfOpenListItems = 1;
    openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
    openX[1] = startX ; openY[1] = startY;
    
    //5.Do the following until a path is founded or deemed nonexistent.
    do
    {
        //6.If the open list is not empty, take the first cell off of the list.
        //	This is the lowest F cost cell on the open list.
        if (numberOfOpenListItems != 0)
        {
            //7. Pop the first item off the open list.
            parentXval = openX[openList[1]];
            parentYval = openY[openList[1]]; //record cell coordinates of the item
            targetX    = parentXval;
            targetY    = parentYval;
            whichList[parentYval][parentXval] = onClosedList;//add the item to the closed list
            
            //	Open List = Binary Heap: Delete this item from the open list, which
            //  is maintained as a binary heap. For more information on binary heaps, see:
            //	http://www.policyalmanac.org/games/binaryHeaps.htm
            numberOfOpenListItems--;//reduce number of open list items by 1
            
            //	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
            openList[1] = openList[numberOfOpenListItems+1];//move the last item in the heap up to slot #1
            v = 1;
            
            //	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.
            do
            {
                u = v;
                if (2*u+1 <= numberOfOpenListItems) //if both children exist
                {
                    //Check if the F cost of the parent is greater than each child.
                    //Select the lowest of the two children.
                    if (Fcost[openList[u]] >= Fcost[openList[2*u  ]]) v = 2*u  ;
                    if (Fcost[openList[v]] >= Fcost[openList[2*u+1]]) v = 2*u+1;
                }
                else
                {
                    if (2*u <= numberOfOpenListItems) //if only child #1 exists
                    {
                        //Check if the F cost of the parent is greater than child #1
                        if (Fcost[openList[u]] >= Fcost[openList[2*u]])
                            v = 2*u;
                    }
                }
                
                if (u != v) //if parent's F is > one of its children, swap them
                {
                    temp = openList[u];
                    openList[u] = openList[v];
                    openList[v] = temp;
                }
                else
                    break; //otherwise, exit loop
                
            }
            while(1); //while (!KeyDown(27));//reorder the binary heap
            
            
            //7.Check the adjacent squares. (Its "children" -- these path children
            //	are similar, conceptually, to the binary heap children mentioned
            //	above, but don't confuse them. They are different. Path children
            //	are portrayed in Demo 1 with grey pointers pointing toward
            //	their parents.) Add these adjacent child squares to the open list
            //	for later consideration if appropriate (see various if statements
            //	below).
            for (b = parentYval-1; b <= parentYval+1; b++) // -1 0 1
            {
                if ( b == -1 || b == mapHeight ) continue;
                
                const BYTE   *pGrid    = gridMap+b*gridStep;
                unsigned short *pWhich   = whichList[b];
                unsigned short *pParentX = parentX  [b];
                unsigned short *pParentY = parentY  [b];
                unsigned short *pGCost   = Gcost    [b];
                
                for (a = parentXval-1; a <= parentXval+1; a++) // -1 0 1
                {
                    if ( a == -1 || a == mapWidth ) continue; // roi check
                    
                    //	If not already on the closed list (items on the closed list have
                    //	already been considered and can now be ignored).
                    if (pWhich[a] == onClosedList) continue;
                    
                    //	If not a wall/obstacle square.
                    if (pGrid[a] == unwalkable) continue;
                    
                    /*
                    if (bPreventCross)
                    {
                        //	Don't cut across corners
                        if (a == parentXval-1)
                        {
                            if      (b == parentYval-1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval-1, parentYval  ) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval-1) == unwalkable)
                                    continue;
                            }
                            else if (b == parentYval+1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval+1) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval-1, parentYval  ) == unwalkable)
                                    continue;
                            }
                        }
                        else if (a == parentXval+1)
                        {
                            if (b == parentYval-1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval-1) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval+1, parentYval  ) == unwalkable)
                                    continue;
                            }
                            else if (b == parentYval+1)
                            {
                                if (*VIP_PTR(BYTE, gridMap, gridStep, parentXval+1, parentYval  ) == unwalkable ||
                                    *VIP_PTR(BYTE, gridMap, gridStep, parentXval  , parentYval+1) == unwalkable)
                                    continue;
                            }
                        }
                    }
                     */
                    
                    //	If not already on the open list, add it to the open list.
                    if (pWhich[a] != onOpenList)
                    {
                        
                        //Create a new open list item in the binary heap.
                        newOpenListItemID++; //each new item has a unique ID #
                        m = numberOfOpenListItems+1;
                        openList[m] = newOpenListItemID;//place the new open list item (actually, its ID#) at the bottom of the heap
                        openX[newOpenListItemID] = a;
                        openY[newOpenListItemID] = b;//record the x and y coordinates of the new item
                        
                        //Figure out its G cost
                        if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
                            addedGCost = 14;//cost of going to diagonal squares
                        else
                            addedGCost = 10;//cost of going to non-diagonal squares
                        pGCost[a] = Gcost[parentYval][parentXval] + addedGCost;
                        
                        //Figure out its H and F costs and parent
                        //Hcost[openList[m]] = 10*(abs(a - targetX) + abs(b - targetY)); // 4?
                        Fcost[openList[m]] = pGCost[a];// + Hcost[openList[m]];
                        pParentX[a] = parentXval;
                        pParentY[a] = parentYval;
                        
                        //Move the new open list item to the proper place in the binary heap.
                        //Starting at the bottom, successively compare to parent items,
                        //swapping as needed until the item finds its place in the heap
                        //or bubbles all the way to the top (if it has the lowest F cost).
                        while (m != 1) //While item hasn't bubbled to the top (m=1)
                        {
                            //Check if child's F cost is < parent's F cost. If so, swap them.
                            if (Fcost[openList[m]] <= Fcost[openList[m/2]])
                            {
                                temp = openList[m/2];
                                openList[m/2] = openList[m];
                                openList[m] = temp;
                                m = m/2;
                            }
                            else
                                break;
                        }
                        numberOfOpenListItems++;//add one to the number of items in the heap
                        
                        //Change whichList to show that the new item is on the open list.
                        pWhich[a] = onOpenList;
                    }
                    
                    //8.If adjacent cell is already on the open list, check to see if this 
                    //	path to that cell from the starting location is a better one. 
                    //	If so, change the parent of the cell and its G and F costs.	
                    else //If whichList(b,a) = onOpenList
                    {
                        
                        //Figure out the G cost of this possible new path
                        if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
                            addedGCost = 14;//cost of going to diagonal tiles	
                        else	
                            addedGCost = 10;//cost of going to non-diagonal tiles				
                        tempGcost = Gcost[parentYval][parentXval] + addedGCost;
                        
                        //If this path is shorter (G cost is lower) then change
                        //the parent cell, G cost and F cost. 		
                        if (tempGcost < pGCost[a]) //if G cost is less,
                        {
                            pParentX[a] = parentXval; //change the square's parent
                            pParentY[a] = parentYval;
                            pGCost  [a] = tempGcost;//change the G cost			
                            
                            //Because changing the G cost also changes the F cost, if
                            //the item is on the open list we need to change the item's
                            //recorded F cost and its position on the open list to make
                            //sure that we maintain a properly ordered open list.
                            for (int x = 1; x <= numberOfOpenListItems; x++) //look for the item in the heap
                            {
                                if (openX[openList[x]] == a && openY[openList[x]] == b) //item founded
                                {
                                    Fcost[openList[x]] = pGCost[a];// + Hcost[openList[x]];//change the F cost
                                    
                                    //See if changing the F score bubbles the item up from it's current location in the heap
                                    m = x;
                                    while (m != 1) //While item hasn't bubbled to the top (m=1)	
                                    {
                                        //Check if child is < parent. If so, swap them.	
                                        if (Fcost[openList[m]] < Fcost[openList[m/2]])
                                        {
                                            temp = openList[m/2];
                                            openList[m/2] = openList[m];
                                            openList[m] = temp;
                                            m = m/2;
                                        }
                                        else
                                            break;
                                    } 
                                    break; //exit for x = loop
                                } //If openX(openList(x)) = a
                            } //For x = 1 To numberOfOpenListItems
                        }//If tempGcost < Gcost(a,b)
                    }//else If whichList(b,a) = onOpenList	
                    
                    
                } //for (a = parentXval-1; a <= parentXval+1; a++){
            } //for (b = parentYval-1; b <= parentYval+1; b++){
            
        }//if (numberOfOpenListItems != 0)
        
        //9.If open list is empty then there is no path.	
        else
        {
            // Shorter than dist path ....
            // path = nonexistent; break;
            path = nonexistent;
            printf("Short path openX=%d, openY=%d\n", targetX, targetY);
            break;
        }  
        
        //If target is added to open list then path has been founded.
        if (numberOfOpenListItems != 0)
        {
            if( (Fcost[openList[1]] >= dist) &&
               (10*(abs(openX[openList[1]] - startX) + abs(openY[openList[1]] - startY)) >= dist) )
            {
                m_nCost = Fcost[openList[1]];
                path = founded;
                targetX = openX[openList[1]];
                targetY = openY[openList[1]];
                printf("Fcost=%d, openX=%d, openY=%d\n", Fcost[openList[1]], openX[openList[1]], openY[openList[1]]);
                break;
            }
        }
        
    }
    while (1);//Do until path is founded or deemed nonexistent
    
    //10.Save the path if it exists.
    if (path == founded)
    {
        pathStatus = founded;
        //a.Working backwards from the target to the starting location by checking
        //	each cell's parent, figure out the length of the path.
        pathX = targetX; pathY = targetY;
        do
        {
            //Look up the parent of the current cell.	
            tempx = parentX[pathY][pathX];		
            pathY = parentY[pathY][pathX];
            pathX = tempx;
            
            //Figure out the path length
            pathLength = pathLength + 1;
        }
        while (pathX != startX || pathY != startY);
        
        //b.Resize the data bank to the right size in bytes
        pathBank = (int*) realloc (pathBank, pathLength*8);
        
        //c. Now copy the path information over to the databank. Since we are
        //	working backwards from the target to the start location, we copy
        //	the information to the data bank in reverse order. The result is
        //	a properly ordered set of path data, from the first step to the
        //	last.
        pathX = targetX ; pathY = targetY;
        cellPosition = pathLength*2;//start at the end	
        do
        {
            cellPosition = cellPosition - 2;//work backwards 2 integers
            pathBank  [cellPosition] = pathX;
            pathBank[cellPosition+1] = pathY;
            
            //d.Look up the parent of the current cell.	
            tempx = parentX[pathY][pathX];		
            pathY = parentY[pathY][pathX];
            pathX = tempx;
            
            //e.If we have reached the starting square, exit the loop.	
        }
        while (pathX != startX || pathY != startY);	
        
        //11.Read the first path step into xPath/yPath arrays
        ReadPath(startingX,startingY,1);
        
    }
    return path;
    
    
    //13.If there is no path to the selected target, set the pathfinder's
    //	xPath and yPath equal to its current location and return that the
    //	path is nonexistent.
    //noPath:
    xPath = startingX;
    yPath = startingY;
    return nonexistent;
}

//==========================================================
//READ PATH DATA: These functions read the path data and convert
//it to screen pixel coordinates.
void CAStar::ReadPath(int currentX,int currentY,
                      int pixelsPerFrame)
{
    /*
     ;	Note on PixelsPerFrame: The need for this parameter probably isn't
     ;	that obvious, so a little explanation is in order. This
     ;	parameter is used to determine if the pathfinder has gotten close
     ;	enough to the center of a given path square to warrant looking up
     ;	the next step on the path.
     ;	 
     ;	This is needed because the speed of certain sprites can
     ;	make reaching the exact center of a path square impossible.
     ;	In Demo #2, the chaser has a velocity of 3 pixels per frame. Our
     ;	tile size is 50 pixels, so the center of a tile will be at location
     ;	25, 75, 125, etc. Some of these are not evenly divisible by 3, so
     ;	our pathfinder has to know how close is close enough to the center.
     ;	It calculates this by seeing if the pathfinder is less than 
     ;	pixelsPerFrame # of pixels from the center of the square. 
     
     ;	This could conceivably cause problems if you have a *really* fast
     ;	sprite and/or really small tiles, in which case you may need to
     ;	adjust the formula a bit. But this should almost never be a problem
     ;	for games with standard sized tiles and normal speeds. Our smiley
     ;	in Demo #4 moves at a pretty fast clip and it isn't even close
     ;	to being a problem.
     */
    
    //If a path has been founded for the pathfinder	...
    if (pathStatus == founded)
    {
        
        //If path finder is just starting a new path or has reached the 
        //center of the current path square (and the end of the path
        //hasn't been reached), look up the next path square.
        if (pathLocation < pathLength)
        {
            //if just starting or if close enough to center of square
            if (pathLocation == 0 || 
                (abs(currentX - xPath) < pixelsPerFrame && abs(currentY - yPath) < pixelsPerFrame))
                pathLocation = pathLocation + 1;
        }
        
        //Read the path data.		
        xPath = ReadPathX(pathLocation);
        yPath = ReadPathY(pathLocation);
        
        //If the center of the last path square on the path has been 
        //reached then reset.
        if (pathLocation == pathLength) 
        {
            if (abs(currentX - xPath) < pixelsPerFrame 
                && abs(currentY - yPath) < pixelsPerFrame) //if close enough to center of square
                pathStatus = notStarted; 
        }
    }
    
    //If there is no path for this pathfinder, simply stay in the current
    //location.
    else
    {	
        xPath = currentX;
        yPath = currentY;
    }
}


//The following two functions read the raw path data from the pathBank.
//You can call these functions directly and skip the readPath function
//above if you want. Make sure you know what your current pathLocation
//is.

//-----------------------------------------------------------------------------
// Name: ReadPathX
// Desc: Reads the x coordinate of the next path step
//-----------------------------------------------------------------------------
int CAStar::ReadPathX(int pathLocation)
{
    int x=-1;
    if (pathLocation <= pathLength)
    {
        
        //Read coordinate from bank
        x = pathBank[pathLocation*2-2];
        
        //Adjust the coordinates so they align with the center
        //of the path square (optional). This assumes that you are using
        //sprites that are centered -- i.e., with the midHandle command.
        //Otherwise you will want to adjust this.
        x = (int)(tileSize*x + .5*tileSize);
        
    }
    return x;
}	


//-----------------------------------------------------------------------------
// Name: ReadPathY
// Desc: Reads the y coordinate of the next path step
//-----------------------------------------------------------------------------
int CAStar::ReadPathY(int pathLocation)
{
    int y=-1;
    if (pathLocation <= pathLength)
    {
        
        //Read coordinate from bank
        y = pathBank[pathLocation*2-1];
        
        //Adjust the coordinates so they align with the center
        //of the path square (optional). This assumes that you are using
        //sprites that are centered -- i.e., with the midHandle command.
        //Otherwise you will want to adjust this.
        y = (int)(tileSize*y + .5*tileSize);
        
    }
    return y;
}
