//
//  aStarLibrary.h
//  PathPlanning
//
//  Created by 윤필립 on 2016. 11. 22..
//  Copyright (c) 2016년 윤필립. All rights reserved.
//

#ifndef __PathPlanning__aStarLibrary__
#define __PathPlanning__aStarLibrary__

#include <stdio.h>

#endif /* defined(__PathPlanning__aStarLibrary__) */

/**
 ;===================================================================
 ;A* Pathfinder (Version 1.71a) by Patrick Lester. Used by permission.
 ;===================================================================
 ;Last updated 06/16/03 -- Visual C++ version
 */
#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <opencv2/opencv.hpp>
#include "Struct.h"

//Declare constants
#define tileSize		1
#define notfinished		0
#define notStarted		0	/// path-related constants
#define founded			1
#define nonexistent		2

#define walkable		255
#define unwalkable		  0	/// gridMap array constants

int  GetShortestPath (const BYTE *csMap, int csStep, int width, int height, iPointArray &viaPt, int viaSize);
bool CheckObstacle (const BYTE *csMap, int csStep, int width, int height, iPoint p0, iPoint p1);

/// Shortest Path Planning¿ª ¿ß«— A-Star library
/**
 * A* Pathfinder (Version 1.71a) by Patrick Lester.
 * Modified by SAIT.
 */
class CAStar
{
public:
    CAStar();
    
protected:
    int mapWidth;
    int mapHeight;
    int onClosedList;
    int	m_nCost;
    
    //Create needed arrays
    
    const BYTE      *gridMap;		/// Gridmap
    int              gridStep;
    
    
    char *pTotalBuffer;
    int   nTotalBufSize;
    unsigned int    *openList;		/// 1 dimensional array holding ID# of open list items
    unsigned short **whichList;     /// 2 dimensional array used to record
    unsigned short **parentX;		/// 2d array to store parent of each cell (x)
    unsigned short **parentY;		/// 2d array to store parent of each cell (y)
    unsigned short **Gcost; 		/// 2d array to store G cost for each cell.
    //	CMatrix <USHORT> parentXBuf;		/// 2d array to store parent of each cell (x)
    //	CMatrix <USHORT> parentYBuf;		/// 2d array to store parent of each cell (y)
    //	CMatrix <USHORT> whichListBuf;
    //	CMatrix <USHORT> GcostBuf;		/// Fcost = Gcost + Hcost
    /// whether a cell is on the open list or on the closed list.
    unsigned short  *openX;			/// 1d array stores the x location of an item on the open list
    unsigned short  *openY;			/// 1d array stores the y location of an item on the open list
    unsigned short  *Fcost;			/// 1d array to store F cost of a cell on the open list
    unsigned short  *Hcost;			/// 1d array to store H cost of a cell on the open list
    
    int pathLength;										/// stores length of the found path for critter
    int pathLocation;									/// stores current position along the chosen path for critter
    int *pathBank;
    
    //Path reading variables
    int pathStatus;
    int xPath;
    int yPath;
    
    int mapWidthMax;
    int mapHeightMax;
    
    iPoint *linePoints;
protected:
    void _ResetWhichListBuf();
    
public:
    BYTE *GetUnusedBuffer1() { return (BYTE*)openX; }
    BYTE *GetUnusedBuffer2() { return (BYTE*)openY; }
    BYTE *GetUnusedBuffer3() { return (BYTE*)Fcost; }
    int   GetUnusedBufferSize() { return (mapWidthMax*mapHeightMax+2)*sizeof(short); }
    
protected:
    //-----------------------------------------------------------------------------
    // Function Prototypes: where needed
    //-----------------------------------------------------------------------------
    void ReadPath (int currentX, int currentY, int pixelsPerFrame);
    int  ReadPathX(int pathLocation);
    int  ReadPathY(int pathLocation);
    
    /// FindAStarø°º≠ »£√‚«œ¥¬ A* search ø£¡¯
    int  FindPath (int startingX, int startingY, int targetX, int targetY, bool bPreventCross);
    
    /// FindAStarFrontierø°º≠ »£√‚«œ¥¬ A* search ø£¡¯
    int  FindFrontier	(int startingX, int startingY, int dist, bool bPreventCross);
    //int  GetShortestPath (iPoint *viaPt) { GetShortestPath (gridMap, gridStep, mapWidth, mapHeight, viaPt, viaSize);}
    
public:
    /// ∏µÁ ∫ØºˆøÕ πˆ∆€∏¶ √ ±‚»≠«‘. malloc Ω««‡
    bool Initialize(int width, int height);
    
    int   GetMemorySize();
    int   CalcMemorySize(int width, int height);
    char *GetInternalBuffer();
    
    /// malloc µ«æÓ ¿÷¥¯ ∫Øºˆ¿« ∏ﬁ∏∏Æ «ÿ¡¶ Ω««‡
    void Release();
    
    bool SetCSMap (const BYTE *csMap, int csStep, int width, int height);
    
    bool FindAStarPath (iPoint robot, iPoint goal, iPointArray& viaPt, int &size, bool bPreventCross, bool bShortPath=true);
    
    bool FindAStarFrontier (iPoint robot, int dist,    iPoint *viaPt, int &size, bool bPreventCross, bool bShortPath=true);
    int  GetMapWdithMax()	{	return mapWidthMax;	}
    int  GetMapHeightMax()	{	return mapHeightMax;}
    
    /// µŒ ∆˜¿Œ∆Æ (p0, p1) ªÁ¿Ãø° ¿Âæ÷π∞¿Ã ¿÷¥¬¡ˆ √º≈©
    bool aStarCheckObstacle (iPoint p0, iPoint p1) { return CheckObstacle (gridMap, gridStep, mapWidth, mapHeight, p0, p1); }
    
    /// µŒ via Point ªÁ¿Ãø° ¿Âæ÷π∞¿Ã æ¯¿ª ∂ß «œ≥™¿« via Point∑Œ merge«‘
    int  GetShortPath(iPointArray& viaPt);
    
    /// Shortest Path¿« cost∏¶ ∏Æ≈œ«‘
    int	 GetCost();
};

#endif
