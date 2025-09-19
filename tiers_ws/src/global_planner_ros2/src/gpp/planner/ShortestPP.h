//
//  ShortestPP.h
//  PathPlanning
//
//  Created by 윤필립 on 2016. 11. 22..
//  Copyright (c) 2016년 윤필립. All rights reserved.
//

#ifndef __PathPlanning__ShortestPP__
#define __PathPlanning__ShortestPP__


#endif /* defined(__PathPlanning__ShortestPP__) */

//#include "vipl.h"
//#include "aStarLibrary.h"
//#include "VipIppWrap.h"

//
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "aStarLibrary.h"
#include <algorithm>
#define _minAstar_size_ 2048
//

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



class CShortestPP
{
public:
    CShortestPP();
    virtual ~CShortestPP();
    
public:
    bool FindPathPyr(const cv::Mat *src, int step, int width, int height, iPoint srcPos, iPoint dstPos, iPointArray& retArray, iPointArray& pathArray, bool bUseIR=false);
    
    bool Initialize(int XSize, int YSize, bool bUseScaleMap=false);
    
    /// A*∑Œ √£¿∫ √÷¿˚ path¿« cost∏¶ return. (1-grid = 10, ¥Î∞¢º± = 14)
    int GetCost(){	return m_Astar.GetCost(); }
    
    static void MakePathSmooth(iPointArray &srcArray, int thresDist = 2);
    
    void Thining(cv::Mat *src, int maxThinCount);
    
protected:
    int m_csStep;
    int m_tmpStep;
    
    int   m_nWidth;
    int   m_nHeight;
    
protected:
    CAStar m_Astar;
    int m_nPyr;
    int m_scaleMap;
    
protected:
    bool _FitPathToSpace(iPointArray& srcArray, const BYTE *csMap, int csStep, int csWidth, int csHeight, int scale);
};

void FitPositionToSpace(iPoint &srcPos, const BYTE *src, int step, int width, int height, const int D);

