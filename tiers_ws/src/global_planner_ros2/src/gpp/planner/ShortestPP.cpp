//
//  ShortestPP.cpp
//  PathPlanning
//
//  Created by 윤필립 on 2016. 11. 22..
//  Copyright (c) 2016년 윤필립. All rights reserved.
//

#include "ShortestPP.h"
#include <stdio.h>


#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//#define _use_time_check_

#ifdef _use_time_check_
#ifndef _LINUX
#include "Oil_ETC/MMTimer.h"
static CMMTimer mm_timer;
#endif

static double GetTick()
{
#ifdef _LINUX
    return mmGetTime();
#else
    return mm_timer.GetTime() * 1000;
#endif
}
#endif

#ifdef _use_time_check_
#define _________________________________________________________time_check_func(name) name = GetTick();
#else
#define _________________________________________________________time_check_func(name) void(0)
#endif

/********************************************************************/
//  Code developed by Lip                                           //
/********************************************************************/
CShortestPP::CShortestPP()
{
    m_nWidth = m_nHeight = 0;
}

CShortestPP::~CShortestPP()
{
    m_Astar.Release();
}

/** Pyramid imageø°º≠ A*∏¶ ¿ÃøÎ«œø© srcPos -> dstPos ±Ó¡ˆ¿« Shortest Path∏¶ √£¥¬¥Ÿ.
 *
 *- const BYTE *src, int step, int width, int height: Gridmap image ¡§∫∏
 *- iPoint srcPos: √‚πﬂ ¿ßƒ° (x, y)
 *- iPoint dstPos: ∏Ò¿˚¡ˆ ¿ßƒ° (x, y)
 *- iPointArray &retArray: √£æ∆¡¯ viaPoint array
 *- return: º∫∞¯=true; Ω«∆–=false;
 */

//원본
//bool CShortestPP::FindPathPyr(const cv::Mat *src, int step, int width, int height, iPoint srcPos, iPoint dstPos, iPointArray& retArray, bool bUseIR)
bool CShortestPP::FindPathPyr(const cv::Mat *src, int step, int width, int height, iPoint srcPos, iPoint dstPos, iPointArray& retArray, iPointArray& pathArray, bool bUseIR)
{
    bool bUseScaleMap = true;
    if (!Initialize(width, height, bUseScaleMap))
    {
        printf("[FindPathPyr] Failed to allocate memory[%d %d]\n", width, height);
        return false;
    }
    
    retArray.resize(0);
    printf("[FindPathPyr] oriSize[%d %d], nPyr[%d] scale[%d] srcPos[%d %d] dstPos[%d %d]\n", width, height, m_nPyr, m_scaleMap, srcPos.x, srcPos.y, dstPos.x, dstPos.y);
    
    iPoint srcTmp = iPoint(std::min(std::max(srcPos.x, 0), width),
                           std::min(std::max(srcPos.y, 0), height) );
    iPoint dstTmp = iPoint(std::min(std::max(dstPos.x, 0), width),
                           std::min(std::max(dstPos.y, 0), height) );
    
    BYTE *tmp_map;
    int   csStep = m_csStep;
    
    //    MakeCSImage(
    //                src, step, csImg, csStep,
    //                tmp, width, height, srcTmp, dstTmp, true, bUseIR);
    
    if (srcTmp.x == dstTmp.x && srcTmp.y == dstTmp.y)
        return false;
    
    int tmpWidth  = width / (1 << m_nPyr);
    int tmpHeight = height/ (1 << m_nPyr);
    int tmpStep   = tmpWidth;
    
    cv::Mat tmp;
    cv::resize(*src, tmp, cv::Size(tmpWidth, tmpHeight));
    
    // For safety path
    Thining(&tmp, 0);
    
    /*cv::erode(tmp, tmp, cv::Mat());
     cv::erode(tmp, tmp, cv::Mat());
     cv::erode(tmp, tmp, cv::Mat());
     cv::erode(tmp, tmp, cv::Mat());
     cv::erode(tmp, tmp, cv::Mat());*/
    
    
    iPoint srcSmall = srcPos / (1 << m_nPyr);
    iPoint dstSmall = dstPos / (1 << m_nPyr);
    
    if (srcSmall.x == dstSmall.x && srcSmall.y == dstSmall.y)
    {
        tmp = *src;
        tmpStep  = width;
        tmpWidth = width;
        tmpHeight= height;
        
        bUseScaleMap = false;
        
        Initialize(width, height, bUseScaleMap);
    }
    
    
    // cv::Mat -> BYTE*
    tmp_map = new unsigned char[tmp.rows * tmp.cols];
    for ( int j = 0 ; j < tmp.rows; j++ )
    {
        for ( int i = 0; i < tmp.cols; i++ )
        {
            unsigned char& uxy = tmp.at<unsigned char>(j, i);
            unsigned char color = (unsigned char) uxy;
            tmp_map[j * tmp.cols + i] = color;
        }
    }
    
    if (!m_Astar.SetCSMap(tmp_map, tmp.cols, tmpWidth, tmpHeight))
        return false;
    
    int nViaPts = 2000;
    if (!bUseScaleMap)
    {
        if (m_Astar.FindAStarPath(srcTmp, dstTmp, retArray, nViaPts, false, false))
        {
            retArray.resize(nViaPts);
            if (retArray[0].x != srcTmp.x || retArray[0].y != srcTmp.y)
                retArray.insert(retArray.begin(), srcTmp);
            
            printf("[Pyr]Close Pts, retArray(%d)\n", (int)retArray.size());
        }
        else
        {
            printf("[CShortestPP]FindPathPyr-Close Pts, Failed!! \n");
            retArray.clear();
            
            delete tmp_map;
            return false;
        }
    }
    else
    {
        retArray.resize(nViaPts);
        if (m_Astar.FindAStarPath(srcSmall, dstSmall, retArray, nViaPts, false, false))
        {
            retArray.resize(nViaPts);
            if (retArray[0].x != srcSmall.x || retArray[0].y != srcSmall.y)
            {
                retArray.insert(retArray.begin(), srcSmall);
                nViaPts++;
            }
            
            ///////////////////////////////////////////////////////
            // Local A*
            {
                for (int i = 0 ; i < retArray.size() ; i++)
                {
                    retArray[i] *= m_scaleMap;
                    //printf("FOR DEBUG (x,y) = (%d, %d)\n", retArray[i].x, retArray[i].y);
                    FitPositionToSpace(retArray[i], tmp_map, width, width, height, m_scaleMap * 2);
                }
                if (srcTmp.x != retArray[0].x || srcTmp.y != retArray[0].y)
                    retArray.insert(retArray.begin(), srcTmp);
                
                printf("[Pyr]retArray(%d)\n", (int)retArray.size());

                //수정
                pathArray = retArray;
                //수정 끝
                
                if (m_scaleMap == 1 || _FitPathToSpace(retArray, tmp_map, tmpStep, tmpWidth, tmpHeight, m_scaleMap))
                {
                    if (retArray[0].x != srcTmp.x || retArray[0].y != srcTmp.y)
                        retArray.insert(retArray.begin(), srcTmp);
                }
                else
                {
                    printf("[CShortestPP]FindPathPyr-Failed in FitPathToSpace!! \n");
                    retArray.clear();
                    
                    delete tmp_map;
                    return false;
                }
            }
        }
        else
        {
            retArray.clear();
            
            delete tmp_map;
            return false;
        }
    }
    
    if (retArray.size() > 0)
    {
        printf("CShortestPP::GetShortestPath Start\n");
        int retSize = GetShortestPath(tmp_map, csStep, width, height, retArray, (int)retArray.size());
        printf("CShortestPP::GetShortestPath End %d\n",retSize);

        retArray.resize(retSize);
        
        MakePathSmooth(retArray, 2);
        
        delete tmp_map;
        return true;
    }
    
    delete tmp_map;
    return false;
}

bool CShortestPP::Initialize(int XSize, int YSize, bool bUseScaleMap)
{
    int astarX = XSize;
    int astarY = YSize;
    
    if (bUseScaleMap)
    {
        if ( XSize < _minAstar_size_ && YSize < _minAstar_size_ )
        {
        m_nPyr  = 0;
        m_scaleMap = 1;
        }
        else if		(XSize< 2*_minAstar_size_ && YSize< 2*_minAstar_size_)
        {
            m_nPyr  = 1;
            m_scaleMap = 2;
        }
        else if (XSize< 4*_minAstar_size_ && YSize< 4*_minAstar_size_)
        {
            m_nPyr  = 2;
            m_scaleMap = 4;
        }
        else
        {
            m_nPyr  = 3;
            m_scaleMap = 8;
        }
        
        astarX = std::max(XSize/m_scaleMap+1, _minAstar_size_);
        astarY = std::max(YSize/m_scaleMap+1, _minAstar_size_);
    }
    
    if (!m_Astar.Initialize(astarX, astarY))
    {
        printf("Failed to Initialize Astar Buffer\n");
        return false;
    }
    
    return true;
}

bool CShortestPP::_FitPathToSpace(iPointArray& srcArray, const BYTE *csMap, int csStep, int csWidth, int csHeight, int scale)
{
    int offset = scale*2;
    int mapWidth = m_Astar.GetMapWdithMax();
    int mapHeight= m_Astar.GetMapHeightMax();
    
    int retSize = 0;
    
    for (int idx0 = 0 ; idx0 < srcArray.size()-1 ; idx0 += retSize-1)
    {
        iPoint minPos(csWidth, csHeight), minTemp(csWidth, csHeight);
        iPoint maxPos(0,0), maxTemp(0,0);
        int width = 0, height= 0;
        int nSize = (int)srcArray.size();
        
        if (retSize>1)
            offset = scale*2;
        
        int idxN;
        for (idxN=idx0 ; idxN<nSize ; idxN++)
        {
            iPoint pt(srcArray[idxN]);
            if (minTemp.x>pt.x) minTemp.x = pt.x;
            if (minTemp.y>pt.y) minTemp.y = pt.y;
            if (maxTemp.x<pt.x) maxTemp.x = pt.x;
            if (maxTemp.y<pt.y) maxTemp.y = pt.y;
            
            width = std::min(csWidth , maxTemp.x+offset) - std::max(0, minTemp.x-offset);
            height= std::min(csHeight, maxTemp.y+offset) - std::max(0, minTemp.y-offset);
            
            if (width>mapWidth || height>mapHeight)
            {
                break;
            }
            else
            {
                minPos = minTemp;
                maxPos = maxTemp;
            }
        }
        
        nSize = idxN - idx0;
        if (nSize==0)
            return false;
        --idxN;
        
        minPos.x = std::max(0, minPos.x-offset); maxPos.x = std::min(csWidth , maxPos.x+offset);
        minPos.y = std::max(0, minPos.y-offset); maxPos.y = std::min(csHeight, maxPos.y+offset);
        width = maxPos.x - minPos.x;
        height= maxPos.y - minPos.y;
        
        iPoint srcPos = srcArray[idx0] - minPos;
        iPoint dstPos = srcArray[idxN] - minPos;
        
        iPointArray retArray;
        retSize = 1000;
        
        m_Astar.Initialize(width, height);
        m_Astar.SetCSMap(csMap+csStep*minPos.y + minPos.x, csStep, width, height);
        if (m_Astar.FindAStarPath(srcPos, dstPos, retArray, retSize, false, false))
        {
            retArray.resize(retSize);
            
            srcArray.erase(srcArray.begin()+idx0, srcArray.begin()+idx0+nSize);
            
            for ( int i = 0; i < retArray.size(); i++ )
            {
                iPoint temp;
                temp.x = retArray[i].x + minPos.x;
                temp.y = retArray[i].y + minPos.y;
                srcArray.insert(srcArray.begin() + idx0 + i, temp);
            }
        }
        else
        {
            if (width >= csWidth || height >= csHeight)
            {
                printf("[ShortestPP] FitPathToSpace , Can't Find Path In Fine GridMap -> Return true\n");
                return true;
            }
            
            retSize = 1;
            offset += scale;
            printf("[ShortestPP] LocalMap Size Increase -> %d\n", offset);
        }
    }
    return true;
}

void CShortestPP::MakePathSmooth(iPointArray &srcArray, int thresDist)
{
    if (!srcArray.size())
        return;
    
    int srcStart=0;
    int thresDist2 = thresDist * thresDist;
    //	while (1)
    {
        // ¿Œ¡¢«— via-pt --> arrayø° ¿˙¿Â
        iPointArray& array = srcArray;	// array.Add(srcArray[srcStart]);
        
        // ¿Œ¡¢«— via-pts 3∞≥ ¿ÃªÛ
        if (array.size()>2)
        {
            int s = 0;
            while (1)
            {
                float sumx = (float)(array[s].x + array[s+1].x);
                float sumy = (float)(array[s].y + array[s+1].y);
                float sumxx= (float)(array[s].x*array[s].x + array[s+1].x*array[s+1].x);
                float sumxy= (float)(array[s].x*array[s].y + array[s+1].x*array[s+1].y);
                
                // s∫Œ≈Õ º¯¬˜¿˚¿∏∑Œ ¡˜º±¿« πÊ¡§Ωƒ ±∏«œ±‚ (<-linear regression)
                float n = 2;
                for (int i=s+2 ; i<array.size() ; i++)
                {
                    n    += 1;
                    sumx += array[i].x;
                    sumy += array[i].y;
                    sumxx+= array[i].x * array[i].x;
                    sumxy+= array[i].x * array[i].y;
                    
                    float Sxx = sumxx - sumx * sumx /n;
                    float Sxy = sumxy - sumx * sumy /n;
                    float aa  =  Sxy / n; //Sxy / Sxx;
                    float bb  = -Sxx / n;
                    float cc  = (sumy*Sxx - Sxy*sumx) /(n*n) ;
                    
                    bool bbreak = false;
                    for (int j=s ; j<=i ; j++)
                    {
                        float den2 = aa * aa + bb * bb;
                        float num  = aa * array[j].x + bb * array[j].y + cc;
                        if (num*num / den2 > thresDist2)
                        {
                            bbreak = true;
                            break;
                        }
                    }
                    
                    if (bbreak || i>=array.size()-1)
                    {
                        int e = bbreak? i-1 : i;
                        if (e-s>=2)	// ∞∞¿∫ ¡˜º±ªÛ¿« pt 3∞≥ ¿ÃªÛ -> ∞°øÓµ• ¡°µÈ ªË¡¶
                        {
                            array.erase(array.begin() + s + 1, array.begin() + e - 1);
                        }
                        s = s + 1;
                        break;
                    }
                }
                if (s >= array.size()-2)	break;
            }
        }
    }
}

void FitPositionToSpace(iPoint &srcPos, const BYTE *src, int step, int width, int height, const int D)
{
    int L = std::max(srcPos.x - D, 0); int R = std::min(srcPos.x + D, width );
    int T = std::max(srcPos.y - D, 0); int B = std::min(srcPos.y + D, height);
    
    int MinDist = INT_MAX;
    iPoint MinPos = srcPos;
    for (int y = T ; y < B ; y++)
    {
        const BYTE *s = src + step * y;
        int yy = y - srcPos.y;
        for (int x = L ; x < R ; x++)
        {
            if (s[x])
            {
                int xx = x - srcPos.x;
                int d = (xx*xx + yy*yy);
                if ( d < MinDist)
                {
                    MinDist = d;
                    MinPos.x = x;
                    MinPos.y = y;
                }
            }
        }
    }
    
    srcPos = MinPos;
}


void CShortestPP::Thining(cv::Mat *src, int maxThinCount)
{
    cv::Mat tmp = src->clone();
    
    int roiWidth = tmp.cols - 2;
    int roiHeight = tmp.rows - 2;
    
    bool bContinue = true;
    
    for ( int thinCount = 0; (thinCount <= maxThinCount) && bContinue; thinCount++ )
    {
        bContinue = false;
        for ( int k = 0; k < 2; k++ )
        {
            {
                for ( int h = 1; h <= roiHeight; h++ )
                {
                    for ( int j = 1; j <= roiWidth; j++ )
                    {
                        //printf("%u  ", tmp.at<uchar>(h,j));
                        if ( tmp.at<uchar>(h,j) > 0 ) // if white cell?
                        {
                            int jm = j - 1;
                            int jp = j + 1;
                            
                            // First case
                            int cnt = 0;
                            if ( tmp.at<uchar>(h-1, jm) != tmp.at<uchar>(h-1, j) ) cnt++;
                            if ( tmp.at<uchar>(h-1, j) != tmp.at<uchar>(h-1, jp) ) cnt++;
                            
                            if ( tmp.at<uchar>(h-1, jp) != tmp.at<uchar>(h, jp) ) cnt++;
                            if ( tmp.at<uchar>(h, jp) != tmp.at<uchar>(h+1, jp) ) cnt++;
                            
                            if ( tmp.at<uchar>(h+1, jp) != tmp.at<uchar>(h+1, j) ) cnt++;
                            if ( tmp.at<uchar>(h+1, j) != tmp.at<uchar>(h+1, jm) ) cnt++;
                            
                            if ( tmp.at<uchar>(h+1, jm) != tmp.at<uchar>(h, jm) ) cnt++;
                            if ( tmp.at<uchar>(h, jm) != tmp.at<uchar>(h-1, jm) ) cnt++;
                            
                            if ( !cnt || cnt >= 4 ) continue;
                            
                            // Second case
                            int Ne = 0;
                            if ( !tmp.at<uchar>(h-1,jm) ) Ne++;
                            if ( !tmp.at<uchar>(h-1,j) ) Ne++;
                            if ( !tmp.at<uchar>(h-1,jp) ) Ne++;
                            
                            if ( !tmp.at<uchar>(h,jp) ) Ne++;
                            if ( !tmp.at<uchar>(h,jm) ) Ne++;
                            
                            if ( !tmp.at<uchar>(h+1,jp) ) Ne++;
                            if ( !tmp.at<uchar>(h+1,j) ) Ne++;
                            if ( !tmp.at<uchar>(h+1,jm) ) Ne++;
                            
                            if ( Ne < 2 || Ne > 6 ) continue;
                            
                            // Third case
                            //if ( tmp.at<uchar>(h,j) && tmp.at<uchar>(h -1,j) && tmp.at<uchar>(h,jp) ) continue;
                            
                            // Forth case
                            //if ( tmp.at<uchar>(h-1,j) && tmp.at<uchar>(h+1,j) && tmp.at<uchar>(h,jm) ) continue;
                            
                            src->at<uchar>(h, j) = 0;
                        }
                    }
                }
            }
            
            ///////////////////////////
            for ( int h = 1; h <= roiHeight; h++ )
            {
                for ( int j = 1; j <= roiWidth; j++ )
                {
                    if ( src->at<uchar>(h, j) != tmp.at<uchar>(h, j) )
                    {
                        tmp.at<uchar>(h, j) = src->at<uchar>(h, j);
                        bContinue = true;
                    }
                }
            }
        }
    }
}
