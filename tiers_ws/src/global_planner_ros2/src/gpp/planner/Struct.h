//
//  Struct.h
//  PathPlanning
//
//  Created by 윤필립 on 2016. 11. 22..
//  Copyright (c) 2016년 윤필립. All rights reserved.
//

#ifndef PathPlanning_Struct_h
#define PathPlanning_Struct_h

#endif

#include <vector>

typedef unsigned char BYTE;

template <class T>
struct Tmpl2DPoint
{
    T x;
    T y;
    
    Tmpl2DPoint() {};
    Tmpl2DPoint(T xx, T yy) { x = xx; y = yy; }
};

typedef Tmpl2DPoint <int> i2DPoint;
typedef std::vector <i2DPoint> i2DPointArray;

typedef i2DPoint iPoint;
typedef i2DPointArray iPointArray;


#define _TmPt2D Tmpl2DPoint <T>
#define _DefTM template <class T>

_DefTM _TmPt2D   operator +  (const _TmPt2D &A, const double    B) { return _TmPt2D(A.x+(T)B, A.y+(T)B); }
_DefTM _TmPt2D   operator -  (const _TmPt2D &A, const double    B) { return _TmPt2D(A.x-(T)B, A.y-(T)B); }
_DefTM _TmPt2D   operator *  (const _TmPt2D &A, const double    B) { return _TmPt2D(A.x*(T)B, A.y*(T)B); }
_DefTM _TmPt2D   operator /  (const _TmPt2D &A, const double    B) { return _TmPt2D(A.x/(T)B, A.y/(T)B); }

_DefTM _TmPt2D & operator += (      _TmPt2D &A, const double    B) { A.x += (T)B; A.y += (T)B; return A;  }
_DefTM _TmPt2D & operator -= (      _TmPt2D &A, const double    B) { A.x -= (T)B; A.y -= (T)B; return A;  }
_DefTM _TmPt2D & operator *= (      _TmPt2D &A, const double    B) { A.x *= (T)B; A.y *= (T)B; return A;  }
_DefTM _TmPt2D & operator /= (      _TmPt2D &A, const double    B) { A.x /= (T)B; A.y /= (T)B; return A;  }

_DefTM _TmPt2D & operator -= (      _TmPt2D &A, const _TmPt2D &B) { A.x -=  B.x; A.y -=  B.y; return A;  }
_DefTM _TmPt2D & operator += (      _TmPt2D &A, const _TmPt2D &B) { A.x +=  B.x; A.y +=  B.y; return A;  }
_DefTM _TmPt2D & operator *= (      _TmPt2D &A, const _TmPt2D &B) { A.x *=  B.x; A.y *=  B.y; return A;  }
_DefTM _TmPt2D & operator /= (      _TmPt2D &A, const _TmPt2D &B) { A.x /=  B.x; A.y /=  B.y; return A;  }

_DefTM _TmPt2D   operator +  (const double    B, const _TmPt2D &A) { return _TmPt2D((T)B + A.x, (T)B + A.y); }
_DefTM _TmPt2D   operator -  (const double    B, const _TmPt2D &A) { return _TmPt2D((T)B - A.x, (T)B - A.y); }
_DefTM _TmPt2D   operator *  (const double    B, const _TmPt2D &A) { return _TmPt2D((T)B * A.x, (T)B * A.y); }
_DefTM _TmPt2D   operator /  (const double    B, const _TmPt2D &A) { return _TmPt2D((T)B / A.x, (T)B / A.y); }

_DefTM _TmPt2D   operator +  (const _TmPt2D &A, const _TmPt2D &B) { return _TmPt2D(A.x  + B.x, A.y  + B.y); }
_DefTM _TmPt2D   operator -  (const _TmPt2D &A, const _TmPt2D &B) { return _TmPt2D(A.x  - B.x, A.y  - B.y); }
_DefTM _TmPt2D   operator *  (const _TmPt2D &A, const _TmPt2D &B) { return _TmPt2D(A.x  * B.x, A.y  * B.y); }
_DefTM _TmPt2D   operator /  (const _TmPt2D &A, const _TmPt2D &B) { return _TmPt2D(A.x  / B.x, A.y  / B.y); }

_DefTM _TmPt2D   operator -  (const _TmPt2D &A                   ) { return _TmPt2D(-A.x, -A.y); }
_DefTM bool      operator == (const _TmPt2D &A, const _TmPt2D &B) { return (A.x == B.x && A.y == B.y) ? true  : false; }
_DefTM bool      operator != (const _TmPt2D &A, const _TmPt2D &B) { return (A.x == B.x && A.y == B.y) ? false : true ; }

#undef _DefTM
#undef _TmPt2D

