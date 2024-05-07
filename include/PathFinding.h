#ifndef PATHFINDING_H_
#define PATHFINDING_H_
#if defined (EXPORTBUILD)  
# define _DLLExport __declspec (dllexport)  
# else  
# define _DLLExport __declspec (dllimport)  
#endif  

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


#include "NavMesh.h"
#include "DetourTileCache.h"
#include "DetourCommon.h"
#include "DetourInit.h"




typedef void(* DebugCallback) (const char *str);

extern "C" void _DLLExport RegisterDebugCallback(DebugCallback callback);

extern "C" bool _DLLExport LoadNavData(unsigned char* data);

extern "C" void _DLLExport UnLoadNavData();

extern "C" bool _DLLExport FindNavPath(float* start, float* end, int jflag, float*  &outarr, int &len);

extern "C" bool _DLLExport Raycast(float* start, float* end, int jflag, float* &outarr, float* &normalDir);

extern "C" bool _DLLExport IsPosInBlock(float* jpos);

extern "C" void _DLLExport ClearIntPtr(void* pBuffer);

extern "C" void _DLLExport FindNearestPoly(float* start, float* end, int flag, int &nearRef, float* &pt);

extern "C" bool _DLLExport IsWalkable(float* start, int flag);

extern "C" float _DLLExport GetPolyHeight(float* point, int flag);

extern "C" void _DLLExport SetPolyPickExtern(float x, float y, float z);

extern "C" unsigned int _DLLExport AddCylinderObstacle(const float* pos, const float radius, const float height);

extern "C" unsigned int _DLLExport AddBoxObstacle(const float* bmin, const float* bmax);

extern "C" bool _DLLExport RemoveObstacle(int obstacleRef);

extern "C" void _DLLExport UpdateObstacle();
#endif