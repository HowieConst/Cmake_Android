/*
 * NavMesh.h
 *
 *  Created on: 2013-10-14
 *      Author: Lufeng
 */

#ifndef NAVMESH_H_
#define NAVMESH_H_

#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS

#include <iterator>
#include <utility>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

using namespace std;

// 寻路用到的最大值数据
static const int MAX_POLYS = 512;
static const int MAX_SMOOTH = 4096;


// 坐标中心扩展坐标轴单位距离
float m_polyPickExt[3] = { 0.1, 500, 0.1 };
float m_polyPickExt1[3] = { 2, 4000, 2 };
#endif /* NAVMESH_H_ */
