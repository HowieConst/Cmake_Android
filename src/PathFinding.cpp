#define EXPORTBUILD  
#include "PathFinding.h"
#include <sstream>
#include <string>
#include <cfloat>

dtTileCache* tileCache = NULL;
dtNavMesh* navMesh = NULL;
dtNavMeshQuery* navMeshQuery = NULL;
DebugCallback gDebugCallback;

void RegisterDebugCallback(DebugCallback callback)
{
	if (callback)
	{
		gDebugCallback = callback;
	}
}

void LogToUnity(const char *msg)
{
	if (gDebugCallback)
	{
		gDebugCallback(msg);
	}
}


bool  LoadNavData(unsigned char* data)
{

	UnLoadNavData();

	tileCache = dtAllocTileCache();

	navMesh = dtLoadNavMesh(data, tileCache);

	if (NULL == navMesh)
	{
		LogToUnity("Function: LoadNavData : 加载 NavMesh Error!");
		tileCache = NULL;
		return false;
	}
	// 初始化查询对象
	navMeshQuery = dtAllocNavMeshQuery();
	if (!navMeshQuery)
	{
		LogToUnity("Function: LoadNavData : 不能创建寻路查询对象Could not create Detour navMeshQuery!");
		return false;
	}

	// 初始化寻路查询对象，如果错误 直接返回
	dtStatus status = navMeshQuery->init(navMesh, 3072);
	if (dtStatusFailed(status))
	{
		LogToUnity("Function: LoadNavData : 不能初始化查询对象Could not init Detour navMesh query");
		return false;
	}

	return true;
}

void  UnLoadNavData()
{
	if (navMesh != NULL)
	{
		dtFreeNavMesh(navMesh);
		navMesh = NULL;
	}

	if (navMeshQuery != NULL)
	{
		dtFreeNavMeshQuery(navMeshQuery);
		navMeshQuery = NULL;
	}
	if (tileCache != NULL)
	{
		dtFreeTileCache(tileCache);
		tileCache = NULL;
	}
}



bool  FindNavPath(float* start, float* end, int jflag, float* &outarr, int &len)
{
	bool findPath = true;
	try
	{
		// 起始位置,终点位置
		float m_spos[3], m_epos[3];

		if (start == NULL || end == NULL)
		{
			return false;
		}

		// 拷贝数据，转为float数组
		dtVcopy(m_spos, start);
		dtVcopy(m_epos, end);

		// 过滤条件
		dtQueryFilter m_filter;
		m_filter.setIncludeFlags((unsigned int)jflag);
		m_filter.setExcludeFlags(0xffffffff ^ jflag);

		if (navMeshQuery)
		{
			// Change costs.default one
			m_filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
			m_filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 0.1f);
		}

		// 距离起始坐标和终点坐标最近的polygon多边形的id
		dtPolyRef m_startRef = 0;
		dtPolyRef m_endRef = 0;
		// 查找距离最近的多边形
		navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
		navMeshQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);


		// 如果没找到起点
		if (!m_startRef) {
			return false;
		}
		// 如果没找到终点, 说明终点在阻挡区域, raycast找直线到终点被阻挡的点, 重新找多边形
		if (!m_endRef) {
			// raycast找直线到终点被阻挡的点
			float* endBlock = NULL;
			float* normalDir = NULL;
			Raycast(start, end, jflag, endBlock, normalDir);
			float* end = endBlock;
			if (end == NULL)
			{
				return false;
			}

			dtVcopy(m_epos, end);
			delete[] end;
			end = NULL;

			navMeshQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
			findPath = false;
			
			if (!m_endRef) {
				return NULL;
			}
		}

		// 记录找到的路径上的多边形id的数组，起点->终点
		dtPolyRef m_polys[MAX_POLYS];
		int m_npolys;		// 多边形个数
		float m_straightPath[MAX_POLYS * 3];		// 直线路径上坐标数组
		unsigned char m_straightPathFlags[MAX_POLYS];
		dtPolyRef m_straightPathPolys[MAX_POLYS];		//找到的直线路径上多边形id的数组 
		float path[MAX_SMOOTH * 3];
		int m_nstraightPath = 0;		// 找到的直线距离坐标个数

		// 查询起点多边形->终点多边形的路径
		navMeshQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);

		// 如果不为0找到了 则找直线路径
		if (m_npolys) {
			// 找直线路径
			float epos[3];
			dtVcopy(epos, m_epos);
			if (m_polys[m_npolys - 1] != m_endRef)
				navMeshQuery->closestPointOnPoly(m_polys[m_npolys - 1], m_epos, epos, 0);

			navMeshQuery->findStraightPath(m_spos, epos, m_polys, m_npolys, m_straightPath, m_straightPathFlags, m_straightPathPolys, &m_nstraightPath, MAX_POLYS);
			

			int pos = 0;
			for (int i = 0; i < m_nstraightPath * 3 ;) {
				path[pos++] = m_straightPath[i++];
				path[pos++] = m_straightPath[i++];
				path[pos++] = m_straightPath[i++];
			}
		}

		/* 将float数组转为jfloatArray作为结果返回 */
		len = m_nstraightPath * 3;
		outarr = new float[len];
		for (int i = 0; i < len; i++)
		{
			outarr[i] = path[i];
		}

		if (outarr == NULL)
		{
			return false;
		}
	}
	catch (exception e) {
		findPath = false;
		LogToUnity(e.what());
	}
	catch (...)
	{
		LogToUnity("Function : FindNavPath Unkown Exception");
	}


	return findPath;
}

bool  Raycast(float* start, float* end, int jflag, float* &outarr, float* &oNormalDir)
{
	float* resultArr = NULL;
	float normalDir[3] = { 0 };
	try {
		// 起始位置,终点位置
		float m_spos[3], m_epos[3];


		if (start == NULL || end == NULL)
		{
			return false;
		}

		// 拷贝数据，转为float数组
		dtVcopy(m_spos, start);
		dtVcopy(m_epos, end);


		// 过滤条件
		dtQueryFilter m_filter;
		m_filter.setIncludeFlags((unsigned int)jflag);
		m_filter.setExcludeFlags(0xffffffff ^ jflag);

		// 距离起始坐标和终点坐标最近的polygon多边形的id
		dtPolyRef m_startRef = 0;

		// 查找距离最近的多边形
		navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);

		// 如果没找到
		if (!m_startRef) {
			return false;
		}

		float t;
		dtPolyRef path[MAX_POLYS];
		int pathCount;
		// raycast判断是否阻挡
	     dtStatus status = navMeshQuery->raycast(m_startRef, m_spos, m_epos, &m_filter, &t, normalDir, path, &pathCount, MAX_POLYS);
		 if (dtStatusFailed(status))
			 return false;

		int const arrayLen = 3;		// 数组长度
		float result[arrayLen];

		// 如果t=FLT_MAX则起点到终点无阻挡，能通过, 如果t=0则起点在阻挡区域, 返回终点坐标
		// 否则0<t<1.0则有阻挡，hitPoint = startPos + (endPos - startPos) * t
		// 以上是官网API附加说明，这里根据raycast的方法注释The hit parameter. (FLT_MAX if no wall hit.)简化处理
		if (t == FLT_MAX) {
			dtVcopy(result, m_epos);
		}
		// 否则0<t<1.0则有阻挡，hitPoint = startPos + (endPos - startPos) * t
		else if (0 < t && t < 1) {
			for (int i = 0; i < arrayLen; ++i)
				result[i] = m_spos[i] + (m_epos[i] - m_spos[i]) * t;
		}
		// 如果t=0则起点在阻挡区域, 直接返回起点
		else if (t == 0) {
			dtVcopy(result, m_spos);
		}
		outarr = new float[arrayLen];
		dtVcopy(outarr, result);
		if (result == NULL)
		{
			return false;
		}
	}

	catch (exception & e) {
		LogToUnity(e.what());
	}
	catch (...)
	{
		LogToUnity("Function : Raycast Unkown Exception");
	}

	oNormalDir = new float[3];
	::memcpy(oNormalDir, normalDir, sizeof(float) * 3);
	return true;
}


/**
* 判断某坐标是否在阻挡区域内
*/
bool  IsPosInBlock(float* jpos) {
	// 距离该坐标最近的polygon多边形的id
	dtPolyRef posRef = 0;
	try
	{
		// 格式转换
		float pos[3];
		dtVcopy(pos, jpos);
		//清理
		// 过滤条件
		dtQueryFilter m_filter;
		m_filter.setIncludeFlags(0xffff);
		m_filter.setExcludeFlags(0);

		float nearsetPos[3];

		// 查找距离最近的多边形
		navMeshQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &posRef, nearsetPos);
		
		if (posRef != 0)
		{
			for (int index = 0; index < 3; ++index)
			{
				//高度不准忽略
				if (index == 1)
				{
					continue;
				}
				if (fabsf(nearsetPos[index] - pos[index]) > 1e-6f)
				{
					return false;
				}
			}
		}

	}
	catch (exception e) {
		LogToUnity(e.what());
	}
	catch (...)
	{
		LogToUnity("Function : IsPosInBlock Unkown Exception");
	}

	return posRef == 0;
}

void  ClearIntPtr(void* pBuffer)
{
	if (NULL != pBuffer)
	{
		delete reinterpret_cast<int*>(pBuffer);
		pBuffer = NULL;
	}
}

void  FindNearestPoly(float* start, float* end, int flag, int &nearRef, float* &pt)
{
	float m_spos[3];
	dtVcopy(m_spos, start);
	dtQueryFilter m_filter;
	m_filter.setIncludeFlags((unsigned int)flag);
	m_filter.setExcludeFlags(0xffff ^ flag);
	pt = new float[3];
	navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, (dtPolyRef*)&nearRef, pt);
}


bool IsWalkable(float* start, int flag)
{
	float m_spos[3];
	dtVcopy(m_spos, start);
	dtQueryFilter m_filter;
	m_filter.setIncludeFlags((unsigned int)flag);
	m_filter.setExcludeFlags(0xffffffff ^ flag);
	dtPolyRef m_startRef = 0;
	navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, NULL);

	if (m_startRef == 0)
	{
		return false;
	}
	return navMeshQuery->isValidPolyRef(m_startRef, &m_filter);
}

float GetPolyHeight(float* point, int flag)
{
	float m_spos[3];
	dtVcopy(m_spos, point);
	dtQueryFilter m_filter;
	m_filter.setIncludeFlags((unsigned int)flag);
	m_filter.setExcludeFlags(0xffffffff ^ flag);
	dtPolyRef m_startRef = 0;
	navMeshQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, NULL);

	float height = 0.f;
	if (m_startRef == 0)
	{
		return height;
	}
	navMeshQuery->getPolyHeight(m_startRef, m_spos, &height);
	return height;
}

void SetPolyPickExtern(float x, float y, float z)
{
	m_polyPickExt[0] = x;
	m_polyPickExt[1] = y;
	m_polyPickExt[2] = z;
}

unsigned int AddCylinderObstacle(const float* pos, const float radius, const float height)
{
	if (tileCache != NULL)
	{
		dtObstacleRef* obstacle = new dtObstacleRef;

		float p[3];
		dtVcopy(p, pos);
		p[1] -= 0.5f;
		dtStatus status = tileCache->addObstacle(p,radius,height, obstacle);
		if (dtStatusSucceed(status))
		{
			return *obstacle;
		}
	}
	return 0;
}
unsigned int AddBoxObstacle(const float* bmin, const float* bmax)
{
	if (tileCache != NULL)
	{
		dtObstacleRef* obstacle = new dtObstacleRef;

		float pmin[3];
		dtVcopy(pmin, bmin);
		pmin[1] -= 0.5f;

		float pmax[3];
		dtVcopy(pmax, bmax);
		pmax[1] -= 0.5f;

		dtStatus status = tileCache->addBoxObstacle(pmin, pmax, obstacle);
		if (dtStatusSucceed(status))
		{
			return *obstacle;
		}
	}
	return 0;
}
bool RemoveObstacle(int obstacleRef)
{
	if (tileCache != NULL)
	{
		dtStatus status = tileCache->removeObstacle(obstacleRef);
		if (dtStatusSucceed(status))
		{
			return true;
		}
	}
	return false;
}
void UpdateObstacle()
{
	if (tileCache != NULL && navMesh != NULL)
	{
		tileCache->update(0,navMesh,0);
	}
}