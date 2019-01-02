#ifndef detour_wrap_h
#define detour_wrap_h

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int NavStatus;
int NavStatus_succeed(NavStatus status);
int NavStatus_(NavStatus status);

struct NavMeshQueryImpl;
typedef void* NavMesh;
typedef struct NavMeshQueryImpl* NavMeshQuery;
typedef float NavPoint[3]; // [x, y, z]

NavStatus NavMesh_create(NavMesh* mesh, const void* buf, size_t sz);
void NavMesh_release(NavMesh mesh);
int NavMesh_getMaxTiles(NavMesh mesh);

NavStatus NavMeshQuery_create(NavMeshQuery* query, NavMesh mesh, const int maxNodes);
void NavMeshQuery_release(NavMeshQuery query);

/*
** 查找两点间的路径，如果不可达，则返回最接近终点的路径。
**
** [in]    query       dtNavMeshQuery
** [in]    startPos    Path start position. [(x, y, z)]
** [in]    endPos      Path end position. [(x, y, z)]
** [out]   path        Points describing the straight path. [(x, y, z) * pathCount].
** [out]   pathCount   The number of points in the straight path.
*/
NavStatus NavMeshQuery_findStraightPath(NavMeshQuery query, const NavPoint startPos, const NavPoint endPos,
    NavPoint** path, int* pathCount);

/*
** 查找两点间的沿表面路径，如果不可达或缓存较小，则返回最接近终点的路径
**
** [in]    query       dtNavMeshQuery
** [in]    startPos    Path start position. [(x, y, z)]
** [in]    endPos      Path end position. [(x, y, z)]
** [in]    step        distance between nearby points
** [out]   path        Points describing the follow path. [(x, y, z) * pathCount].
** [out]   pathCount   The number of points in the follow path.
*/
NavStatus NavMeshQuery_findFollowPath(NavMeshQuery q, const NavPoint startPos, const NavPoint endPos, const float step,
    NavPoint** path, int* pathCount);

/*
** 返回导航图上的随机一个点
**
** [in]    query       dtNavMeshQuery
** [out]   pos         The random location.
*/
NavStatus NavMeshQuery_findRandomPoint(NavMeshQuery q, NavPoint pos, dtPolyRef &poly);

/*
** 搜索center点extent半径内，落在多边形上的点
**
** [in]    query  dtNavMeshQuery
** [in]    center 搜索中心点. [(x, y, z)]
** [in]    extent 搜索半径. [(x, y, z)]
** [out]   pos    搜索结果
*/
NavStatus NavMeshQuery_findNearestPointOnPoly(NavMeshQuery q, const NavPoint center, const NavPoint extent, NavPoint pos);
#ifdef __cplusplus
}
#endif

#endif // detour_wrap_h
