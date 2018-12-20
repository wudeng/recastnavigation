#include <cstdlib>
#include <cstring>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "recast_wrap.h"

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshQueryImpl {
    dtNavMeshQuery* navQuery;
    dtQueryFilter filter;
    int maxPolys; // polys的长度
    int maxPoints; // points的长度
    dtPolyRef* polys; // 寻路过程中使用的临时缓存
    NavPoint* points; // 寻路过程中的路点缓存
    dtPolyRef* polys2; // 寻路过程中使用的临时缓存
    NavPoint* points2; // 寻路过程中的路点缓存
};

struct NavMeshSetHeader {
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader {
    dtTileRef tileRef;
    int dataSize;
};

// Returns a random number [0..1]
static float frand()
{
    //	return ((float)(rand() & 0xffff)/(float)0xffff);
    return (float)rand() / (float)RAND_MAX;
}

// 把cur向前移动n, 若成功返回移动前的指针，否则返回NULL
static void* offset_n(unsigned char*& cur, size_t& sz, size_t n)
{
    if (sz < n) {
        return NULL;
    }

    unsigned char* old = cur;
    cur = cur + n;
    sz = sz - n;
    return old;
}

// Returns 1 if status is success.
int NavStatus_succeed(NavStatus status)
{
    return (status & DT_SUCCESS) != 0;
}

// 从dump到文件的mesh数据，还原出NavMesh的内存结构
NavStatus NavMesh_create(NavMesh* mesh, const void* buf, size_t sz)
{
    unsigned char* stream = (unsigned char*)buf;
    NavMeshSetHeader* header = (NavMeshSetHeader*)offset_n(stream, sz, sizeof(NavMeshSetHeader));
    if (!header) {
        return DT_FAILURE | DT_INVALID_PARAM;
    }
    if (header->magic != NAVMESHSET_MAGIC) {
        return DT_FAILURE | DT_WRONG_MAGIC;
    }
    if (header->version != NAVMESHSET_VERSION) {
        return DT_FAILURE | DT_WRONG_VERSION;
    }

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        return DT_FAILURE;
    }
    dtStatus status = navMesh->init(&header->params);
    if (!dtStatusSucceed(status)) {
        goto error;
    }

    // Read tiles.
    for (int i = 0; i < header->numTiles; ++i) {
        NavMeshTileHeader* tileHeader = (NavMeshTileHeader*)offset_n(stream, sz, sizeof(NavMeshTileHeader));
        if (!tileHeader) {
            status = DT_FAILURE | DT_INVALID_PARAM;
            goto error;
        }

        if (!tileHeader->tileRef || !tileHeader->dataSize) {
            status = DT_FAILURE | DT_INVALID_PARAM;
            goto error;
        }

        void* src = offset_n(stream, sz, tileHeader->dataSize);
        if (!src) {
            status = DT_FAILURE | DT_INVALID_PARAM;
            goto error;
        }
        unsigned char* data = (unsigned char*)dtAlloc(tileHeader->dataSize, DT_ALLOC_PERM);
        if (!data) {
            status = DT_FAILURE;
            goto error;
        }
        memcpy(data, src, tileHeader->dataSize);
        navMesh->addTile(data, tileHeader->dataSize, DT_TILE_FREE_DATA, tileHeader->tileRef, 0);
    }

    *mesh = navMesh;
    return DT_SUCCESS;
error:
    if (navMesh) {
        dtFreeNavMesh(navMesh);
    }
    return status;
}

void NavMesh_release(NavMesh mesh)
{
    dtNavMesh* navMesh = (dtNavMesh*)mesh;
    dtFreeNavMesh(navMesh);
}

int NavMesh_getMaxTiles(NavMesh mesh)
{
    dtNavMesh* navMesh = (dtNavMesh*)mesh;
    return navMesh->getMaxTiles();
}
/*
    [in]	nav	Pointer to the dtNavMesh object to use for all queries.
    [in]	maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
*/
NavStatus NavMeshQuery_create(NavMeshQuery* query, NavMesh mesh, const int maxNodes)
{
    dtStatus status = DT_SUCCESS;
    NavMeshQueryImpl* impl = (NavMeshQueryImpl*)calloc(1, sizeof(NavMeshQueryImpl));
    if (!impl) {
        status = DT_FAILURE | DT_OUT_OF_MEMORY;
        goto error;
    }
    impl->filter = dtQueryFilter();

    impl->navQuery = dtAllocNavMeshQuery();
    if (!impl->navQuery) {
        status = DT_FAILURE | DT_OUT_OF_MEMORY;
        goto error;
    }

    status = impl->navQuery->init((dtNavMesh*)mesh, maxNodes);
    if (!dtStatusSucceed(status)) {
        goto error;
    }

    impl->polys = (dtPolyRef*)calloc(maxNodes, sizeof(dtPolyRef));
    if (!impl->polys) {
        status = DT_FAILURE | DT_OUT_OF_MEMORY;
        goto error;
    }

    impl->polys2 = (dtPolyRef*)calloc(maxNodes, sizeof(dtPolyRef));
    if (!impl->polys2) {
        status = DT_FAILURE | DT_OUT_OF_MEMORY;
        goto error;
    }

    impl->maxPolys = maxNodes;

    impl->points = (NavPoint*)calloc(maxNodes, sizeof(NavPoint));
    if (!impl->points) {
        status = DT_FAILURE | DT_OUT_OF_MEMORY;
        goto error;
    }

    impl->points2 = (NavPoint*)calloc(maxNodes, sizeof(NavPoint));
    if (!impl->points2) {
        status = DT_FAILURE | DT_OUT_OF_MEMORY;
        goto error;
    }

    impl->maxPoints = maxNodes;

    *query = impl;
    return DT_SUCCESS;

error:
    NavMeshQuery_release(impl);
    return status;
}

void NavMeshQuery_release(NavMeshQuery query)
{
    if (query == NULL) {
        return;
    }

    if (query->navQuery) {
        dtFreeNavMeshQuery(query->navQuery);
        query->navQuery = NULL;
    }

    if (query->polys) {
        free(query->polys);
        query->polys = NULL;
    }

    if (query->polys2) {
        free(query->polys2);
        query->polys2 = NULL;
    }

    if (query->points) {
        free(query->points);
        query->points = NULL;
    }

    if (query->points2) {
        free(query->points2);
        query->points2 = NULL;
    }
    free(query);
}

/*
** 查找两点间的路径，如果不可达或缓存较小，则返回最接近终点的路径。
**
** [in]    query       dtNavMeshQuery
** [in]    startPos    Path start position. [(x, y, z)]
** [in]    endPos      Path end position. [(x, y, z)]
** [out]   path        Points describing the straight path. [(x, y, z) * pathCount].
** [out]   pathCount   The number of points in the straight path.
*/
NavStatus NavMeshQuery_findStraightPath(NavMeshQuery q, const NavPoint startPos, const NavPoint endPos,
    NavPoint** path, int* pathCount)
{
    dtStatus status = DT_SUCCESS;
    dtPolyRef startRef, endRef; // 起点/终点所在的多边形
    float halfExtents[3] = { 2, 4, 2 }; // 沿着每个轴的搜索长度

    q->navQuery->findNearestPoly(startPos, halfExtents, &q->filter, &startRef, 0);
    q->navQuery->findNearestPoly(endPos, halfExtents, &q->filter, &endRef, 0);
    if (!startRef || !endRef) {
        return DT_FAILURE | DT_INVALID_PARAM; // 起点或终点附近没有导航图
    }

    int npolys = 0;
    status = q->navQuery->findPath(startRef, endRef, (float*)startPos, (float*)endPos, &q->filter, q->polys, &npolys, q->maxPolys);
    if (!npolys) {
        return status;
    }

    // In case of partial path, make sure the end point is clamped to the last polygon.
    float epos[3];
    dtVcopy(epos, endPos);
    if (q->polys[npolys - 1] != endRef)
        q->navQuery->closestPointOnPoly(q->polys[npolys - 1], endPos, epos, 0);

    status = q->navQuery->findStraightPath(startPos, epos, q->polys, npolys,
        (float*)q->points, NULL, NULL, pathCount, q->maxPoints, 0);
    *path = q->points;
    return status;
}

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
    NavPoint** path, int* pathCount)
{
    dtStatus status = DT_SUCCESS;
    dtPolyRef startRef, endRef; // 起点/终点所在的多边形
    float halfExtents[3] = { 2, 4, 2 }; // 沿着每个轴的搜索长度

    q->navQuery->findNearestPoly(startPos, halfExtents, &q->filter, &startRef, 0);
    q->navQuery->findNearestPoly(endPos, halfExtents, &q->filter, &endRef, 0);
    if (!startRef || !endRef) {
        return DT_FAILURE | DT_INVALID_PARAM; // 起点或终点附近没有导航图
    }

    int npolys = 0;
    status = q->navQuery->findPath(startRef, endRef, (float*)startPos, (float*)endPos, &q->filter, q->polys, &npolys, q->maxPolys);
    if (!npolys) {
        return status;
    }

    // In case of partial path, make sure the end point is clamped to the last polygon.
    float epos[3];
    dtVcopy(epos, endPos);
    if (q->polys[npolys - 1] != endRef)
        q->navQuery->closestPointOnPoly(q->polys[npolys - 1], endPos, epos, 0);

    int nsteerPath = 0;
    status = q->navQuery->findStraightPath(startPos, epos, q->polys, npolys,
        (float*)q->points2, NULL, q->polys2, &nsteerPath, q->maxPoints, DT_STRAIGHTPATH_ALL_CROSSINGS);

    // 根据step插点
    float iterPos[3];
    q->navQuery->closestPointOnPoly(startRef, startPos, iterPos, 0);
    dtVcopy(q->points[0], iterPos);
    *pathCount = 1;

    int ns = 1;
    while (ns < nsteerPath && *pathCount < q->maxPoints) {
        float delta[3], len;
        dtVsub(delta, q->points2[ns], iterPos);
        len = dtMathSqrtf(dtVdot(delta, delta));
        if (len < step) {
            len = 1;
        } else {
            len = step / len;
        }

        float moveTgt[3];
        dtVmad(moveTgt, iterPos, delta, len);

        float h = 0;
        int qstatus = q->navQuery->getPolyHeight(q->polys2[ns-1], moveTgt, &h);
        if(dtStatusSucceed(qstatus)) {
            moveTgt[1] = h;
        }
        dtVcopy(iterPos, moveTgt);
        dtVcopy(q->points[*pathCount], iterPos);

        if(len >= 1) ++ns;
        *pathCount = *pathCount + 1;
    }

    if (ns < nsteerPath && *pathCount == q->maxPoints) {
        status |= DT_BUFFER_TOO_SMALL;
    }
    *path = q->points;
    return status;
}

/*
** 返回导航图上的随机一个点
**
** [in]    query       dtNavMeshQuery
** [out]   pos         The random location.
*/
NavStatus NavMeshQuery_findRandomPoint(NavMeshQuery q, NavPoint pos)
{
    dtPolyRef ref;

    /*
    filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
    filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
    filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL);
    */

    q->navQuery->findRandomPoint(&q->filter, frand, &ref, pos);
    if(!ref) {
        return DT_FAILURE | DT_INVALID_PARAM; // 没有搜索到合适的点
    }
    return DT_SUCCESS;
}

/*
** 搜索center点extent半径内，落在多边形上的点
**
** [in]    query  dtNavMeshQuery
** [in]    center 搜索中心点. [(x, y, z)]
** [in]    extent 搜索半径. [(x, y, z)]
** [out]   pos    搜索结果
*/
NavStatus NavMeshQuery_findNearestPointOnPoly(NavMeshQuery q, const NavPoint center, const NavPoint extent, NavPoint pos) {
    dtPolyRef ref;
    q->navQuery->findNearestPoly(center, extent, &q->filter, &ref, pos);
    if(!ref) {
        return DT_FAILURE | DT_INVALID_PARAM; // 没有搜索到合适的点
    }
    return DT_SUCCESS;
}

