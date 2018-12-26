#pragma once
#include <vector>
#include <string>
#include <cstdint>

#include "DetourNavMesh.h"

#define DEFAULT_CELL_SIZE (1.0f / 6.0f)
#define DEFAULT_TILE_SIZE (DEFAULT_CELL_SIZE*256)

static const float m_orig[3] = { 0.0f , 0.0f , 0.0f };

class UnityNavMeshLoader
{
public:
	UnityNavMeshLoader() : m_navMesh(dtAllocNavMesh()) {
	}

	~UnityNavMeshLoader() {
		if (m_navMesh) {
			dtFreeNavMesh(m_navMesh);
			m_navMesh = NULL;
		}
	}
	
	bool load(std::string filepath);

	float getCellSize() {
		return m_cellSize;
	}
	
	float getTileSize() {
		return m_tileSize;
	}

	float getWalkableHeight() {
		return m_walkableHeight;
	}

	float getWalkableRadius() {
		return m_walkableRadius;
	}

	float getWalkableClimb() {
		return m_walkableClimb;
	}

	int getMaxTile() {
		return m_maxTile;
	}

	dtNavMesh* getNavMesh() { 
		return m_navMesh; 
	}

private:
	bool loadText(char *content, int bufSize);
	bool loadBinary(char *content, int bufSize);

	std::vector<std::string> m_MeshData;
	float m_tileSize;
	float m_walkableHeight;
	float m_walkableRadius;
	float m_walkableClimb;
	float m_cellSize;
	int m_maxTile;

	dtNavMesh* m_navMesh;
	//std::string m_meshData[];
};

struct AssetMeshHeader {
	int magic;
	int version;
	int x, y;
	unsigned int userId;
	int polyCount;
	int vertCount;
	int detailMeshCount;
	int detailVertCount;
	int detailTriCount;
	int bvNodeCount;
	float bmin[3], bmax[3];
	float bvQuantFactor;
};

struct AssetPoly {
	unsigned short verts[6];
	unsigned short neis[6];
	unsigned int flags;
	unsigned char vertCount;
	unsigned char area;
};

typedef unsigned short dtPolyDetailIndex;

struct AssetPolyDetail {
	unsigned int vertBase;			///< The offset of the vertices in the dtMeshTile::detailVerts array.
	unsigned int triBase;			///< The offset of the triangles in the dtMeshTile::detailTris array.
	dtPolyDetailIndex vertCount;		///< The number of vertices in the sub-mesh.
	dtPolyDetailIndex triCount;			///< The number of triangles in the sub-mesh.
};

struct AssetLink {
	uint64_t ref;
	unsigned int next;
	unsigned char edge;
	unsigned char side;
	unsigned char bmin, bmax;
};

struct dtParam {
	float *verts;
	AssetPoly *polys;
	AssetPolyDetail *detailMeshes;
	float *detailVerts;
	dtPolyDetailIndex* detailTris;
	dtBVNode* bvTree;
};

struct OffMesh {
	float start[3];
	float end[3];
	float rad;
	unsigned int type;
	unsigned int area;
	unsigned int dir;
};



