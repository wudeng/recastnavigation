#pragma once
#include <vector>
#include <string>

#include "DetourNavMesh.h"

#define DEFAULT_TILE_SIZE 42.666668f
#define DEFAULT_CELL_SIZE 0.16666667f

class YamlAssetLoader
{
public:
	YamlAssetLoader() : m_navMesh(dtAllocNavMesh()) {
	}

	~YamlAssetLoader() {
		if (m_navMesh) {
			dtFreeNavMesh(m_navMesh);
			m_navMesh = NULL;
		}
	}
	
	bool load(std::string filepath);

	std::vector<std::string> getMeshData();

	float getCellSize() {
		return m_cellSize;
	}
	
	float getTileSize() {
		return m_tileSize;
	}

	void setTileSize(float sz) {
		m_tileSize = sz;
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
	std::vector<std::string> m_MeshData;
	float m_tileSize;
	float m_walkableHeight;
	float m_walkableRadius;
	float m_walkableClimb;
	float m_cellSize;
	int m_maxTile;

	float m_orig[3] = { 0.0f , 0.0f , 0.0f };
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



