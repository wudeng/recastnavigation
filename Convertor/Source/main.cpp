#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cassert>
#include "UnityNavMeshLoader.h"
#include "recast_wrap.h"

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;


void saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

dtNavMesh* loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

void readfile(const char * filename, unsigned char *&buf, int &sz) {
	FILE *fp = fopen(filename, "rb");
	fseek(fp, 0, SEEK_END);
	sz = ftell(fp);
	buf = new unsigned char[sz];
	fseek(fp, 0, SEEK_SET);
	assert(fread(buf, sz, 1, fp)==1);
}

bool verify(const char *filepath) {

	/*dtNavMesh * navMesh;
	navMesh = loadAll("yaml_navmesh.bin");
	dtNavMeshQuery *navQuery = dtAllocNavMeshQuery();
	navQuery->init(navMesh, 65535);
	dtQueryFilter filter;
	filter.setIncludeFlags(0xffff ^ 0x10);
	filter.setExcludeFlags(0);*/
	/*dtPolyRef startRef;
	dtPolyRef endRef;*/
	/*float spos[3];
	float epos[3];*/
	//dtPolyRef m_polys[256];
	//int npolys;
	unsigned char *buf;
	int sz;
	readfile(filepath, buf, sz);
	NavMesh mesh;
	NavMesh_create(&mesh, buf, sz);
	NavMeshQuery query;
	NavMeshQuery_create(&query, mesh, 2048);
	NavStatus status;
	int foundPath = 0;
	int total = 1000;

	for (int i = 0; i < total; i++) {
		//navQuery->findRandomPoint(&filter, frand, &startRef, spos);
		//navQuery->findRandomPoint(&filter, frand, &endRef, epos);
		/*dtStatus status = navQuery->findPath(startRef, endRef, spos, epos, &filter, m_polys, &npolys, 255);
		if (dtStatusSucceed(status)) {
			printf("npolys = %d\n", npolys);
		}*/

		NavPoint spos;
		NavPoint epos;
		status = NavMeshQuery_findRandomPoint(query, spos);
		status = NavMeshQuery_findRandomPoint(query, epos);
		int pathCount = 0;
		NavPoint* path;
		status = NavMeshQuery_findStraightPath(query, spos, epos, &path, &pathCount);
		if (!NavStatus_succeed(status)) {
			fprintf(stderr, "convert error!!\n");
			return false;
		}
		foundPath += (pathCount > 0) ? 1 : 0;
	}
	printf("FoundPath: %d/%d\n", foundPath, total);
	return true;
}

int main(int argc, const char **argv) {
	if (argc < 2) {
		fprintf(stderr, "Usage: ./Convertor clientMesh serverMesh\n");
		return -1;
	}

	const char *clientMesh = argv[1];
	const char *serverMesh = argv[2];
	
	UnityNavMeshLoader loader;
	loader.load(clientMesh);
	saveAll(serverMesh, loader.getNavMesh());
	if (!verify(serverMesh)) {
		fprintf(stderr, "\033[40;31mconvert NavMesh %s error\033[0m\n", clientMesh);
		return -1;
	}
	else {
		fprintf(stderr, "\033[40;32mconvert NavMesh %s to %s succeed\033[0m\n", clientMesh, serverMesh);
		return 0;
	}
}
