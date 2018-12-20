#include <string>
#include <cstdio>
#include <cassert>
#include "Recast.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "YamlAssetLoader.h"
#include "DetourNavMeshQuery.h"
#include "YamlAssetLoader.h"


int hexval(char ch) {
	if (ch >= '0' && ch <= '9') {
		return ch - '0';
	}
	else if (ch >= 'a' && ch <= 'f') {
		return ch - 'a' + 10;
	}
	else //if (ch >= 'A' && ch <= 'F') 
	{
		return ch - 'A' + 10;
	}
}


int str2hex(std::string s, char *dest) {
	int sz = s.size();
	const char *str = s.c_str();
	int j = 0;
	for (int i = 0; i < sz - 1; i += 2, j++) {
		dest[j] = hexval(str[i]) << 4 | hexval(str[i + 1]);
	}
	return j;
}

inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}


static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
		case '\\':
			break;
		case '\n':
			if (start) break;
			done = true;
			break;
		case '\r':
			break;
		case '\t':
		case ' ':
			if (start) break;
			// else falls through
		default:
			start = false;
			row[n++] = c;
			if (n >= len - 1)
				done = true;
			break;
		}
	}
	row[n] = '\0';
	return buf;
}


bool readFile(std::string filepath, char *&buf, int &bufSize) {
	FILE *fp = fopen(filepath.c_str(), "rb");
	if (!fp) {
		return false;
	}
	if (fseek(fp, 0, SEEK_END) != 0) {
		fclose(fp);
		return false;
	}
	bufSize = ftell(fp);
	if (bufSize < 0) {
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0) {
		fclose(fp);
		return false;
	}
	buf = new char[bufSize];
	if (!buf) {
		fclose(fp);
		return false;
	}
	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);
	if (readLen != 1) {
		delete[] buf;
		return false;
	}
	return true;
}




void printMagic(int magic) {
	char *p = (char *)&magic;
	printf("magic = ");
	for (int i = 0; i < 4; i++) {
		printf("%c ", *p++);
	}
	printf("\n");
}

bool addTile(AssetMeshHeader *h, dtParam *param, dtNavMesh *mesh) {
	// Calculate data size
	int maxLinkCount = 6 * h->polyCount * 3;
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float) * 3 * h->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*h->polyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*h->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float) * 3 * h->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * h->detailTriCount);
	const int bvTreeSize = dtAlign4(sizeof(dtBVNode)*h->polyCount * 2);
	const int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * 0);

	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
		detailMeshesSize + detailVertsSize + detailTrisSize +
		bvTreeSize + offMeshConsSize;

	unsigned char* data = (unsigned char*)dtAlloc(sizeof(unsigned char)*dataSize, DT_ALLOC_PERM);
	if (!data)
	{
		return false;
	}
	memset(data, 0, dataSize);

	unsigned char* d = data;

	dtMeshHeader* header = dtGetThenAdvanceBufferPointer<dtMeshHeader>(d, headerSize);
	float* navVerts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	dtPoly* navPolys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	d += linksSize; // Ignore links; just leave enough space for them. They'll be created on load.
	dtPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	float* navDVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	unsigned char* navDTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	dtBVNode* navBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvTreeSize);
	dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshConsSize);


	// Store header
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->x = h->x;
	header->y = h->y;
	header->layer = 0;
	header->userId = h->userId;
	header->polyCount = h->polyCount;
	header->vertCount = h->vertCount;
	header->maxLinkCount = maxLinkCount;
	dtVcopy(header->bmin, h->bmin);
	dtVcopy(header->bmax, h->bmax);
	header->detailMeshCount = h->detailMeshCount;
	header->detailVertCount = h->detailVertCount;
	header->detailTriCount = h->detailTriCount;
	header->bvQuantFactor = h->bvQuantFactor;
	header->offMeshBase = h->polyCount;
	header->walkableHeight = 2;
	header->walkableRadius = 0.5;
	header->walkableClimb = 0.4166667f;
	header->offMeshConCount = 0;
	header->bvNodeCount = h->polyCount * 2;

	memcpy(navVerts, param->verts, dtAlign4(sizeof(float)) * 3 * header->vertCount);

	// Store polygons
	// Mesh polys
	for (int i = 0; i < header->polyCount; ++i)
	{
		dtPoly* p = &navPolys[i];
		AssetPoly *from = &param->polys[i];
		p->vertCount = from->vertCount;
		p->flags = from->flags;
		p->setArea(from->area);
		p->setType(DT_POLYTYPE_GROUND);
		memcpy(p->verts, from->verts, sizeof(unsigned short) * DT_VERTS_PER_POLYGON);
		memcpy(p->neis, from->neis, sizeof(unsigned short) * DT_VERTS_PER_POLYGON);
	}

	for (int i = 0; i < header->detailMeshCount; ++i)
	{
		dtPolyDetail& dtl = navDMeshes[i];
		AssetPolyDetail *from = &param->detailMeshes[i];

		dtl.vertBase = from->vertBase;
		if (from->vertCount > 255 || from->triCount > 255) {
			printf("errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrror! invalid data\n");
			exit(-1);
		}
		dtl.vertCount = (unsigned char)from->vertCount; // warning
		dtl.triBase = from->triBase;
		dtl.triCount = (unsigned char)from->triCount;
	}
	memcpy(navDVerts, param->detailVerts, sizeof(float) * 3 * header->detailVertCount);
	for (int i = 0; i < 4 * header->detailTriCount; ++i) {
		navDTris[i] = (unsigned char)param->detailTris[i];
	}

	memcpy(navBvtree, param->bvTree, dtAlign4(sizeof(dtBVNode) * header->polyCount * 2));

	mesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);

	return true;
}

void parseTile(dtNavMesh *mesh, char *buf, int len) {
	AssetMeshHeader *header = (AssetMeshHeader *)buf;
	printMagic(header->magic);
	printf("version = %d\n", header->version);
	printf("x = %d, y = %d\n", header->x, header->y);
	printf("userId = %d\n", header->userId);
	printf("polyCount = %d\n", header->polyCount);
	printf("vertCount = %d\n", header->vertCount);
	printf("detailMeshCount = %d\n", header->detailMeshCount);
	printf("detailVertCount = %d\n", header->detailVertCount);
	printf("detailTriCount = %d\n", header->detailTriCount);
	printf("bvNodeCount = %d\n", header->bvNodeCount);
	printf("bmin = %f, %f, %f\n", header->bmin[0], header->bmin[1], header->bmin[2]);
	printf("bmax = %f, %f, %f\n", header->bmax[0], header->bmax[1], header->bmax[2]);
	float w = header->bmax[0] - header->bmin[0];
	float h = header->bmax[0] - header->bmin[0];
	printf("tilewidth, tileHeight = %f, %f, %f, %d\n", w, h, header->bmin[0] / DEFAULT_TILE_SIZE, header->x);
	printf("bvQuantFactor = %f\n", header->bvQuantFactor);
	//rcVmin(orig, header->bmin);

	// Calculate data size
	const int headerSize = dtAlign4(sizeof(AssetMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
	const int polysSize = dtAlign4(sizeof(AssetPoly)*header->polyCount);
	const int detailMeshesSize = dtAlign4(sizeof(AssetPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(dtPolyDetailIndex) * 4 * header->detailTriCount);
	const int bvTreeSize = dtAlign4(sizeof(dtBVNode)*header->polyCount * 2);

	unsigned char * d = (unsigned char *) buf;
	d += headerSize;
	float *navVerts = (float *)d; d += vertsSize;
	AssetPoly *navPolys = (AssetPoly *)d; d += polysSize;
	AssetPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<AssetPolyDetail>(d, detailMeshesSize);
	float* navDVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	dtPolyDetailIndex* navDTris = dtGetThenAdvanceBufferPointer<dtPolyDetailIndex>(d, detailTrisSize);
	dtBVNode* navBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvTreeSize);

	if (d - (unsigned char *)buf != len) {
		printf("errrrrrrrrrrrrrrrrrrror, invalid data, input len = %d, calc len = %d \n", len, d - (unsigned char *)buf);
		exit(-1);
	}

	dtParam param;
	param.verts = navVerts;
	param.polys = navPolys;
	param.detailMeshes = navDMeshes;
	param.detailVerts = navDVerts;
	param.detailTris = navDTris;
	param.bvTree = navBvtree;

	addTile(header, &param, mesh);	
}



bool YamlAssetLoader::load(std::string filepath) {

	char *content;
	int bufSize;
	if (!readFile(filepath, content, bufSize)) {
		return false;
	}

	char *src = content;
	char *srcEnd = content + bufSize;
	char *row = new char[bufSize];

	while (src < srcEnd) {
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, bufSize);
		if (strncmp(row, "- m_MeshData: ", 14) == 0) {
			m_MeshData.push_back(row + 14);
		}
		else if (strncmp(row, "tileSize: ", 10) == 0) {
			sscanf(row + 10, "%f", &m_tileSize);
		}
		else if (strncmp(row, "walkableHeight: ", 16) == 0) {
			sscanf(row + 16, "%f", &m_walkableHeight);
		}
		else if (strncmp(row, "walkableRadius: ", 16) == 0) {
			sscanf(row + 16, "%f", &m_walkableRadius);
		}
		else if (strncmp(row, "walkableClimb: ", 15) == 0) {
			sscanf(row + 15, "%f", &m_walkableClimb);
		}
		else if (strncmp(row, "cellSize: ", 10) == 0) {
			sscanf(row + 10, "%f", &m_cellSize);
		}
		else {
			//
		}
	}

	delete[] content;
	content = NULL;

	
	m_maxTile = m_MeshData.size();
	m_cellSize = (m_cellSize == 0) ? DEFAULT_CELL_SIZE : m_cellSize;
	m_tileSize = (m_tileSize == 0) ? DEFAULT_TILE_SIZE : m_tileSize;
	m_walkableClimb = (m_walkableClimb == 0) ? 0.4166667f : m_walkableClimb;
	m_walkableRadius = (m_walkableRadius == 0) ? 0.5f : m_walkableRadius;
	m_walkableHeight = (m_walkableHeight == 0) ? 2.0f : m_walkableHeight;

	dtNavMeshParams params;
	params.tileWidth = m_tileSize;
	params.tileHeight = m_tileSize;
	rcVcopy(params.orig, m_orig);

	int tileBits = rcMin((int)ilog2(nextPow2(m_maxTile)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	params.maxTiles = 1 << tileBits;
	params.maxPolys = 1 << polyBits;
	
	m_navMesh->init(&params);

	for (int i = 0; i < m_maxTile; i++) {
		int len = str2hex(m_MeshData[i], row);
		printf("str size = %d, binary size = %d\n", m_MeshData[i].size(), len);
		parseTile(m_navMesh, row, len);	
	}

	delete[] row;
	row = NULL;

	return true;
}