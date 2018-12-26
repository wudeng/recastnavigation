#include <cfloat>
#include <string>
#include <cstdio>
#include <cstring>
#include <cassert>
#include "Recast.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "UnityNavMeshLoader.h"
#include "DetourNavMeshQuery.h"
#include "Sample.h"

static float ts = DEFAULT_TILE_SIZE;
static const float walkableClimb = 0.4166667f;
static int tsc = 0;


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

// 判断pt位于bmin和bmax表示的aabb在xz平面的方位。一共9种可能性。0-7表示在外面，表示8个方位，从x+开始，逆时针。0xff表示在aabb里面。
static unsigned char classifyOffMeshPoint(const float* pt, const float* bmin, const float* bmax)
{
	static const unsigned char XP = 1<<0;
	static const unsigned char ZP = 1<<1;
	static const unsigned char XM = 1<<2;
	static const unsigned char ZM = 1<<3;	

	unsigned char outcode = 0; 
	outcode |= (pt[0] >= bmax[0]) ? XP : 0;
	outcode |= (pt[2] >= bmax[2]) ? ZP : 0;
	outcode |= (pt[0] < bmin[0])  ? XM : 0;
	outcode |= (pt[2] < bmin[2])  ? ZM : 0;

	switch (outcode)
	{
	case XP: return 0;
	case XP|ZP: return 1;
	case ZP: return 2;
	case XM|ZP: return 3;
	case XM: return 4;
	case XM|ZM: return 5;
	case ZM: return 6;
	case XP|ZM: return 7;
	};

	return 0xff;	
}

bool addTile(AssetMeshHeader *header, dtParam *param, dtNavMesh *mesh) {
	std::vector<OffMesh> *offmesh = param->offmesh;
	// Classify off-mesh connection points. We store only the connections
	// whose start point is inside the tile.
	unsigned char* offMeshConClass = 0;
	int storedOffMeshConCount = 0;
	int offMeshConLinkCount = 0;

	unsigned int offMeshConCount = 0;
	if (offmesh) {
		offMeshConCount = offmesh->size();
	}

	if (offMeshConCount > 0U)
	{
		offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char)*offMeshConCount * 2, DT_ALLOC_TEMP);
		if (!offMeshConClass)
			return false;

		// 计算3D的AABB盒边界。因为tile中只有xz平面的，所以还需要计算y方向的
		// Find tight heigh bounds, used for culling out off-mesh start locations.
		float hmin = FLT_MAX;
		float hmax = -FLT_MAX;

		if (param->detailVerts && header->detailVertCount)
		{
			for (int i = 0; i < header->detailVertCount; ++i)
			{
				const float height = param->detailVerts[i * 3 + 1];
				hmin = dtMin(hmin, height);
				hmax = dtMax(hmax, height);
			}
		}
		else
		{
			for (int i = 0; i < header->vertCount; ++i)
			{
				const float height = param->verts[i * 3 + 1];
				hmin = dtMin(hmin, height);
				hmax = dtMax(hmax, height);
			}
		}
		hmin -= walkableClimb;
		hmax += walkableClimb;
		float bmin[3], bmax[3];
		dtVcopy(bmin, header->bmin);
		dtVcopy(bmax, header->bmax);
		bmin[1] = hmin;
		bmax[1] = hmax;

		for (unsigned int i = 0; i < offMeshConCount; ++i)
		{
			const float* p0 = offmesh->at(i).start;
			const float* p1 = offmesh->at(i).end;
			// 判断xz方向
			offMeshConClass[i * 2 + 0] = classifyOffMeshPoint(p0, bmin, bmax);
			offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, bmin, bmax);

			// 判断y方向
			// Zero out off-mesh start positions which are not even potentially touching the mesh.
			if (offMeshConClass[i * 2 + 0] == 0xff)
			{
				if (p0[1] < bmin[1] || p0[1] > bmax[1])
					offMeshConClass[i * 2 + 0] = 0;
			}

			// Count how many links should be allocated for off-mesh connections.
			if (offMeshConClass[i * 2 + 0] == 0xff)
				offMeshConLinkCount++;
			if (offMeshConClass[i * 2 + 1] == 0xff)
				offMeshConLinkCount++;

			if (offMeshConClass[i * 2 + 0] == 0xff)
				storedOffMeshConCount++;
		}
	}

	// Off-mesh connectionss are stored as polygons, adjust values.
	const int totPolyCount = header->polyCount + storedOffMeshConCount;
	const int totVertCount = header->vertCount + storedOffMeshConCount * 2;

	// Find portal edges which are at tile borders.
	int edgeCount = 0;
	int portalCount = 0;
	for (int i = 0; i < header->polyCount; ++i)
	{
		const AssetPoly* p = &param->polys[i];
		edgeCount += p->vertCount;
		for (int j = 0; j < p->vertCount; ++j)
		{
			if (p->neis[j] & DT_EXT_LINK)
			{
				portalCount++;
			}
		}
	}

	// Calculate data size
	const int maxLinkCount = edgeCount + portalCount * 2 + offMeshConLinkCount * 2 + offMeshConLinkCount * 2;
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float) * 3 * totVertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly) * totPolyCount);
	const int linksSize = dtAlign4(sizeof(dtLink) * maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
	const int bvTreeSize = dtAlign4(sizeof(dtBVNode)*header->polyCount * 2);
	const int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * storedOffMeshConCount);

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

	dtMeshHeader* hd = dtGetThenAdvanceBufferPointer<dtMeshHeader>(d, headerSize);
	float* navVerts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	dtPoly* navPolys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	d += linksSize; // Ignore links; just leave enough space for them. They'll be created on load.
	dtPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	float* navDVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	unsigned char* navDTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	dtBVNode* navBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvTreeSize);
	dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshConsSize);


	// Store header
	hd->magic = DT_NAVMESH_MAGIC;
	hd->version = DT_NAVMESH_VERSION;
	hd->x = header->x;
	hd->y = header->y;
	hd->layer = 0;
	hd->userId = header->userId;
	hd->polyCount = totPolyCount;
	hd->vertCount = totVertCount;
	hd->maxLinkCount = maxLinkCount;
	dtVcopy(hd->bmin, header->bmin);
	dtVcopy(hd->bmax, header->bmax);
	hd->detailMeshCount = header->detailMeshCount;
	hd->detailVertCount = header->detailVertCount;
	hd->detailTriCount = header->detailTriCount;
	hd->bvQuantFactor = header->bvQuantFactor;
	hd->offMeshBase = header->polyCount;
	hd->walkableHeight = 2;
	hd->walkableRadius = 0.5;
	hd->walkableClimb = walkableClimb;
	hd->offMeshConCount = storedOffMeshConCount;
	hd->bvNodeCount = header->polyCount * 2;

	const int offMeshVertsBase = header->vertCount;
	const int offMeshPolyBase = header->polyCount;

	memcpy(navVerts, param->verts, dtAlign4(sizeof(float)) * 3 * hd->vertCount);
	// Off-mesh link vertices.
	int n = 0;
	for (unsigned int i = 0; i < offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i * 2 + 0] == 0xff)
		{
			OffMesh o = offmesh->at(i);
			float* v = &navVerts[(offMeshVertsBase + n * 2) * 3];
			dtVcopy(&v[0], o.start);
			dtVcopy(&v[3], o.end);
			n++;
		}
	}

	// Store polygons
	// Mesh polys
	for (int i = 0; i < hd->polyCount; ++i)
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
	// Off-mesh connection vertices.
	n = 0;
	for (unsigned int i = 0; i < offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i * 2 + 0] == 0xff)
		{
			dtPoly* p = &navPolys[offMeshPolyBase + n];
			OffMesh o = offmesh->at(i);
			p->vertCount = 2;
			p->verts[0] = (unsigned short)(offMeshVertsBase + n * 2 + 0);
			p->verts[1] = (unsigned short)(offMeshVertsBase + n * 2 + 1);
			p->flags = SAMPLE_POLYFLAGS_JUMP; //TODO
			p->setArea(o.area);
			p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
			n++;
		}
	}

	for (int i = 0; i < header->detailMeshCount; ++i)
	{
		dtPolyDetail& dtl = navDMeshes[i];
		AssetPolyDetail *from = &param->detailMeshes[i];

		dtl.vertBase = from->vertBase;
		if (from->vertCount > 255 || from->triCount > 255) {
			fprintf(stderr, "errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrror! invalid data\n");
			exit(-1);
		}
		dtl.vertCount = (unsigned char)from->vertCount; // warning
		dtl.triBase = from->triBase;
		dtl.triCount = (unsigned char)from->triCount;
	}
	memcpy(navDVerts, param->detailVerts, sizeof(float) * 3 * hd->detailVertCount);
	for (int i = 0; i < 4 * hd->detailTriCount; ++i) {
		navDTris[i] = (unsigned char)param->detailTris[i];
	}

	memcpy(navBvtree, param->bvTree, dtAlign4(sizeof(dtBVNode) * hd->polyCount * 2));

	// Store Off-Mesh connections.
	n = 0;
	for (unsigned int i = 0; i < offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i * 2 + 0] == 0xff)
		{
			dtOffMeshConnection* con = &offMeshCons[n];
			con->poly = (unsigned short)(offMeshPolyBase + n);
			// Copy connection end-points.
			OffMesh o = offmesh->at(i);
			dtVcopy(&con->pos[0], o.start);
			dtVcopy(&con->pos[3], o.end);
			con->rad = o.rad;
			con->flags = o.dir ? DT_OFFMESH_CON_BIDIR : 0;
			con->side = offMeshConClass[i * 2 + 1];
			// con->userId = 0;
			n++;
		}
	}

	mesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);

	dtFree(offMeshConClass);
	return true;
}

static const float eps = 0.0001f;

void update_ts(float sz) {
	if (tsc == 0) {
		ts = sz;
		tsc = 1;
	}
	else {
		tsc += dtAbs(ts - sz) < eps ? 1 : -1;
	}
}

void parseTile(dtNavMesh *mesh, char *buf, int len, std::vector<OffMesh> *offmesh = NULL) {
	AssetMeshHeader *header = (AssetMeshHeader *)buf;

	/*
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
	printf("tileSize = %f, %d\n", ts, tsc);
	printf("bvQuantFactor = %f\n", header->bvQuantFactor);
	//rcVmin(orig, header->bmin);*/

	float w = header->bmax[0] - header->bmin[0];
	float h = header->bmax[2] - header->bmin[2];
	float sz = (w > h) ? w : h;
	update_ts(sz);

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

	if (len != 0 && d - (unsigned char *)buf != len) {
		fprintf(stderr, "errrrrrrrrrrrrrrrrrrror, invalid data, input len = %d, calc len = %d \n", len, (int)(d - (unsigned char *)buf));
		exit(-1);
	}

	dtParam param;
	param.verts = navVerts;
	param.polys = navPolys;
	param.detailMeshes = navDMeshes;
	param.detailVerts = navDVerts;
	param.detailTris = navDTris;
	param.bvTree = navBvtree;
	param.offmesh = offmesh;

	addTile(header, &param, mesh);	
}

void setTileSize(dtNavMesh *nav) {
	dtNavMeshParams *pa = (dtNavMeshParams *)nav->getParams();
	pa->tileWidth = ts;
	pa->tileHeight = ts;
}

bool UnityNavMeshLoader::load(std::string filepath) {
	char *content;
	int bufSize;
	if (!readFile(filepath, content, bufSize)) {
		return false;
	}
	if (content[0] == '%') {
		return loadText(content, bufSize);
	}
	else {
		return loadBinary(content, bufSize);
	}
}


bool UnityNavMeshLoader::loadText(char *content, int bufSize) {

	char *src = content;
	char *srcEnd = content + bufSize;
	char *row = new char[bufSize];

	// offmesh
	OffMesh o;
	std::vector<OffMesh> offmesh;

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

		// offmesh
		else if (strncmp(row, "- m_Start: ", 11) == 0) {
			sscanf(row + 11, "{x: %f, y: %f, z: %f}", &o.start[0], &o.start[1], &o.start[2]);
		}
		else if (strncmp(row, "m_End: ", 7) == 0) {
			sscanf(row + 7, "{x: %f, y: %f, z: %f}", &o.end[0], &o.end[1], &o.end[2]);
		}
		else if (strncmp(row, "m_Radius: ", 10) == 0) {
			sscanf(row + 10, "%f", &o.rad);
		}
		else if (strncmp(row, "m_LinkType: ", 12) == 0) {
			sscanf(row + 12, "%u", &o.type);
		}
		else if (strncmp(row, "m_Area: ", 8) == 0) {
			sscanf(row + 8, "%u", &o.area);
		}
		else if (strncmp(row, "m_LinkDirection: ", 17) == 0) {
			sscanf(row + 17, "%u", &o.dir);
			offmesh.push_back(o);
		}
		else {
			//
		}
	}

	delete[] content;
	content = NULL;

	//int offmeshCount = offmesh.size();
	m_maxTile = m_MeshData.size();
	m_cellSize = (m_cellSize == 0) ? DEFAULT_CELL_SIZE : m_cellSize;
	m_tileSize = (m_tileSize == 0) ? DEFAULT_TILE_SIZE : m_tileSize;
	m_walkableClimb = (m_walkableClimb == 0) ? 0.4166667f : m_walkableClimb;
	m_walkableRadius = (m_walkableRadius == 0) ? 0.5f : m_walkableRadius;
	m_walkableHeight = (m_walkableHeight == 0) ? 2.0f : m_walkableHeight;

	dtNavMeshParams params;
	//params.tileWidth = m_tileSize;
	//params.tileHeight = m_tileSize;
	rcVcopy(params.orig, m_orig);

	int tileBits = rcMin((int)ilog2(nextPow2(m_maxTile)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	params.maxTiles = 1 << tileBits;
	params.maxPolys = 1 << polyBits;

	m_navMesh->init(&params);

	for (int i = 0; i < m_maxTile; i++) {
		int len = str2hex(m_MeshData[i], row);
		parseTile(m_navMesh, row, len, &offmesh);
	}
	setTileSize(m_navMesh);
	printf("tileSize = %f, tsc/tiles = %d/%d\n", ts, tsc, m_maxTile);


	delete[] row;
	row = NULL;

	return true;
}

bool UnityNavMeshLoader::loadBinary(char *content, int bufSize) {
	std::vector<int> index;
	std::vector<int> length;
	int *ptr;

	for (int i = 1; i < bufSize - 4; i++) {
		ptr = (int *)(content + i);
		if (*ptr == DT_NAVMESH_MAGIC && *(ptr + 1) == 16) {
			index.push_back(i);
			length.push_back(*(ptr - 1));
			i += sizeof(AssetMeshHeader);
		}
	}

	m_maxTile = index.size();
	m_cellSize = DEFAULT_CELL_SIZE;
	//m_tileSize = DEFAULT_TILE_SIZE;

	dtNavMeshParams params;
	//params.tileWidth = m_tileSize;
	//params.tileHeight = m_tileSize;
	rcVcopy(params.orig, m_orig);

	int tileBits = rcMin((int)ilog2(nextPow2(m_maxTile)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	params.maxTiles = 1 << tileBits;
	params.maxPolys = 1 << polyBits;

	m_navMesh->init(&params);

	for (int i = 0; i < m_maxTile; i++) {
		parseTile(m_navMesh, content+index[i], length[i]);
	}
	setTileSize(m_navMesh);
	printf("tileSize = %f, tsc/tiles = %d/%d\n", ts, tsc, m_maxTile);
	
	delete[] content;
	content = NULL;

	return true;
}
