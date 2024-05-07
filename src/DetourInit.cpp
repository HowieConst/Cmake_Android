#define _CRT_SECURE_NO_WARNINGS
#include "DetourInit.h"
#include <cstdio>
#include <cstring>
#include <sstream>
#include <iostream>
#include <fstream>

static const int TILECACHESET_MAGIC = 'T'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'TSET';
static const int TILECACHESET_VERSION = 1;


LinearAllocator* m_talloc = new LinearAllocator(32000);
FastLZCompressor* m_tcomp = new FastLZCompressor;
MeshProcess* m_tmproc = new MeshProcess;

dtNavMesh* dtLoadNavMesh(const unsigned char* ndata,dtTileCache*   tileCache)
{

	TileCacheSetHeader* header = new TileCacheSetHeader();
	const unsigned char* pos = ndata;
	memcpy(header, pos, sizeof(TileCacheSetHeader));
	pos += sizeof(TileCacheSetHeader);



	if (header->magic != TILECACHESET_MAGIC)
	{
		return 0;
	}
	if (header->version != TILECACHESET_VERSION)
	{
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		return 0;
	}

	dtStatus status = mesh->init(&header->meshParams);
	if (dtStatusFailed(status))
	{
		return 0;
	}
	if (!tileCache)
	{
		return 0;
	}

	

	status = tileCache->init(&header->cacheParams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header->numTiles; ++i)
	{
		TileCacheTileHeader* tileHeader = new TileCacheTileHeader();

		memcpy(tileHeader, pos, sizeof(TileCacheTileHeader));

		pos += sizeof(TileCacheTileHeader);

		if (!tileHeader->tileRef || !tileHeader->dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader->dataSize, DT_ALLOC_PERM);

		if (!data) break;


		memset(data, 0, tileHeader->dataSize);

		memcpy(data, pos, tileHeader->dataSize);

		dtCompressedTileRef tile = 0;
		dtStatus addTileStatus = tileCache->addTile(data, tileHeader->dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		if (dtStatusFailed(addTileStatus))
		{
			dtFree(data);
		}

		if (tile)
			tileCache->buildNavMeshTile(tile, mesh);

		pos += tileHeader->dataSize;

		delete tileHeader;
	}

	delete header;
	return mesh;
}

