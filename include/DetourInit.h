#pragma once
#include "DetourNavMesh.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourCommon.h"
#include "fastlz.h"
#include "DetourNavMeshBuilder.h"



/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
	SAMPLE_POLYAREA_OBSTACLE1,
	SAMPLE_POLYAREA_OBSTACLE2,
	SAMPLE_POLYAREA_OBSTACLE3,
	SAMPLE_POLYAREA_OBSTACLE4,
	SAMPLE_POLYAREA_OBSTACLE5,
	SAMPLE_POLYAREA_OBSTACLE6,
	SAMPLE_POLYAREA_OBSTACLE7,
	SAMPLE_POLYAREA_OBSTACLE8,
	SAMPLE_POLYAREA_OBSTACLE9,
	SAMPLE_POLYAREA_OBSTACLE10,
	SAMPLE_POLYAREA_OBSTACLE11,
	SAMPLE_POLYAREA_OBSTACLE12,
	SAMPLE_POLYAREA_OBSTACLE13,
	SAMPLE_POLYAREA_OBSTACLE14,
	SAMPLE_POLYAREA_OBSTACLE15,
	SAMPLE_POLYAREA_OBSTACLE16,
	SAMPLE_POLYAREA_OBSTACLE17,
	SAMPLE_POLYAREA_OBSTACLE18,

};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon

	SAMPLE_POLYFLAGS_OBSTACLE1 = 0x20,		// 32
	SAMPLE_POLYFLAGS_OBSTACLE2 = 0x40,		// 64
	SAMPLE_POLYFLAGS_OBSTACLE3 = 0x80,		// 128
	SAMPLE_POLYFLAGS_OBSTACLE4 = 0x100,		// 256		

	SAMPLE_POLYFLAGS_OBSTACLE5 = 0x200,		// 512
	SAMPLE_POLYFLAGS_OBSTACLE6 = 0x400,		// 1024
	SAMPLE_POLYFLAGS_OBSTACLE7 = 0x800,		// 2048
	SAMPLE_POLYFLAGS_OBSTACLE8 = 0x1000,    // 4096		

	SAMPLE_POLYFLAGS_OBSTACLE9 = 0x2000,
	SAMPLE_POLYFLAGS_OBSTACLE10 = 0x4000,
	SAMPLE_POLYFLAGS_OBSTACLE11 = 0x8000,
	SAMPLE_POLYFLAGS_OBSTACLE12 = 0x10000,


	SAMPLE_POLYFLAGS_OBSTACLE13 = 0x20000,
	SAMPLE_POLYFLAGS_OBSTACLE14 = 0x40000,
	SAMPLE_POLYFLAGS_OBSTACLE15 = 0x80000,
	SAMPLE_POLYFLAGS_OBSTACLE16 = 0x100000,

	SAMPLE_POLYFLAGS_OBSTACLE17 = 0x200000,
	SAMPLE_POLYFLAGS_OBSTACLE18 = 0x400000,


	SAMPLE_POLYFLAGS_ALL = 0xffffffff	// All abilities.
};


#pragma pack(push,4)

typedef void(*FuncPtr)(const char *);

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};
struct TileCacheSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
};
struct TileCacheTileHeader
{
	dtCompressedTileRef tileRef;
	int dataSize;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};
struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize * 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
		unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void* const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
		unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{

	inline MeshProcess()
	{
	}

	inline void init()
	{
	}

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned int* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE1)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE1;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE2)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE2;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE3)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE3;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE4)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE4;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE5)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE5;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE6)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE6;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE7)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE7;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_OBSTACLE8)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_OBSTACLE8;
			}
		}
	}
};

#pragma pack(pop)

struct dtNavPath
{
public:
	enum { MAX_POLYS = 128 };
public:
	float spos[3];
	float epos[3];

	float straightPath[MAX_POLYS * 3];
	unsigned char straightPathFlags[MAX_POLYS];
	dtPolyRef straightPathPolys[MAX_POLYS];
	int nstraightPath;

};
extern dtNavMesh* dtLoadNavMesh(const unsigned char* path, dtTileCache*  tileCache);
