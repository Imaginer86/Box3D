/*
* Copyright (c) 2006-2009 Erin Catto http://www.Box3D.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "Box3D/Common/b3BlockAllocator.h"
#include <limits.h>
#include <string.h>
#include <stddef.h>

int32 b3BlockAllocator::s_blockSizes[b3_blockSizes] = 
{
	16,		// 0
	32,		// 1
	64,		// 2
	96,		// 3
	128,	// 4
	160,	// 5
	192,	// 6
	224,	// 7
	256,	// 8
	320,	// 9
	384,	// 10
	448,	// 11
	512,	// 12
	640,	// 13
};
uint8 b3BlockAllocator::s_blockSizeLookup[b3_maxBlockSize + 1];
bool b3BlockAllocator::s_blockSizeLookupInitialized;

struct b3Chunk
{
	int32 blockSize;
	b3Block* blocks;
};

struct b3Block
{
	b3Block* next;
};

b3BlockAllocator::b3BlockAllocator()
{
	b3Assert(b3_blockSizes < UCHAR_MAX);

	m_chunkSpace = b3_chunkArrayIncrement;
	m_chunkCount = 0;
	m_chunks = (b3Chunk*)b3Alloc(m_chunkSpace * sizeof(b3Chunk));
	
	memset(m_chunks, 0, m_chunkSpace * sizeof(b3Chunk));
	memset(m_freeLists, 0, sizeof(m_freeLists));

	if (s_blockSizeLookupInitialized == false)
	{
		int32 j = 0;
		for (int32 i = 1; i <= b3_maxBlockSize; ++i)
		{
			b3Assert(j < b3_blockSizes);
			if (i <= s_blockSizes[j])
			{
				s_blockSizeLookup[i] = (uint8)j;
			}
			else
			{
				++j;
				s_blockSizeLookup[i] = (uint8)j;
			}
		}

		s_blockSizeLookupInitialized = true;
	}
}

b3BlockAllocator::~b3BlockAllocator()
{
	for (int32 i = 0; i < m_chunkCount; ++i)
	{
		b3Free(m_chunks[i].blocks);
	}

	b3Free(m_chunks);
}

void* b3BlockAllocator::Allocate(int32 size)
{
	if (size == 0)
		return nullptr;

	b3Assert(0 < size);

	if (size > b3_maxBlockSize)
	{
		return b3Alloc(size);
	}

	int32 index = s_blockSizeLookup[size];
	b3Assert(0 <= index && index < b3_blockSizes);

	if (m_freeLists[index])
	{
		b3Block* block = m_freeLists[index];
		m_freeLists[index] = block->next;
		return block;
	}
	else
	{
		if (m_chunkCount == m_chunkSpace)
		{
			b3Chunk* oldChunks = m_chunks;
			m_chunkSpace += b3_chunkArrayIncrement;
			m_chunks = (b3Chunk*)b3Alloc(m_chunkSpace * sizeof(b3Chunk));
			memcpy(m_chunks, oldChunks, m_chunkCount * sizeof(b3Chunk));
			memset(m_chunks + m_chunkCount, 0, b3_chunkArrayIncrement * sizeof(b3Chunk));
			b3Free(oldChunks);
		}

		b3Chunk* chunk = m_chunks + m_chunkCount;
		chunk->blocks = (b3Block*)b3Alloc(b3_chunkSize);
#if defined(_DEBUG)
		memset(chunk->blocks, 0xcd, b3_chunkSize);
#endif
		int32 blockSize = s_blockSizes[index];
		chunk->blockSize = blockSize;
		int32 blockCount = b3_chunkSize / blockSize;
		b3Assert(blockCount * blockSize <= b3_chunkSize);
		for (int32 i = 0; i < blockCount - 1; ++i)
		{
			b3Block* block = (b3Block*)((int8*)chunk->blocks + blockSize * i);
			b3Block* next = (b3Block*)((int8*)chunk->blocks + blockSize * (i + 1));
			block->next = next;
		}
		b3Block* last = (b3Block*)((int8*)chunk->blocks + blockSize * (blockCount - 1));
		last->next = nullptr;

		m_freeLists[index] = chunk->blocks->next;
		++m_chunkCount;

		return chunk->blocks;
	}
}

void b3BlockAllocator::Free(void* p, int32 size)
{
	if (size == 0)
	{
		return;
	}

	b3Assert(0 < size);

	if (size > b3_maxBlockSize)
	{
		b3Free(p);
		return;
	}

	int32 index = s_blockSizeLookup[size];
	b3Assert(0 <= index && index < b3_blockSizes);

#ifdef _DEBUG
	// Verify the memory address and size is valid.
	int32 blockSize = s_blockSizes[index];
	bool found = false;
	for (int32 i = 0; i < m_chunkCount; ++i)
	{
		b3Chunk* chunk = m_chunks + i;
		if (chunk->blockSize != blockSize)
		{
			b3Assert(	(int8*)p + blockSize <= (int8*)chunk->blocks ||
						(int8*)chunk->blocks + b3_chunkSize <= (int8*)p);
		}
		else
		{
			if ((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + b3_chunkSize)
			{
				found = true;
			}
		}
	}

	b3Assert(found);

	memset(p, 0xfd, blockSize);
#endif

	b3Block* block = (b3Block*)p;
	block->next = m_freeLists[index];
	m_freeLists[index] = block;
}

void b3BlockAllocator::Clear()
{
	for (int32 i = 0; i < m_chunkCount; ++i)
	{
		b3Free(m_chunks[i].blocks);
	}

	m_chunkCount = 0;
	memset(m_chunks, 0, m_chunkSpace * sizeof(b3Chunk));

	memset(m_freeLists, 0, sizeof(m_freeLists));
}
