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

#ifndef b3_BLOCK_ALLOCATOR_H
#define b3_BLOCK_ALLOCATOR_H

#include "Box3D/Common/b3Settings.h"

const int32 b3_chunkSize = 16 * 1024;
const int32 b3_maxBlockSize = 640;
const int32 b3_blockSizes = 14;
const int32 b3_chunkArrayIncrement = 128;

struct b3Block;
struct b3Chunk;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class b3BlockAllocator
{
public:
	b3BlockAllocator();
	~b3BlockAllocator();

	/// Allocate memory. This will use b3Alloc if the size is larger than b3_maxBlockSize.
	void* Allocate(int32 size);

	/// Free memory. This will use b3Free if the size is larger than b3_maxBlockSize.
	void Free(void* p, int32 size);

	void Clear();

private:

	b3Chunk* m_chunks;
	int32 m_chunkCount;
	int32 m_chunkSpace;

	b3Block* m_freeLists[b3_blockSizes];

	static int32 s_blockSizes[b3_blockSizes];
	static uint8 s_blockSizeLookup[b3_maxBlockSize + 1];
	static bool s_blockSizeLookupInitialized;
};

#endif
