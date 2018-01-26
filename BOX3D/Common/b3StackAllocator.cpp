/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include "Box3D/Common/b3StackAllocator.h"
#include "Box3D/Common/b3Math.h"

b3StackAllocator::b3StackAllocator()
{
	m_index = 0;
	m_allocation = 0;
	m_maxAllocation = 0;
	m_entryCount = 0;
}

b3StackAllocator::~b3StackAllocator()
{
	b3Assert(m_index == 0);
	b3Assert(m_entryCount == 0);
}

void* b3StackAllocator::Allocate(int32 size)
{
	b3Assert(m_entryCount < b3_maxStackEntries);

	b3StackEntry* entry = m_entries + m_entryCount;
	entry->size = size;
	if (m_index + size > b3_stackSize)
	{
		entry->data = (char*)b3Alloc(size);
		entry->usedMalloc = true;
	}
	else
	{
		entry->data = m_data + m_index;
		entry->usedMalloc = false;
		m_index += size;
	}

	m_allocation += size;
	m_maxAllocation = b3Max(m_maxAllocation, m_allocation);
	++m_entryCount;

	return entry->data;
}

void b3StackAllocator::Free(void* p)
{
	b3Assert(m_entryCount > 0);
	b3StackEntry* entry = m_entries + m_entryCount - 1;
	b3Assert(p == entry->data);
	if (entry->usedMalloc)
	{
		b3Free(p);
	}
	else
	{
		m_index -= entry->size;
	}
	m_allocation -= entry->size;
	--m_entryCount;

	p = nullptr;
}

int32 b3StackAllocator::GetMaxAllocation() const
{
	return m_maxAllocation;
}
