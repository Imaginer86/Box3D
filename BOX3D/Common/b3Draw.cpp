#include "Box3D/Common/b3Draw.h"

b3Draw::b3Draw()
{
	m_drawFlags = 0;
}

void b3Draw::SetFlags(uint32 flags)
{
	m_drawFlags = flags;
}

uint32 b3Draw::GetFlags() const
{
	return m_drawFlags;
}

void b3Draw::AppendFlags(uint32 flags)
{
	m_drawFlags |= flags;
}

void b3Draw::ClearFlags(uint32 flags)
{
	m_drawFlags &= ~flags;
}
