// b3DynamicBvh implementation by Nathanael Presson

#include "b3DynamicBvh.h"

//
typedef b3AlignedObjectArray<b3DbvtNode*> b3NodeArray;
typedef b3AlignedObjectArray<const b3DbvtNode*> b3ConstNodeArray;

//
struct b3DbvtNodeEnumerator : b3DynamicBvh::ICollide
{
	b3ConstNodeArray nodes;
	void Process(const b3DbvtNode* n) { nodes.push_back(n); }
};

//
static B3_DBVT_INLINE int b3IndexOf(const b3DbvtNode* node)
{
	return (node->parent->childs[1] == node);
}

//
static B3_DBVT_INLINE b3DbvtVolume b3Merge(const b3DbvtVolume& a,
										   const b3DbvtVolume& b)
{
#if (B3_DBVT_MERGE_IMPL == B3_DBVT_IMPL_SSE)
	B3_ATTRIBUTE_ALIGNED16(char locals[sizeof(b3DbvtAabbMm)]);
	b3DbvtVolume& res = *(b3DbvtVolume*)locals;
#else
	b3DbvtVolume res;
#endif
	b3Merge(a, b, res);
	return (res);
}

// volume+edge lengths
static B3_DBVT_INLINE b3Scalar b3Size(const b3DbvtVolume& a)
{
	const b3Vector3 edges = a.Lengths();
	return (edges.x * edges.y * edges.z +
			edges.x + edges.y + edges.z);
}

//
static void b3GetMaxDepth(const b3DbvtNode* node, int depth, int& maxdepth)
{
	if (node->isinternal())
	{
		b3GetMaxDepth(node->childs[0], depth + 1, maxdepth);
		b3GetMaxDepth(node->childs[1], depth + 1, maxdepth);
	}
	else
		maxdepth = b3Max(maxdepth, depth);
}

//
static B3_DBVT_INLINE void b3DeleteNode(b3DynamicBvh* pdbvt,
										b3DbvtNode* node)
{
	b3AlignedFree(pdbvt->m_free);
	pdbvt->m_free = node;
}

//
static void b3RecurseDeleteNode(b3DynamicBvh* pdbvt,
								b3DbvtNode* node)
{
	if (!node->isleaf())
	{
		b3RecurseDeleteNode(pdbvt, node->childs[0]);
		b3RecurseDeleteNode(pdbvt, node->childs[1]);
	}
	if (node == pdbvt->m_root) pdbvt->m_root = 0;
	b3DeleteNode(pdbvt, node);
}

//
static B3_DBVT_INLINE b3DbvtNode* b3CreateNode(b3DynamicBvh* pdbvt,
											   b3DbvtNode* parent,
											   void* data)
{
	b3DbvtNode* node;
	if (pdbvt->m_free)
	{
		node = pdbvt->m_free;
		pdbvt->m_free = 0;
	}
	else
	{
		node = new (b3AlignedAlloc(sizeof(b3DbvtNode), 16)) b3DbvtNode();
	}
	node->parent = parent;
	node->data = data;
	node->childs[1] = 0;
	return (node);
}

//
static B3_DBVT_INLINE b3DbvtNode* b3CreateNode(b3DynamicBvh* pdbvt,
											   b3DbvtNode* parent,
											   const b3DbvtVolume& volume,
											   void* data)
{
	b3DbvtNode* node = b3CreateNode(pdbvt, parent, data);
	node->volume = volume;
	return (node);
}

//
static B3_DBVT_INLINE b3DbvtNode* b3CreateNode(b3DynamicBvh* pdbvt,
											   b3DbvtNode* parent,
											   const b3DbvtVolume& volume0,
											   const b3DbvtVolume& volume1,
											   void* data)
{
	b3DbvtNode* node = b3CreateNode(pdbvt, parent, data);
	b3Merge(volume0, volume1, node->volume);
	return (node);
}

//
static void b3InsertLeaf(b3DynamicBvh* pdbvt,
						 b3DbvtNode* root,
						 b3DbvtNode* leaf)
{
	if (!pdbvt->m_root)
	{
		pdbvt->m_root = leaf;
		leaf->parent = 0;
	}
	else
	{
		if (!root->isleaf())
		{
			do
			{
				root = root->childs[b3Select(leaf->volume,
											 root->childs[0]->volume,
											 root->childs[1]->volume)];
			} while (!root->isleaf());
		}
		b3DbvtNode* prev = root->parent;
		b3DbvtNode* node = b3CreateNode(pdbvt, prev, leaf->volume, root->volume, 0);
		if (prev)
		{
			prev->childs[b3IndexOf(root)] = node;
			node->childs[0] = root;
			root->parent = node;
			node->childs[1] = leaf;
			leaf->parent = node;
			do
			{
				if (!prev->volume.Contain(node->volume))
					b3Merge(prev->childs[0]->volume, prev->childs[1]->volume, prev->volume);
				else
					break;
				node = prev;
			} while (0 != (prev = node->parent));
		}
		else
		{
			node->childs[0] = root;
			root->parent = node;
			node->childs[1] = leaf;
			leaf->parent = node;
			pdbvt->m_root = node;
		}
	}
}

//
static b3DbvtNode* b3RemoveLeaf(b3DynamicBvh* pdbvt,
								b3DbvtNode* leaf)
{
	if (leaf == pdbvt->m_root)
	{
		pdbvt->m_root = 0;
		return (0);
	}
	else
	{
		b3DbvtNode* parent = leaf->parent;
		b3DbvtNode* prev = parent->parent;
		b3DbvtNode* sibling = parent->childs[1 - b3IndexOf(leaf)];
		if (prev)
		{
			prev->childs[b3IndexOf(parent)] = sibling;
			sibling->parent = prev;
			b3DeleteNode(pdbvt, parent);
			while (prev)
			{
				const b3DbvtVolume pb = prev->volume;
				b3Merge(prev->childs[0]->volume, prev->childs[1]->volume, prev->volume);
				if (b3NotEqual(pb, prev->volume))
				{
					prev = prev->parent;
				}
				else
					break;
			}
			return (prev ? prev : pdbvt->m_root);
		}
		else
		{
			pdbvt->m_root = sibling;
			sibling->parent = 0;
			b3DeleteNode(pdbvt, parent);
			return (pdbvt->m_root);
		}
	}
}

//
static void b3FetchLeaves(b3DynamicBvh* pdbvt,
						  b3DbvtNode* root,
						  b3NodeArray& leaves,
						  int depth = -1)
{
	if (root->isinternal() && depth)
	{
		b3FetchLeaves(pdbvt, root->childs[0], leaves, depth - 1);
		b3FetchLeaves(pdbvt, root->childs[1], leaves, depth - 1);
		b3DeleteNode(pdbvt, root);
	}
	else
	{
		leaves.push_back(root);
	}
}

static bool b3LeftOfAxis(const b3DbvtNode* node,
						 const b3Vector3& org,
						 const b3Vector3& axis)
{
	return b3Dot(axis, node->volume.Center() - org) <= 0;
}

// Partitions leaves such that leaves[0, n) are on the
// left of axis, and leaves[n, count) are on the right
// of axis. returns N.
static int b3Split(b3DbvtNode** leaves,
				   int count,
				   const b3Vector3& org,
				   const b3Vector3& axis)
{
	int begin = 0;
	int end = count;
	for (;;)
	{
		while (begin != end && b3LeftOfAxis(leaves[begin], org, axis))
		{
			++begin;
		}

		if (begin == end)
		{
			break;
		}

		while (begin != end && !b3LeftOfAxis(leaves[end - 1], org, axis))
		{
			--end;
		}

		if (begin == end)
		{
			break;
		}

		// swap out of place nodes
		--end;
		b3DbvtNode* temp = leaves[begin];
		leaves[begin] = leaves[end];
		leaves[end] = temp;
		++begin;
	}

	return begin;
}

//
static b3DbvtVolume b3Bounds(b3DbvtNode** leaves,
							 int count)
{
#if B3_DBVT_MERGE_IMPL == B3_DBVT_IMPL_SSE
	B3_ATTRIBUTE_ALIGNED16(char locals[sizeof(b3DbvtVolume)]);
	b3DbvtVolume& volume = *(b3DbvtVolume*)locals;
	volume = leaves[0]->volume;
#else
	b3DbvtVolume volume = leaves[0]->volume;
#endif
	for (int i = 1, ni = count; i < ni; ++i)
	{
		b3Merge(volume, leaves[i]->volume, volume);
	}
	return (volume);
}

//
static void b3BottomUp(b3DynamicBvh* pdbvt,
					   b3DbvtNode** leaves,
					   int count)
{
	while (count > 1)
	{
		b3Scalar minsize = B3_INFINITY;
		int minidx[2] = {-1, -1};
		for (int i = 0; i < count; ++i)
		{
			for (int j = i + 1; j < count; ++j)
			{
				const b3Scalar sz = b3Size(b3Merge(leaves[i]->volume, leaves[j]->volume));
				if (sz < minsize)
				{
					minsize = sz;
					minidx[0] = i;
					minidx[1] = j;
				}
			}
		}
		b3DbvtNode* n[] = {leaves[minidx[0]], leaves[minidx[1]]};
		b3DbvtNode* p = b3CreateNode(pdbvt, 0, n[0]->volume, n[1]->volume, 0);
		p->childs[0] = n[0];
		p->childs[1] = n[1];
		n[0]->parent = p;
		n[1]->parent = p;
		leaves[minidx[0]] = p;
		leaves[minidx[1]] = leaves[count - 1];
		--count;
	}
}

//
static b3DbvtNode* b3TopDown(b3DynamicBvh* pdbvt,
							 b3DbvtNode** leaves,
							 int count,
							 int bu_treshold)
{
	static const b3Vector3 axis[] = {b3MakeVector3(1, 0, 0),
									 b3MakeVector3(0, 1, 0),
									 b3MakeVector3(0, 0, 1)};
	b3Assert(bu_treshold > 1);
	if (count > 1)
	{
		if (count > bu_treshold)
		{
			const b3DbvtVolume vol = b3Bounds(leaves, count);
			const b3Vector3 org = vol.Center();
			int partition;
			int bestaxis = -1;
			int bestmidp = count;
			int splitcount[3][2] = {{0, 0}, {0, 0}, {0, 0}};
			int i;
			for (i = 0; i < count; ++i)
			{
				const b3Vector3 x = leaves[i]->volume.Center() - org;
				for (int j = 0; j < 3; ++j)
				{
					++splitcount[j][b3Dot(x, axis[j]) > 0 ? 1 : 0];
				}
			}
			for (i = 0; i < 3; ++i)
			{
				if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0))
				{
					const int midp = (int)b3Fabs(b3Scalar(splitcount[i][0] - splitcount[i][1]));
					if (midp < bestmidp)
					{
						bestaxis = i;
						bestmidp = midp;
					}
				}
			}
			if (bestaxis >= 0)
			{
				partition = b3Split(leaves, count, org, axis[bestaxis]);
				b3Assert(partition != 0 && partition != count);
			}
			else
			{
				partition = count / 2 + 1;
			}
			b3DbvtNode* node = b3CreateNode(pdbvt, 0, vol, 0);
			node->childs[0] = b3TopDown(pdbvt, &leaves[0], partition, bu_treshold);
			node->childs[1] = b3TopDown(pdbvt, &leaves[partition], count - partition, bu_treshold);
			node->childs[0]->parent = node;
			node->childs[1]->parent = node;
			return (node);
		}
		else
		{
			b3BottomUp(pdbvt, leaves, count);
			return (leaves[0]);
		}
	}
	return (leaves[0]);
}

//
static B3_DBVT_INLINE b3DbvtNode* b3Sort(b3DbvtNode* n, b3DbvtNode*& r)
{
	b3DbvtNode* p = n->parent;
	b3Assert(n->isinternal());
	if (p > n)
	{
		const int i = b3IndexOf(n);
		const int j = 1 - i;
		b3DbvtNode* s = p->childs[j];
		b3DbvtNode* q = p->parent;
		b3Assert(n == p->childs[i]);
		if (q)
			q->childs[b3IndexOf(p)] = n;
		else
			r = n;
		s->parent = n;
		p->parent = n;
		n->parent = q;
		p->childs[0] = n->childs[0];
		p->childs[1] = n->childs[1];
		n->childs[0]->parent = p;
		n->childs[1]->parent = p;
		n->childs[i] = p;
		n->childs[j] = s;
		b3Swap(p->volume, n->volume);
		return (p);
	}
	return (n);
}

//
// Api
//

//
b3DynamicBvh::b3DynamicBvh()
{
	m_root = 0;
	m_free = 0;
	m_lkhd = -1;
	m_leaves = 0;
	m_opath = 0;
}

//
b3DynamicBvh::~b3DynamicBvh()
{
	clear();
}

//
void b3DynamicBvh::clear()
{
	if (m_root)
		b3RecurseDeleteNode(this, m_root);
	b3AlignedFree(m_free);
	m_free = 0;
	m_lkhd = -1;
	m_stkStack.clear();
	m_opath = 0;
}

//
void b3DynamicBvh::optimizeBottomUp()
{
	if (m_root)
	{
		b3NodeArray leaves;
		leaves.reserve(m_leaves);
		b3FetchLeaves(this, m_root, leaves);
		b3BottomUp(this, &leaves[0], leaves.size());
		m_root = leaves[0];
	}
}

//
void b3DynamicBvh::optimizeTopDown(int bu_treshold)
{
	if (m_root)
	{
		b3NodeArray leaves;
		leaves.reserve(m_leaves);
		b3FetchLeaves(this, m_root, leaves);
		m_root = b3TopDown(this, &leaves[0], leaves.size(), bu_treshold);
	}
}

//
void b3DynamicBvh::optimizeIncremental(int passes)
{
	if (passes < 0) passes = m_leaves;
	if (m_root && (passes > 0))
	{
		do
		{
			b3DbvtNode* node = m_root;
			unsigned bit = 0;
			while (node->isinternal())
			{
				node = b3Sort(node, m_root)->childs[(m_opath >> bit) & 1];
				bit = (bit + 1) & (sizeof(unsigned) * 8 - 1);
			}
			update(node);
			++m_opath;
		} while (--passes);
	}
}

//
b3DbvtNode* b3DynamicBvh::insert(const b3DbvtVolume& volume, void* data)
{
	b3DbvtNode* leaf = b3CreateNode(this, 0, volume, data);
	b3InsertLeaf(this, m_root, leaf);
	++m_leaves;
	return (leaf);
}

//
void b3DynamicBvh::update(b3DbvtNode* leaf, int lookahead)
{
	b3DbvtNode* root = b3RemoveLeaf(this, leaf);
	if (root)
	{
		if (lookahead >= 0)
		{
			for (int i = 0; (i < lookahead) && root->parent; ++i)
			{
				root = root->parent;
			}
		}
		else
			root = m_root;
	}
	b3InsertLeaf(this, root, leaf);
}

//
void b3DynamicBvh::update(b3DbvtNode* leaf, b3DbvtVolume& volume)
{
	b3DbvtNode* root = b3RemoveLeaf(this, leaf);
	if (root)
	{
		if (m_lkhd >= 0)
		{
			for (int i = 0; (i < m_lkhd) && root->parent; ++i)
			{
				root = root->parent;
			}
		}
		else
			root = m_root;
	}
	leaf->volume = volume;
	b3InsertLeaf(this, root, leaf);
}

//
bool b3DynamicBvh::update(b3DbvtNode* leaf, b3DbvtVolume& volume, const b3Vector3& velocity, b3Scalar margin)
{
	if (leaf->volume.Contain(volume)) return (false);
	volume.Expand(b3MakeVector3(margin, margin, margin));
	volume.SignedExpand(velocity);
	update(leaf, volume);
	return (true);
}

//
bool b3DynamicBvh::update(b3DbvtNode* leaf, b3DbvtVolume& volume, const b3Vector3& velocity)
{
	if (leaf->volume.Contain(volume)) return (false);
	volume.SignedExpand(velocity);
	update(leaf, volume);
	return (true);
}

//
bool b3DynamicBvh::update(b3DbvtNode* leaf, b3DbvtVolume& volume, b3Scalar margin)
{
	if (leaf->volume.Contain(volume)) return (false);
	volume.Expand(b3MakeVector3(margin, margin, margin));
	update(leaf, volume);
	return (true);
}

//
void b3DynamicBvh::remove(b3DbvtNode* leaf)
{
	b3RemoveLeaf(this, leaf);
	b3DeleteNode(this, leaf);
	--m_leaves;
}

//
void b3DynamicBvh::write(IWriter* iwriter) const
{
	b3DbvtNodeEnumerator nodes;
	nodes.nodes.reserve(m_leaves * 2);
	enumNodes(m_root, nodes);
	iwriter->Prepare(m_root, nodes.nodes.size());
	for (int i = 0; i < nodes.nodes.size(); ++i)
	{
		const b3DbvtNode* n = nodes.nodes[i];
		int p = -1;
		if (n->parent) p = nodes.nodes.findLinearSearch(n->parent);
		if (n->isinternal())
		{
			const int c0 = nodes.nodes.findLinearSearch(n->childs[0]);
			const int c1 = nodes.nodes.findLinearSearch(n->childs[1]);
			iwriter->WriteNode(n, i, p, c0, c1);
		}
		else
		{
			iwriter->WriteLeaf(n, i, p);
		}
	}
}

//
void b3DynamicBvh::clone(b3DynamicBvh& dest, IClone* iclone) const
{
	dest.clear();
	if (m_root != 0)
	{
		b3AlignedObjectArray<sStkCLN> stack;
		stack.reserve(m_leaves);
		stack.push_back(sStkCLN(m_root, 0));
		do
		{
			const int i = stack.size() - 1;
			const sStkCLN e = stack[i];
			b3DbvtNode* n = b3CreateNode(&dest, e.parent, e.node->volume, e.node->data);
			stack.pop_back();
			if (e.parent != 0)
				e.parent->childs[i & 1] = n;
			else
				dest.m_root = n;
			if (e.node->isinternal())
			{
				stack.push_back(sStkCLN(e.node->childs[0], n));
				stack.push_back(sStkCLN(e.node->childs[1], n));
			}
			else
			{
				iclone->CloneLeaf(n);
			}
		} while (stack.size() > 0);
	}
}

//
int b3DynamicBvh::maxdepth(const b3DbvtNode* node)
{
	int depth = 0;
	if (node) b3GetMaxDepth(node, 1, depth);
	return (depth);
}

//
int b3DynamicBvh::countLeaves(const b3DbvtNode* node)
{
	if (node->isinternal())
		return (countLeaves(node->childs[0]) + countLeaves(node->childs[1]));
	else
		return (1);
}

//
void b3DynamicBvh::extractLeaves(const b3DbvtNode* node, b3AlignedObjectArray<const b3DbvtNode*>& leaves)
{
	if (node->isinternal())
	{
		extractLeaves(node->childs[0], leaves);
		extractLeaves(node->childs[1], leaves);
	}
	else
	{
		leaves.push_back(node);
	}
}
