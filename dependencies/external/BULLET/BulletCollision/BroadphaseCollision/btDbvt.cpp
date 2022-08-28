#include "btDbvt.h"

//
typedef btAlignedObjectArray<btDbvtNode*> tNodeArray;
typedef btAlignedObjectArray<const btDbvtNode*> tConstNodeArray;

//
struct btDbvtNodeEnumerator : btDbvt::ICollide
{
	tConstNodeArray nodes;
	void Process(const btDbvtNode* n) { nodes.push_back(n); }
};

//
static DBVT_INLINE int indexof(const btDbvtNode* node)
{
	return (node->parent->childs[1] == node);
}

//
static DBVT_INLINE btDbvtVolume merge(const btDbvtVolume& a, const btDbvtVolume& b)
{
	btDbvtVolume res;
	Merge(a, b, res);
	return (res);
}

// volume+edge lengths
static DBVT_INLINE btScalar size(const btDbvtVolume& a)
{
	const btVector3 edges = a.Lengths();
	return (edges.x() * edges.y() * edges.z() + edges.x() + edges.y() + edges.z());
}

//
static void getmaxdepth(const btDbvtNode* node, int depth, int& maxdepth)
{
	if (node->isinternal())
	{
		getmaxdepth(node->childs[0], depth + 1, maxdepth);
		getmaxdepth(node->childs[1], depth + 1, maxdepth);
	}
	else
		maxdepth = btMax(maxdepth, depth);
}

//
static DBVT_INLINE void deletenode(btDbvt* pdbvt, btDbvtNode* node)
{
	btAlignedFree(pdbvt->m_free);
	pdbvt->m_free = node;
}

//
static void recursedeletenode(btDbvt* pdbvt, btDbvtNode* node)
{
	if (node == 0) return;
	if (!node->isleaf())
	{
		recursedeletenode(pdbvt, node->childs[0]);
		recursedeletenode(pdbvt, node->childs[1]);
	}
	if (node == pdbvt->m_root) pdbvt->m_root = 0;
	deletenode(pdbvt, node);
}

//
static DBVT_INLINE btDbvtNode* createnode(btDbvt* pdbvt, btDbvtNode* parent, void* data)
{
	btDbvtNode* node;
	if (pdbvt->m_free)
	{
		node = pdbvt->m_free;
		pdbvt->m_free = 0;
	}
	else
	{
		node = new (btAlignedAlloc(sizeof(btDbvtNode), 16)) btDbvtNode();
	}
	node->parent = parent;
	node->data = data;
	node->childs[1] = 0;
	return (node);
}

//
static DBVT_INLINE btDbvtNode* createnode(btDbvt* pdbvt, btDbvtNode* parent, const btDbvtVolume& volume, void* data)
{
	btDbvtNode* node = createnode(pdbvt, parent, data);
	node->volume = volume;
	return (node);
}

//
static DBVT_INLINE btDbvtNode* createnode(btDbvt* pdbvt, btDbvtNode* parent, const btDbvtVolume& volume0, const btDbvtVolume& volume1, void* data)
{
	btDbvtNode* node = createnode(pdbvt, parent, data);
	Merge(volume0, volume1, node->volume);
	return (node);
}

//
static void insertleaf(btDbvt* pdbvt, btDbvtNode* root, btDbvtNode* leaf)
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
				root = root->childs[Select(leaf->volume,
										   root->childs[0]->volume,
										   root->childs[1]->volume)];
			} while (!root->isleaf());
		}
		btDbvtNode* prev = root->parent;
		btDbvtNode* node = createnode(pdbvt, prev, leaf->volume, root->volume, 0);
		if (prev)
		{
			prev->childs[indexof(root)] = node;
			node->childs[0] = root;
			root->parent = node;
			node->childs[1] = leaf;
			leaf->parent = node;
			do
			{
				if (!prev->volume.Contain(node->volume))
					Merge(prev->childs[0]->volume, prev->childs[1]->volume, prev->volume);
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
static btDbvtNode* removeleaf(btDbvt* pdbvt,
							  btDbvtNode* leaf)
{
	if (leaf == pdbvt->m_root)
	{
		pdbvt->m_root = 0;
		return (0);
	}
	else
	{
		btDbvtNode* parent = leaf->parent;
		btDbvtNode* prev = parent->parent;
		btDbvtNode* sibling = parent->childs[1 - indexof(leaf)];
		if (prev)
		{
			prev->childs[indexof(parent)] = sibling;
			sibling->parent = prev;
			deletenode(pdbvt, parent);
			while (prev)
			{
				const btDbvtVolume pb = prev->volume;
				Merge(prev->childs[0]->volume, prev->childs[1]->volume, prev->volume);
				if (NotEqual(pb, prev->volume))
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
			deletenode(pdbvt, parent);
			return (pdbvt->m_root);
		}
	}
}

//
static void fetchleaves(btDbvt* pdbvt, btDbvtNode* root, tNodeArray& leaves, int depth = -1)
{
	if (root->isinternal() && depth)
	{
		fetchleaves(pdbvt, root->childs[0], leaves, depth - 1);
		fetchleaves(pdbvt, root->childs[1], leaves, depth - 1);
		deletenode(pdbvt, root);
	}
	else
	{
		leaves.push_back(root);
	}
}

//
static bool leftOfAxis(const btDbvtNode* node, const btVector3& org, const btVector3& axis)
{
	return btDot(axis, node->volume.Center() - org) <= 0;
}

// Partitions leaves such that leaves[0, n) are on the left of axis, and leaves[n, count) are on the right of axis. returns N.
static int split(btDbvtNode** leaves, int count, const btVector3& org, const btVector3& axis)
{
	int begin = 0;
	int end = count;
	for (;;)
	{
		while (begin != end && leftOfAxis(leaves[begin], org, axis))
		{
			++begin;
		}

		if (begin == end)
		{
			break;
		}

		while (begin != end && !leftOfAxis(leaves[end - 1], org, axis))
		{
			--end;
		}

		if (begin == end)
		{
			break;
		}

		// swap out of place nodes
		--end;
		btDbvtNode* temp = leaves[begin];
		leaves[begin] = leaves[end];
		leaves[end] = temp;
		++begin;
	}

	return begin;
}

//
static btDbvtVolume bounds(btDbvtNode** leaves, int count)
{
	btDbvtVolume volume = leaves[0]->volume;
	for (int i = 1, ni = count; i < ni; ++i)
	{
		Merge(volume, leaves[i]->volume, volume);
	}
	return (volume);
}

//
static void bottomup(btDbvt* pdbvt, btDbvtNode** leaves, int count)
{
	while (count > 1)
	{
		btScalar minsize = SIMD_INFINITY;
		int minidx[2] = {-1, -1};
		for (int i = 0; i < count; ++i)
		{
			for (int j = i + 1; j < count; ++j)
			{
				const btScalar sz = size(merge(leaves[i]->volume, leaves[j]->volume));
				if (sz < minsize)
				{
					minsize = sz;
					minidx[0] = i;
					minidx[1] = j;
				}
			}
		}
		btDbvtNode* n[] = {leaves[minidx[0]], leaves[minidx[1]]};
		btDbvtNode* p = createnode(pdbvt, 0, n[0]->volume, n[1]->volume, 0);
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
static btDbvtNode* topdown(btDbvt* pdbvt, btDbvtNode** leaves, int count, int bu_treshold)
{
	static const btVector3 axis[] = {btVector3(1, 0, 0), btVector3(0, 1, 0), btVector3(0, 0, 1)};
	btAssert(bu_treshold > 2);
	if (count > 1)
	{
		if (count > bu_treshold)
		{
			const btDbvtVolume vol = bounds(leaves, count);
			const btVector3 org = vol.Center();
			int partition;
			int bestaxis = -1;
			int bestmidp = count;
			int splitcount[3][2] = {{0, 0}, {0, 0}, {0, 0}};
			int i;
			for (i = 0; i < count; ++i)
			{
				const btVector3 x = leaves[i]->volume.Center() - org;
				for (int j = 0; j < 3; ++j)
				{
					++splitcount[j][btDot(x, axis[j]) > 0 ? 1 : 0];
				}
			}
			for (i = 0; i < 3; ++i)
			{
				if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0))
				{
					const int midp = (int)btFabs(btScalar(splitcount[i][0] - splitcount[i][1]));
					if (midp < bestmidp)
					{
						bestaxis = i;
						bestmidp = midp;
					}
				}
			}
			if (bestaxis >= 0)
			{
				partition = split(leaves, count, org, axis[bestaxis]);
				btAssert(partition != 0 && partition != count);
			}
			else
			{
				partition = count / 2 + 1;
			}
			btDbvtNode* node = createnode(pdbvt, 0, vol, 0);
			node->childs[0] = topdown(pdbvt, &leaves[0], partition, bu_treshold);
			node->childs[1] = topdown(pdbvt, &leaves[partition], count - partition, bu_treshold);
			node->childs[0]->parent = node;
			node->childs[1]->parent = node;
			return (node);
		}
		else
		{
			bottomup(pdbvt, leaves, count);
			return (leaves[0]);
		}
	}
	return (leaves[0]);
}

//
static DBVT_INLINE btDbvtNode* sort(btDbvtNode* n, btDbvtNode*& r)
{
	btDbvtNode* p = n->parent;
	btAssert(n->isinternal());
	if (p > n)
	{
		const int i = indexof(n);
		const int j = 1 - i;
		btDbvtNode* s = p->childs[j];
		btDbvtNode* q = p->parent;
		btAssert(n == p->childs[i]);
		if (q)
			q->childs[indexof(p)] = n;
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
		btSwap(p->volume, n->volume);
		return (p);
	}
	return (n);
}

//
// Api
//

//
btDbvt::btDbvt()
{
	m_root = 0;
	m_free = 0;
	m_lkhd = -1;
	m_leaves = 0;
	m_opath = 0;
}

//
btDbvt::~btDbvt()
{
	clear();
}

//
void btDbvt::clear()
{
	if (m_root)
		recursedeletenode(this, m_root);
	btAlignedFree(m_free);
	m_free = 0;
	m_lkhd = -1;
	m_stkStack.clear();
	m_opath = 0;
}

//
void btDbvt::optimizeBottomUp()
{
	if (m_root)
	{
		tNodeArray leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this, m_root, leaves);
		bottomup(this, &leaves[0], leaves.size());
		m_root = leaves[0];
	}
}

//
void btDbvt::optimizeTopDown(int bu_treshold)
{
	if (m_root)
	{
		tNodeArray leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this, m_root, leaves);
		m_root = topdown(this, &leaves[0], leaves.size(), bu_treshold);
	}
}

//
void btDbvt::optimizeIncremental(int passes)
{
	if (passes < 0) passes = m_leaves;
	if (m_root && (passes > 0))
	{
		do
		{
			btDbvtNode* node = m_root;
			unsigned bit = 0;
			while (node->isinternal())
			{
				node = sort(node, m_root)->childs[(m_opath >> bit) & 1];
				bit = (bit + 1) & (sizeof(unsigned) * 8 - 1);
			}
			update(node);
			++m_opath;
		} while (--passes);
	}
}

//
btDbvtNode* btDbvt::insert(const btDbvtVolume& volume, void* data)
{
	btDbvtNode* leaf = createnode(this, 0, volume, data);
	insertleaf(this, m_root, leaf);
	++m_leaves;
	return (leaf);
}

//
void btDbvt::update(btDbvtNode* leaf, int lookahead)
{
	btDbvtNode* root = removeleaf(this, leaf);
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
	insertleaf(this, root, leaf);
}

//
void btDbvt::update(btDbvtNode* leaf, btDbvtVolume& volume)
{
	btDbvtNode* root = removeleaf(this, leaf);
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
	insertleaf(this, root, leaf);
}

//
bool btDbvt::update(btDbvtNode* leaf, btDbvtVolume& volume, const btVector3& velocity, btScalar margin)
{
	if (leaf->volume.Contain(volume)) return (false);
	volume.Expand(btVector3(margin, margin, margin));
	volume.SignedExpand(velocity);
	update(leaf, volume);
	return (true);
}

//
bool btDbvt::update(btDbvtNode* leaf, btDbvtVolume& volume, const btVector3& velocity)
{
	if (leaf->volume.Contain(volume)) return (false);
	volume.SignedExpand(velocity);
	update(leaf, volume);
	return (true);
}

//
bool btDbvt::update(btDbvtNode* leaf, btDbvtVolume& volume, btScalar margin)
{
	if (leaf->volume.Contain(volume)) return (false);
	volume.Expand(btVector3(margin, margin, margin));
	update(leaf, volume);
	return (true);
}

//
void btDbvt::remove(btDbvtNode* leaf)
{
	removeleaf(this, leaf);
	deletenode(this, leaf);
	--m_leaves;
}

//
void btDbvt::write(IWriter* iwriter) const
{
	btDbvtNodeEnumerator nodes;
	nodes.nodes.reserve(m_leaves * 2);
	enumNodes(m_root, nodes);
	iwriter->Prepare(m_root, nodes.nodes.size());
	for (int i = 0; i < nodes.nodes.size(); ++i)
	{
		const btDbvtNode* n = nodes.nodes[i];
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
void btDbvt::clone(btDbvt& dest, IClone* iclone) const
{
	dest.clear();
	if (m_root != 0)
	{
		btAlignedObjectArray<sStkCLN> stack;
		stack.reserve(m_leaves);
		stack.push_back(sStkCLN(m_root, 0));
		do
		{
			const int i = stack.size() - 1;
			const sStkCLN e = stack[i];
			btDbvtNode* n = createnode(&dest, e.parent, e.node->volume, e.node->data);
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
int btDbvt::maxdepth(const btDbvtNode* node)
{
	int depth = 0;
	if (node) getmaxdepth(node, 1, depth);
	return (depth);
}

//
int btDbvt::countLeaves(const btDbvtNode* node)
{
	if (node->isinternal())
		return (countLeaves(node->childs[0]) + countLeaves(node->childs[1]));
	else
		return (1);
}

//
void btDbvt::extractLeaves(const btDbvtNode* node, btAlignedObjectArray<const btDbvtNode*>& leaves)
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