#ifndef B3_OVERLAPPING_PAIR_H
#define B3_OVERLAPPING_PAIR_H

#include "Bullet3Common/shared/b3Int4.h"

#define B3_NEW_PAIR_MARKER -1
#define B3_REMOVED_PAIR_MARKER -2

typedef b3Int4 b3BroadphasePair;

inline b3Int4 b3MakeBroadphasePair(int xx, int yy)
{
	b3Int4 pair;

	if (xx < yy)
	{
		pair.x = xx;
		pair.y = yy;
	}
	else
	{
		pair.x = yy;
		pair.y = xx;
	}
	pair.z = B3_NEW_PAIR_MARKER;
	pair.w = B3_NEW_PAIR_MARKER;
	return pair;
}

/*struct b3BroadphasePair : public b3Int4
{
	explicit b3BroadphasePair(){}
	
};
*/

class b3BroadphasePairSortPredicate
{
public:
	bool operator()(const b3BroadphasePair& a, const b3BroadphasePair& b) const
	{
		const int uidA0 = a.x;
		const int uidB0 = b.x;
		const int uidA1 = a.y;
		const int uidB1 = b.y;
		return uidA0 > uidB0 || (uidA0 == uidB0 && uidA1 > uidB1);
	}
};

B3_FORCE_INLINE bool operator==(const b3BroadphasePair& a, const b3BroadphasePair& b)
{
	return (a.x == b.x) && (a.y == b.y);
}

#endif  //B3_OVERLAPPING_PAIR_H
