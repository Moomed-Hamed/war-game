#ifndef OVERLAPPING_PAIR_CALLBACK_H
#define OVERLAPPING_PAIR_CALLBACK_H

class btDispatcher;
struct btBroadphasePair;

///The btOverlappingPairCallback class is an additional optional broadphase user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
class btOverlappingPairCallback
{
protected:
	btOverlappingPairCallback() {}

public:
	virtual ~btOverlappingPairCallback()
	{
	}

	virtual btBroadphasePair* addOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) = 0;

	virtual void* removeOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, btDispatcher* dispatcher) = 0;

	virtual void removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy0, btDispatcher* dispatcher) = 0;
};

#endif  //OVERLAPPING_PAIR_CALLBACK_H
