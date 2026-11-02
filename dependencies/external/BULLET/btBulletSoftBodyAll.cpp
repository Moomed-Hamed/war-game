// classic Bullet soft-body build (patches, ropes, soft-rigid collision).
// the deformable-body fork is intentionally not included; the game uses the
// stable btSoftBody path.
#include "BulletSoftBody/btSoftBody.cpp"
#include "BulletSoftBody/btSoftBodyHelpers.cpp"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp"
#include "BulletSoftBody/btDefaultSoftBodySolver.cpp"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.cpp"
#include "BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp"
#include "BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp"
#include "BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp"
