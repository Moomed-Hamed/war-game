#include "drawer.h"

/* ------------------------- *
	-       Physics         -
 * ------------------------- */

// bullet world. one body array, one handle convention : a plain uint index,
// 0 = none (bodies[0] stays reserved, the same rule as mesh ids). the index
// also goes into bullet with setUserIndex, so raycasts later come back as a
// phys_id with no guessing. transforms are read straight from bullet on
// demand, never mirrored into a cache.

const uint MAX_BODIES = 64;

// feel : props grip the floor and give a little when they land
const float PHYS_FRICTION    = .7f;
const float PHYS_RESTITUTION = .2f;
const float GROUND_FRICTION  = .9f;

enum { PHYS_BOX = 1, PHYS_SPHERE }; // shape tags stored per body (0 = none)

struct Phys_Body
{
	btRigidBody* body;
	uint shape;
	vec3 dims; // box half extents / sphere radius (world units)
};

struct Physics
{
	// world guts : allocated in init(), lives for the process (engine style)
	btDefaultCollisionConfiguration* config;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* broadphase;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* world;

	Phys_Body bodies[MAX_BODIES]; // [0] reserved : phys_id 0 = none
	uint num_bodies;

	void init()
	{
		num_bodies = 1;

		config     = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(config);
		broadphase = new btDbvtBroadphase();
		solver     = new btSequentialImpulseConstraintSolver();
		world      = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
		world->setGravity(btVector3(0, -9.80665f, 0));

		console->add_entry("Init Physics", SUCCESS, PHYS);
	}

	// one body in the world. mass 0 = static. returns phys_id (0 = failed, error already logged)
	uint add_body(uint shape, vec3 pos, vec3 dims, float mass)
	{
		if (num_bodies >= MAX_BODIES)
		{
			console->add_entry("Physics body slots full (MAX_BODIES)", FIXME, PHYS);
			return 0;
		}

		btCollisionShape* collision_shape = NULL;
		switch (shape)
		{
			case PHYS_BOX:    collision_shape = new btBoxShape(btVector3(dims.x, dims.y, dims.z)); break;
			case PHYS_SPHERE: collision_shape = new btSphereShape(dims.x); break;
		}
		if (!collision_shape)
		{
			console->add_entry("add_body : unknown shape", FIXME, PHYS);
			return 0;
		}

		btTransform start;
		start.setIdentity();
		start.setOrigin(btVector3(pos.x, pos.y, pos.z));

		btVector3 inertia(0, 0, 0);
		if (mass > 0) collision_shape->calculateLocalInertia(mass, inertia);

		btDefaultMotionState* motion = new btDefaultMotionState(start);
		btRigidBody::btRigidBodyConstructionInfo info(mass, motion, collision_shape, inertia);
		btRigidBody* body = new btRigidBody(info);

		body->setFriction(mass > 0 ? PHYS_FRICTION : GROUND_FRICTION);
		if (mass > 0) body->setRestitution(PHYS_RESTITUTION);

		body->setUserIndex(num_bodies); // handle rides inside bullet : raycasts read it back
		world->addRigidBody(body);

		bodies[num_bodies] = { body, shape, dims };
		return num_bodies++;
	}

	uint add_box(vec3 pos, vec3 half_extents, float mass)
	{
		return add_body(PHYS_BOX, pos, half_extents, mass);
	}

	uint add_sphere(vec3 pos, float radius, float mass)
	{
		return add_body(PHYS_SPHERE, pos, vec3(radius), mass);
	}

	// static ground slab, top face exactly at y 0 (sits under a 60 x 60 floor)
	uint add_ground(float half_size)
	{
		return add_body(PHYS_BOX, vec3(0, -1, 0), vec3(half_size, 1, half_size), 0);
	}

	// fixed 60hz internal step, up to 10 substeps to catch hitches
	void step(float dt)
	{
		if (dt > .05f) dt = .05f; // don't fast-forward through a stall
		world->stepSimulation(dt, 10, 1.f / 60.f);
	}

	// world pose straight out of bullet (motion state gives the interpolated
	// transform, so props stay smooth even on substep boundaries). static
	// bodies never move, so they have no pose to give : the authored one stays.
	bool get_transform(uint phys_id, vec3* pos, mat3* rot)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body)
			return false;
		if (bodies[phys_id].body->getMass() == 0)
			return false;

		btTransform transform;
		bodies[phys_id].body->getMotionState()->getWorldTransform(transform);

		float m[16];
		transform.getOpenGLMatrix(m); // column-major, gl convention

		if (pos) *pos = vec3(m[12], m[13], m[14]);
		if (rot) *rot = mat3(
			vec3(m[0], m[1], m[2]),
			vec3(m[4], m[5], m[6]),
			vec3(m[8], m[9], m[10]));
		return true;
	}
};
