#include "terrain.h"

mat3 scale(mat3 m, vec3 v) // should this be here?
{
	m[0][0] *= v.x;
	m[1][1] *= v.y;
	m[2][2] *= v.z;

	return m;
}

#define LOG(msg) out("[log]" << msg)
#define ERR(msg) out("[err]" << msg)

#define GRAVITY -9.80665f
#define MAX_COLLIDERS 16 // max colliders of a single type

#define Vec3 btVector3 // temporary

void get_transform(btRigidBody* obj, vec3* pos = NULL, mat3* rot = NULL, vec3* scale = NULL)
{
	btTransform trans;
	obj->getMotionState()->getWorldTransform(trans);

	union // for decomposing OpenGL matrix
	{
		mat4 t;
		float f[16];
	} transform;

	trans.getOpenGLMatrix((float*)&transform);
	btVector3 s = obj->getCollisionShape()->getLocalScaling();

	if (pos) *pos = { transform.f[12],transform.f[13],transform.f[14] };
	if (rot) *rot = transform.t;
	if (scale) *scale = { s.getX(), s.getY() , s.getZ() }; // is this actually scale?
}
void print_phys(btDiscreteDynamicsWorld* world)
{
	uint num_objects = world->getNumCollisionObjects();

	print("[phys] %d : ", num_objects);
	for (uint i = 0; i < num_objects; i++)
	{
		// collision object is parent class of rigidbody; can be rigid, soft, or kinematic
		btCollisionObject* obj = world->getCollisionObjectArray()[i];

		switch (obj->getCollisionShape()->getShapeType())
		{
		case 0 : print("cu-"); break; // cube
		case 11: print("co-"); break; // cone
		case 8 : print("sp-"); break; // sphere
		case 10: print("ca-"); break; // capsule
		case 13: print("cy-"); break; // cylinder
		case 24: print("te-"); break; // terrain
		case 31: print("ch-"); break; // chassis // what is this actually?
		default: print("new shape : %d-", obj->getCollisionShape()->getShapeType());
		}
	}
}

enum {
	BODYPART_PELVIS = 0,
	BODYPART_SPINE,
	BODYPART_HEAD,
	BODYPART_LEFT_UPPER_LEG,
	BODYPART_LEFT_LOWER_LEG,
	BODYPART_RIGHT_UPPER_LEG,
	BODYPART_RIGHT_LOWER_LEG,
	BODYPART_LEFT_UPPER_ARM,
	BODYPART_LEFT_LOWER_ARM,
	BODYPART_RIGHT_UPPER_ARM,
	BODYPART_RIGHT_LOWER_ARM,
	BODYPART_COUNT
};
enum {
	JOINT_PELVIS_SPINE = 0,
	JOINT_SPINE_HEAD,
	JOINT_LEFT_HIP,
	JOINT_LEFT_KNEE,
	JOINT_RIGHT_HIP,
	JOINT_RIGHT_KNEE,
	JOINT_LEFT_SHOULDER,
	JOINT_LEFT_ELBOW,
	JOINT_RIGHT_SHOULDER,
	JOINT_RIGHT_ELBOW,
	JOINT_COUNT
};

struct Phys_Cube
{
	vec3 position, scale;
	mat3 rotation;
	btRigidBody* body;
};

struct Phys_Cone
{
	vec3 position, scale;
	mat3 rotation;
	float height, radius;
	btRigidBody* body;
};

struct Phys_Sphere
{
	vec3 position, scale;
	mat3 rotation;
	float radius;
	btRigidBody* body;
};

struct Phys_Cylinder
{
	vec3 position, scale;
	mat3 rotation;
	float radius, height;
	btRigidBody* body;
};

struct Phys_Capsule
{
	vec3 position, scale;
	mat3 rotation;
	float height, radius; // height = total height
	btRigidBody* body;
};

struct Phys_Vehicle
{
	// wheels : BL, BR, FR, FL // TODO : correct this
	btRigidBody* wheels[4];
	btHinge2Constraint* wheel_hinges[4];

	//doors : BL, BR, FR, FL
	btRigidBody* doors[4];
	btHingeConstraint* door_hinges[4];
};

struct Phys_Ragdoll
{
	btRigidBody* bodies[BODYPART_COUNT];
	btTypedConstraint* joints[JOINT_COUNT];
};

struct Physics
{
	btDiscreteDynamicsWorld* world;

	uint num_cubes, num_cones, num_spheres, num_capsules, num_cylinders;

	Phys_Cube     cubes     [MAX_COLLIDERS];
	Phys_Cone     cones     [MAX_COLLIDERS];
	Phys_Sphere   spheres   [MAX_COLLIDERS];
	Phys_Capsule  capsules  [MAX_COLLIDERS];
	Phys_Cylinder cylinders [MAX_COLLIDERS];

	// terrain
	btRigidBody* terrain;

	// vehicle
	uint num_vehicles;

	btRigidBody* chassis [MAX_COLLIDERS];
	btRigidBody* wheels  [4];
	btHinge2Constraint* wheel_hinges[4];

	// ragdolls
	Phys_Ragdoll ragdoll;
};

void init(Physics* p)
{
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	p->world = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	p->world->setGravity(btVector3(0, GRAVITY, 0));
}

uint add_phys_cube(Physics* p, vec3 position, vec3 dimensions, float mass = 1)
{
	uint i = p->num_cubes++;
	p->cubes[i].scale = dimensions;
	p->cubes[i].position = position;

	dimensions *= .5f; // bullet uses half extents
	btCollisionShape* ground_shape = new btBoxShape(btVector3(dimensions.x, dimensions.y, dimensions.z));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(Vec3(position.x, position.y, position.z));

	btVector3 local_inertia(0, 0, 0);
	// rigidbody is dynamic if mass is non zero, otherwise static
	if (mass != 0.f) ground_shape->calculateLocalInertia(mass, local_inertia);

	// motionstate : optional, provides interpolation capabilities & only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, ground_shape, local_inertia);
	p->cubes[i].body = new btRigidBody(rbInfo);

	p->world->addRigidBody(p->cubes[i].body);

	return i; // index of this cube in the phys array
}
void add_phys_cone(Physics* b, vec3 position, float radius, float height, float mass = 1)
{
	uint i = b->num_cones++;
	b->cones[i].height = height;
	b->cones[i].radius = radius;
	b->cones[i].position = position;

	btCollisionShape* ground_shape = new btConeShape(radius, height);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btVector3 local_inertia(0, 0, 0);
	// rigidbody is dynamic if and only if mass is non zero, otherwise static
	if (mass != 0.f) ground_shape->calculateLocalInertia(mass, local_inertia);

	// motionstate : optional, provides interpolation capabilities & only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, ground_shape, local_inertia);
	b->cones[i].body = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->cones[i].body);
}
void add_phys_sphere(Physics* b, vec3 position, float radius, float mass = 1)
{
	uint i = b->num_spheres++;
	b->spheres[i].radius = radius;
	b->spheres[i].position = position;

	btCollisionShape* shape = new btSphereShape(radius);

	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	b->spheres[i].body = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->spheres[i].body);
}
uint add_phys_capsule(Physics* b, vec3 position, float radius = .5, float height = 1, float mass = 1)
{
	uint i = b->num_capsules++;
	b->capsules[i].radius = radius;
	b->capsules[i].height = height + (2.f * radius); // total height

	btCollisionShape* shape = new btCapsuleShape(radius, height);

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	b->capsules[i].body = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->capsules[i].body);

	return i;
}
void add_phys_cylinder(Physics* b, vec3 position, float radius, float height, float mass = 1)
{
	uint i = b->num_cylinders++;
	b->cylinders[i].height = height;
	b->cylinders[i].radius = radius;
	b->cylinders[i].position = position;

	btCollisionShape* shape = new btCylinderShape(btVector3(radius, height / 2.f, radius));

	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	b->cylinders[i].body = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->cylinders[i].body);

	//btGeneric6DofSpring2Constraint::setDamping(1, (btScalar)1, true);
}
void add_phys_terrain(Physics* b, Heightmap* map)
{
	btCollisionShape* shape = new btHeightfieldTerrainShape(
		HEIGHTMAP_N, HEIGHTMAP_N, map->height, -1.f * HEIGHTMAP_S, 1.f * HEIGHTMAP_S, 1, false);

	// this varies with terrain resolution
	float scale = HEIGHTMAP_N / (float)HEIGHTMAP_L;
	shape->setLocalScaling(btVector3(1.f / scale, 1, 1.f / scale));

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(HEIGHTMAP_L / 2.f, 0, HEIGHTMAP_L / 2.f));

	btVector3 localInertia(0, 0, 0);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(0.f, myMotionState, shape, localInertia);
	b->terrain = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->terrain);
}

// vehicles
btRigidBody* make_phys_chassis(Physics* p, float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	p->chassis[0] = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	p->world->addRigidBody(p->chassis[0]);
	return p->chassis[0];
}
btRigidBody* make_phys_wheel(uint i, Physics* phys, float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btVector3 localInertia(0, 0, 0);

	if (mass != 0.f)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	phys->wheels[i] = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	phys->wheels[i]->setUserIndex(-1); // what is this?
	phys->world->addRigidBody(phys->wheels[i]);
	return phys->wheels[i];
}
void add_phys_vehicle(Physics* b)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -3, 0));

	btRigidBody* car_chassis = NULL;
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));

	btCompoundShape* compound = new btCompoundShape();
	btTransform localTrans; // effectively shifts the center of mass with respect to the chassis
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0, 1, 0));

	compound->addChildShape(localTrans, chassisShape);

	{ // localTrans effectively shifts the center of mass with respect to the chassis
		btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
		btTransform suppLocalTrans;
		suppLocalTrans.setIdentity();
		suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));
		compound->addChildShape(suppLocalTrans, suppShape);
	}

	const btVector3 start_offset(4, 5, 4);
	tr.setOrigin(start_offset);

	const float chassis_mass = 2.0f;
	const float wheel_mass = 1.0f;
	car_chassis = make_phys_chassis(b, chassis_mass, tr, compound);
	//car_chassis->setDamping(0.2, 0.2);

	btVector3 wheel_positions[4] = {
		start_offset + btVector3(-1., -0.25,  1.25),
		start_offset + btVector3(1., -0.25,  1.25),
		start_offset + btVector3(1., -0.25, -1.25),
		start_offset + btVector3(-1., -0.25, -1.25)
	};

	const float wheel_radius = 0.5f;
	const float wheel_width = 0.4f;
	const float wheel_friction = 1110;
	const float suspension_stiffness = 40.f;
	const float suspension_damping = 2.0f;

	btCollisionShape* wheel_shape = new btCylinderShapeX(btVector3(wheel_width, wheel_radius, wheel_radius));

	btRigidBody* parent = car_chassis; // parent of the wheels
	parent->setActivationState(DISABLE_DEACTIVATION);

	btHinge2Constraint** hinge = b->wheel_hinges;

	for (int i = 0; i < 4; i++)
	{
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(wheel_positions[i]);

		// wheel is a child of the chassis
		b->wheels[i] = make_phys_wheel(i, b, wheel_mass, tr, wheel_shape);
		btRigidBody* child = b->wheels[i];
		child->setFriction(wheel_friction);
		child->setActivationState(DISABLE_DEACTIVATION);

		// hinge constraint
		btVector3 parent_axis(0.f, 1.f, 0.f);
		btVector3 child_axis(1.f, 0.f, 0.f);
		btVector3 anchor = tr.getOrigin();
		hinge[i] = new btHinge2Constraint(*parent, *child, anchor, parent_axis, child_axis);

		b->world->addConstraint(hinge[i], true);

		//pHinge2->setUpperLimit( PI / 4.f);
		//pHinge2->setLowerLimit(-PI / 4.f);
		hinge[i]->setParam(BT_CONSTRAINT_CFM, .15f, 2); // constraint-force mixing
		hinge[i]->setParam(BT_CONSTRAINT_ERP, .35f, 2); // error-reduction parameter

		hinge[i]->setDamping(2, suspension_damping);
		hinge[i]->setStiffness(2, suspension_stiffness);

		// drive engine
		hinge[i]->enableMotor(3, true);
		hinge[i]->setMaxMotorForce(3, 1000);
		hinge[i]->setTargetVelocity(3, 0);

		// steering engine
		hinge[i]->enableMotor(5, true);
		hinge[i]->setMaxMotorForce(5, 1000);
		hinge[i]->setTargetVelocity(5, 0);
	}
}

// ragdolls
btRigidBody* create_bodypart(btDynamicsWorld* world, float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btVector3 localInertia(0, 0, 0);

	if (mass != 0.f)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	body->setUserIndex(-1);
	world->addRigidBody(body);
	return body;
}
void add_phys_ragdoll(Physics* b, float scale, Vec3 position_offset)
{
	btCollisionShape* shapes[11] = {};
	shapes[BODYPART_PELVIS]          = new btCapsuleShape(.15 * scale, .20 * scale);
	shapes[BODYPART_SPINE]           = new btCapsuleShape(.15 * scale, .28 * scale);
	shapes[BODYPART_HEAD]            = new btCapsuleShape(.10 * scale, .05 * scale);
	shapes[BODYPART_LEFT_UPPER_LEG]  = new btCapsuleShape(.07 * scale, .45 * scale);
	shapes[BODYPART_LEFT_LOWER_LEG]  = new btCapsuleShape(.05 * scale, .37 * scale);
	shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(.07 * scale, .45 * scale);
	shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(.05 * scale, .37 * scale);
	shapes[BODYPART_LEFT_UPPER_ARM]  = new btCapsuleShape(.05 * scale, .33 * scale);
	shapes[BODYPART_LEFT_LOWER_ARM]  = new btCapsuleShape(.04 * scale, .25 * scale);
	shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(.05 * scale, .33 * scale);
	shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(.04 * scale, .25 * scale);

	// Setup rigid bodies
	btRigidBody** bodies = b->ragdoll.bodies;

	btTransform offset;
	offset.setIdentity();
	offset.setOrigin(position_offset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.f, 1.f, 0.f));
	bodies[BODYPART_PELVIS] = create_bodypart(b->world, 1.f, offset * transform, shapes[BODYPART_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0., 1.2, 0.));
	bodies[BODYPART_SPINE] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_SPINE]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0., 1.6, 0.));
	bodies[BODYPART_HEAD] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_HEAD]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.18, 0.65, 0.));
	bodies[BODYPART_LEFT_UPPER_LEG] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_LEFT_UPPER_LEG]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.18, 0.2, 0.));
	bodies[BODYPART_LEFT_LOWER_LEG] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_LEFT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.18, 0.65, 0.));
	bodies[BODYPART_RIGHT_UPPER_LEG] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_UPPER_LEG]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.18, 0.2, 0.));
	bodies[BODYPART_RIGHT_LOWER_LEG] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_LOWER_LEG]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.35, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, PI * .5f);
	bodies[BODYPART_LEFT_UPPER_ARM] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_LEFT_UPPER_ARM]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.7, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, PI * .5f);
	bodies[BODYPART_LEFT_LOWER_ARM] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_LEFT_LOWER_ARM]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.35, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, -PI * .5f);
	bodies[BODYPART_RIGHT_UPPER_ARM] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_UPPER_ARM]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.7, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, -PI * .5f);
	bodies[BODYPART_RIGHT_LOWER_ARM] = create_bodypart(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_LOWER_ARM]);
	
	// Setup some damping on the bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		bodies[i]->setDamping(0.05f, 0.85f);
		bodies[i]->setDeactivationTime(0.8f);
		bodies[i]->setSleepingThresholds(1.6f, 2.5f);
	}

	// Now setup the constraints

	btTypedConstraint** joints = b->ragdoll.joints;

	btHingeConstraint* hingeC;
	btConeTwistConstraint* coneC;
	btTransform localA, localB;

	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, PI * .5f, 0);
	localA.setOrigin(scale * Vec3(0., 0.15, 0.));
	localB.getBasis().setEulerZYX(0, PI * .5f, 0);
	localB.setOrigin(scale * Vec3(0., -0.15, 0.));
	hingeC = new btHingeConstraint(*bodies[BODYPART_PELVIS], *bodies[BODYPART_SPINE], localA, localB);
	hingeC->setLimit(btScalar(-PI * .25f), btScalar(PI * .5f));
	joints[JOINT_PELVIS_SPINE] = hingeC;
	b->world->addConstraint(joints[JOINT_PELVIS_SPINE], true);

	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, PI * .5f);
	localA.setOrigin(scale * Vec3(0., 0.30, 0.));
	localB.getBasis().setEulerZYX(0, 0, PI * .5f);
	localB.setOrigin(scale * Vec3(0., -0.14, 0.));
	coneC = new btConeTwistConstraint(*bodies[BODYPART_SPINE], *bodies[BODYPART_HEAD], localA, localB);
	coneC->setLimit(PI * .25f, PI * .25f, PI * .5f);
	joints[JOINT_SPINE_HEAD] = coneC;
	b->world->addConstraint(joints[JOINT_SPINE_HEAD], true);

	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, -PI * .25f * 5);
	localA.setOrigin(scale * Vec3(-0.18, -0.10, 0.));
	localB.getBasis().setEulerZYX(0, 0, -PI * .25f * 5);
	localB.setOrigin(scale * Vec3(0., 0.225, 0.));
	coneC = new btConeTwistConstraint(*bodies[BODYPART_PELVIS], *bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
	coneC->setLimit(PI * .25f, PI * .25f, 0);
	joints[JOINT_LEFT_HIP] = coneC;
	b->world->addConstraint(joints[JOINT_LEFT_HIP], true);
	
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, PI * .5f, 0);
	localA.setOrigin(scale * Vec3(0., -0.225, 0.));
	localB.getBasis().setEulerZYX(0, PI * .5f, 0);
	localB.setOrigin(scale * Vec3(0., 0.185, 0.));
	hingeC = new btHingeConstraint(*bodies[BODYPART_LEFT_UPPER_LEG], *bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(PI * .5f));
	joints[JOINT_LEFT_KNEE] = hingeC;
	b->world->addConstraint(joints[JOINT_LEFT_KNEE], true);
	
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, PI * .25f);
	localA.setOrigin(scale * Vec3(0.18, -0.10, 0.));
	localB.getBasis().setEulerZYX(0, 0, PI * .25f);
	localB.setOrigin(scale * Vec3(0., 0.225, 0.));
	coneC = new btConeTwistConstraint(*bodies[BODYPART_PELVIS], *bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
	coneC->setLimit(PI * .25f, PI * .25f, 0);
	joints[JOINT_RIGHT_HIP] = coneC;
	b->world->addConstraint(joints[JOINT_RIGHT_HIP], true);
	
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, PI * .5f, 0);
	localA.setOrigin(scale * Vec3(0., -0.225, 0.));
	localB.getBasis().setEulerZYX(0, PI * .5f, 0);
	localB.setOrigin(scale * Vec3(0., 0.185, 0.));
	hingeC = new btHingeConstraint(*bodies[BODYPART_RIGHT_UPPER_LEG], *bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
	hingeC->setLimit(0, PI * .5f);
	joints[JOINT_RIGHT_KNEE] = hingeC;
	b->world->addConstraint(joints[JOINT_RIGHT_KNEE], true);

	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, PI);
	localA.setOrigin(scale * Vec3(-0.2, 0.15, 0.));
	localB.getBasis().setEulerZYX(0, 0, PI * .5f);
	localB.setOrigin(scale * Vec3(0., -0.18, 0.));
	coneC = new btConeTwistConstraint(*bodies[BODYPART_SPINE], *bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
	coneC->setLimit(PI * .5f, PI * .5f, 0);
	joints[JOINT_LEFT_SHOULDER] = coneC;
	b->world->addConstraint(joints[JOINT_LEFT_SHOULDER], true);
	
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, PI * .5f, 0);
	localA.setOrigin(scale * Vec3(0., 0.18, 0.));
	localB.getBasis().setEulerZYX(0, PI * .5f, 0);
	localB.setOrigin(scale * Vec3(0., -0.14, 0.));
	hingeC = new btHingeConstraint(*bodies[BODYPART_LEFT_UPPER_ARM], *bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
	hingeC->setLimit(-PI * .5f, 0);
	joints[JOINT_LEFT_ELBOW] = hingeC;
	b->world->addConstraint(joints[JOINT_LEFT_ELBOW], true);
	
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, 0);
	localA.setOrigin(scale * Vec3(0.2, 0.15, 0.));
	localB.getBasis().setEulerZYX(0, 0, PI * .5f);
	localB.setOrigin(scale * Vec3(0., -0.18, 0.));
	coneC = new btConeTwistConstraint(*bodies[BODYPART_SPINE], *bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
	coneC->setLimit(PI * .5f, PI * .5f, 0);
	joints[JOINT_RIGHT_SHOULDER] = coneC;
	b->world->addConstraint(joints[JOINT_RIGHT_SHOULDER], true);
	
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, PI * .5f, 0);
	localA.setOrigin(scale * Vec3(0., 0.18, 0.));
	localB.getBasis().setEulerZYX(0, PI * .5f, 0);
	localB.setOrigin(scale * Vec3(0., -0.14, 0.));
	hingeC = new btHingeConstraint(*bodies[BODYPART_RIGHT_UPPER_ARM], *bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
	hingeC->setLimit(btScalar(-PI * .5f), btScalar(0));
	joints[JOINT_RIGHT_ELBOW] = hingeC;
	b->world->addConstraint(joints[JOINT_RIGHT_ELBOW], true);
}

// [physics updates]

void update_phys_vehicle(Physics* b, Keyboard keys)
{
	btHinge2Constraint** hinge = b->wheel_hinges;

	if (keys.UP.is_pressed)
		for (int i = 0; i < 4; i++)
			hinge[i]->setTargetVelocity(3, -4);
	else if (keys.DOWN.is_pressed)
		for (int i = 0; i < 4; i++)
			hinge[i]->setTargetVelocity(3, 4);
	else
		for (int i = 0; i < 4; i++)
			hinge[i]->setTargetVelocity(3, 0);

	// left
	if (keys.LEFT.is_pressed)
		for (int i = 0; i < 2; i++)
			hinge[i]->setTargetVelocity(5, -1);
	else if (keys.RIGHT.is_pressed)
		for (int i = 0; i < 2; i++)
			hinge[i]->setTargetVelocity(5, 1);
	else
		for (int i = 0; i < 2; i++)
			hinge[i]->setTargetVelocity(5, 0);

	static float EngineForce = 0.f;

	static float defaultBreakingForce = 10.f;
	static float BreakingForce = 100.f;

	static float maxEngineForce = 1000.f;  // should be engine/velocity dependent
	static float maxBreakingForce = 100.f;

	static float VehicleSteering = 0.f;
	static float steeringIncrement = 0.04f;
	static float steeringClamp = 0.3f;

	if (keys.UP.is_pressed)
	{
		EngineForce = maxEngineForce;
		BreakingForce = 0.f;
	}

	if (keys.DOWN.is_pressed)
	{
		EngineForce = -maxEngineForce;
		BreakingForce = 0.f;
	}

	if (keys.LEFT.is_pressed)
	{
		VehicleSteering += steeringIncrement;
		VehicleSteering = (VehicleSteering > steeringClamp) ? steeringClamp : VehicleSteering;
	}

	if (keys.RIGHT.is_pressed)
	{
		VehicleSteering -= steeringIncrement;
		VehicleSteering = (VehicleSteering < -steeringClamp) ? -steeringClamp : VehicleSteering;
	}

	if (false) // reset
	{
		VehicleSteering = 0.f;
		BreakingForce = defaultBreakingForce;
		EngineForce = 0.f;
	}
}
void update_phys_terrain(Physics* b, Heightmap* map)
{
	b->world->removeRigidBody(b->terrain);
	delete b->terrain;
	add_phys_terrain(b, map);
}

void update(Physics* phys, float dtime, Keyboard keys)
{
	phys->world->stepSimulation(dtime, 10);

	update_phys_vehicle(phys, keys);

	for (uint i = 0; i < phys->num_cubes; i++)
		get_transform(phys->cubes[i].body,
			&phys->cubes[i].position,
			&phys->cubes[i].rotation,
			&phys->cubes[i].scale);

	for (uint i = 0; i < phys->num_cones; i++)
		get_transform(phys->cones[i].body,
			&phys->cones[i].position,
			&phys->cones[i].rotation,
			&phys->cones[i].scale);

	for (uint i = 0; i < phys->num_spheres; i++)
		get_transform(phys->spheres[i].body,
			&phys->spheres[i].position,
			&phys->spheres[i].rotation,
			&phys->spheres[i].scale);

	for (uint i = 0; i < phys->num_cylinders; i++)
		get_transform(phys->cylinders[i].body,
			&phys->cylinders[i].position,
			&phys->cylinders[i].rotation,
			&phys->cylinders[i].scale);

	for (uint i = 0; i < phys->num_capsules; i++)
		get_transform(phys->capsules[i].body,
			&phys->capsules[i].position,
			&phys->capsules[i].rotation,
			&phys->capsules[i].scale);

	// vehicles

	for (uint i = 0; i < phys->num_vehicles; i++)
	{
		get_transform(phys->chassis[i],
			&phys->cubes[phys->num_cubes + i].position,
			&phys->cubes[phys->num_cubes + i].rotation);

		phys->cubes[phys->num_cubes + i].scale = vec3(1.f, 0.5f, 2.f) * 2.f;

		for (uint i = 0; i < 4; i++)
		{
			vec3 pos;
			mat3 rot;

			get_transform(phys->wheels[i], &pos, &rot);

			phys->cylinders[i + phys->num_cylinders].position = pos;
			phys->cylinders[i + phys->num_cylinders].rotation = rotate(mat4(rot), PI / 2.f, vec3(0, 0, 1));
			phys->cylinders[i + phys->num_cylinders].radius = .5;
			phys->cylinders[i + phys->num_cylinders].height = .4;
		}
	}

	// ragdolls

	for (uint i = 1; i < BODYPART_COUNT; i++) // WARNING : fix this
	{
		// we could allow capsules that do not fit this formula, but we enforce
		// this rule because it applies in >99% cases and is much easier to render wireframes for
		// TODO : maybe fix this so we can draw every type of capsule; what would you use that for tho?
		float total_height = phys->capsules[i].height + (2.f * phys->capsules[i].radius);

		get_transform(phys->ragdoll.bodies[i],
			&phys->capsules[i].position,
			&phys->capsules[i].rotation,
			&phys->capsules[i].scale);

		// maybe make this tied to how 'alive' someone is
		phys->ragdoll.bodies[i]->setAngularFactor(.1); // makes ragdolls look more realistic

		//phys->ragdoll.bodies[i]->getCollisionShape()->

		phys->capsules[i].height = .4;
		phys->capsules[i].radius = .2; // WARNING DIPSHIT YOU HARDCODED SOMETHING BEEP BEEP CHANGE THIS NOW
	}
}

// rendering

struct Collider_Drawable
{
	vec3 position;
	mat3 rotation;
	vec3 scale;
};

struct Physics_Renderer
{
	uint num_cubes, num_cones, num_spheres, num_capsules, num_cylinders;

	Collider_Drawable cubes     [MAX_COLLIDERS]; // also stores chassis; fix it
	Collider_Drawable cones     [MAX_COLLIDERS];
	Collider_Drawable spheres   [MAX_COLLIDERS];
	Collider_Drawable capsules  [MAX_COLLIDERS];
	Collider_Drawable cylinders [MAX_COLLIDERS]; // also stores wheels; fix it

	Drawable_Mesh_UV cube_mesh, cone_mesh, sphere_mesh, capsule_mesh, cylinder_mesh;
	GLuint texture, material;
	Shader shader;
};

void init(Physics_Renderer* renderer)
{
	uint reserved_size = sizeof(Collider_Drawable) * MAX_COLLIDERS;

	load(&renderer->cube_mesh, "assets/meshes/basic/cube.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform
	mesh_add_attrib_vec3(7, sizeof(Collider_Drawable), sizeof(vec3) + sizeof(mat3)); // scale

	load(&renderer->cone_mesh, "assets/meshes/basic/cone.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform
	mesh_add_attrib_vec3(7, sizeof(Collider_Drawable), sizeof(vec3) + sizeof(mat3)); // scale

	load(&renderer->sphere_mesh, "assets/meshes/basic/sphere.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform
	mesh_add_attrib_vec3(7, sizeof(Collider_Drawable), sizeof(vec3) + sizeof(mat3)); // scale

	load(&renderer->capsule_mesh, "assets/meshes/basic/capsule.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform
	mesh_add_attrib_vec3(7, sizeof(Collider_Drawable), sizeof(vec3) + sizeof(mat3)); // scale

	load(&renderer->cylinder_mesh, "assets/meshes/basic/cylinder.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform
	mesh_add_attrib_vec3(7, sizeof(Collider_Drawable), sizeof(vec3) + sizeof(mat3)); // scale

	load(&renderer->shader, "assets/shaders/transform/wireframe.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture = load_texture("assets/textures/palette.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");
}
void update(Physics_Renderer* renderer, Physics* phys)
{
	renderer->num_cubes     = 0;
	renderer->num_cones     = 0;
	renderer->num_spheres   = 0;
	renderer->num_capsules  = 0;
	renderer->num_cylinders = 0;

	for (uint i = 0; i < MAX_COLLIDERS; i++)
	{
		if (phys->cubes[i].body) // is there a better way to check?
		{
			renderer->num_cubes++;
			renderer->cubes[i].position = phys->cubes[i].position;
			renderer->cubes[i].rotation = phys->cubes[i].rotation;
			renderer->cubes[i].scale    = phys->cubes[i].scale;
		}

		if (phys->cones[i].body)
		{
			renderer->num_cones += 1;
			renderer->cones[i].position = phys->cones[i].position;
			renderer->cones[i].rotation = phys->cones[i].rotation;
			renderer->cones[i].scale    = vec3(1);
		}

		if (phys->spheres[i].body)
		{
			renderer->num_spheres += 1;
			renderer->spheres[i].position = phys->spheres[i].position;
			renderer->spheres[i].rotation = phys->spheres[i].rotation;
			renderer->spheres[i].scale    = vec3(phys->spheres[i].radius / .5);
		}

		if (phys->capsules[i].body)
		{
			float width = phys->capsules[i].radius / .5f;

			renderer->num_capsules += 1;
			renderer->capsules[i].position = phys->capsules[i].position;
			renderer->capsules[i].rotation = phys->capsules[i].rotation;
			renderer->capsules[i].scale    = vec3(1);
		}

		if (phys->cylinders[i].body)
		{
			float width = phys->cylinders[i].radius / .5f;

			renderer->num_cylinders += 1;
			renderer->cylinders[i].position = phys->cylinders[i].position;
			renderer->cylinders[i].rotation = phys->cylinders[i].rotation;
			renderer->cylinders[i].scale    = vec3(width, phys->cylinders[i].height, width);
		}
	}

	// vehicle
	uint k = phys->num_cubes;
	if (phys->chassis[0])
	{
		vec3 pos;
		mat3 rot;

		get_transform(phys->chassis[0], &pos, &rot);

		vec3 scale = vec3(1.f, 0.5f, 2.f) * 2.f;

		renderer->num_cubes++;
		renderer->cubes[k].position = pos;
		renderer->cubes[k].rotation = rot;
		renderer->cubes[k].scale = scale;
	}

	k = phys->num_cylinders;
	for (uint i = 0; i < 4; i++) // wheels
	{
		if (phys->wheels[i])
		{
			float radius = .5;
			float height = .4;
			float width = radius / .5f;

			vec3 pos;
			mat3 rot;
			get_transform(phys->wheels[i], &pos, &rot);

			renderer->num_cylinders++;
			renderer->cylinders[k + i].position = pos;
			renderer->cylinders[k + i].rotation = rotate(mat4(rot), PI / 2.f, vec3(0, 0, 1));
			renderer->cylinders[k + i].scale = vec3(width, height, width);
		}
	}

	// ragdoll
	k = phys->num_capsules;
	for (uint i = 0; i < BODYPART_COUNT; i++) // wheels
	{
		if (phys->ragdoll.bodies[i])
		{
			float radius = .2;
			float height = .4;
			float width = radius / .5f;

			vec3 pos;
			mat3 rot;
			get_transform(phys->ragdoll.bodies[i], &pos, &rot);

			renderer->num_capsules++;
			renderer->capsules[k + i].position = pos;
			renderer->capsules[k + i].rotation = rot;
			renderer->capsules[k + i].scale = vec3(width, height, width);

			// maybe make this tied to how 'alive' someone is
			phys->ragdoll.bodies[i]->setAngularFactor(.1); // makes ragdolls look more realistic

			//b->doll.bodies[i]->getCollisionShape()->
		}
	}

	update(renderer->cube_mesh    , sizeof(Collider_Drawable) * renderer->num_cubes    , (byte*)(&renderer->cubes));
	update(renderer->cone_mesh    , sizeof(Collider_Drawable) * renderer->num_cones    , (byte*)(&renderer->cones));
	update(renderer->sphere_mesh  , sizeof(Collider_Drawable) * renderer->num_spheres  , (byte*)(&renderer->spheres));
	update(renderer->capsule_mesh , sizeof(Collider_Drawable) * renderer->num_capsules , (byte*)(&renderer->capsules));
	update(renderer->cylinder_mesh, sizeof(Collider_Drawable) * renderer->num_cylinders, (byte*)(&renderer->cylinders));
}
void draw(Physics_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);

	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, renderer->texture);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, renderer->material);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_CULL_FACE);
	draw(renderer->cube_mesh    , renderer->num_cubes);
	draw(renderer->cone_mesh    , renderer->num_cones);
	draw(renderer->sphere_mesh  , renderer->num_spheres);
	draw(renderer->capsule_mesh , renderer->num_capsules);
	draw(renderer->cylinder_mesh, renderer->num_cylinders);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);
}