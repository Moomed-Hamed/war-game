#include "terrain.h"

#define LOG_PHYS(msg) out("[phys] " << msg)
#define ERR_PHYS(msg) LOG_PHYS("ERROR: " << msg)

#define GRAVITY -9.80665f
#define MAX_COLLIDERS 16 // max colliders of a single type

#define Vec3 btVector3 // temporary

typedef btCollisionObject RigidBody;

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

struct Cube_Collider
{
	vec3 position, scale;
	mat3 transform;
	RigidBody* rigidbody;
};

struct Cone_Collider
{
	vec3 position;
	mat3 transform;
	float height, radius;
	RigidBody* rigidbody;
};

struct Sphere_Collider
{
	vec3 position;
	mat3 transform;
	float radius;
	RigidBody* rigidbody;
};

struct Cylinder_Collider
{
	vec3 position;
	mat3 transform;
	float radius, height;
	RigidBody* rigidbody;
};

struct Capsule_Collider
{
	vec3 position, scale;
	mat3 transform;
	float height, radius; // height = total height
	RigidBody* rigidbody;
};

struct Physics_Vehicle
{
	// wheels : BL, BR, FR, FL // TODO : correct this
	btRigidBody* wheels[4];
	btHinge2Constraint* wheel_hinges[4];

	//doors : BL, BR, FR, FL
	btRigidBody* doors[4];
	btHingeConstraint* door_hinges[4];
};

//void add_phys_vehicle()

struct Ragdoll
{
	btRigidBody* bodies[BODYPART_COUNT];
	btTypedConstraint* joints[JOINT_COUNT];
};

struct Bullet_Objects
{
	btDiscreteDynamicsWorld* world;

	uint num_cubes, num_cones, num_spheres, num_capsules, num_cylinders;
	uint num_chassis;

	btRigidBody* chassis   [MAX_COLLIDERS];
	btRigidBody* cubes     [MAX_COLLIDERS];
	btRigidBody* cones     [MAX_COLLIDERS];
	btRigidBody* spheres   [MAX_COLLIDERS];
	btRigidBody* capsules  [MAX_COLLIDERS];
	btRigidBody* cylinders [MAX_COLLIDERS];

	// vehicle
	btRigidBody* wheels[4];
	btHinge2Constraint* wheel_hinges[4];

	btRigidBody* terrain;

	// ragdolls
	Ragdoll doll;
};

void init(Bullet_Objects* b)
{
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	b->world = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	b->world->setGravity(btVector3(0, GRAVITY, 0));
}

void add_phys_cube(Bullet_Objects* b, vec3 position, vec3 dimensions, float mass = 1)
{
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
	b->cubes[b->num_cubes] = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->cubes[b->num_cubes]);
	b->num_cubes += 1;
}
void add_phys_cone(Bullet_Objects* b, vec3 position, float radius, float height, float mass = 1)
{
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
	b->cones[b->num_cones] = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->cones[b->num_cones]);
	b->num_cones += 1;
}
void add_phys_sphere(Bullet_Objects* b, vec3 position, float radius, float mass = 1)
{
	btCollisionShape* shape = new btSphereShape(radius);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	b->spheres[b->num_spheres] = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->spheres[b->num_spheres]);
	b->num_spheres += 1;
}
void add_phys_capsule(Bullet_Objects* b, vec3 position, float radius, float height, float mass = 1)
{
	btCollisionShape* shape = new btCapsuleShape(radius, height);

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	b->capsules[b->num_capsules] = new btRigidBody(rbInfo);

	b->capsules[b->num_capsules]->setAngularFactor(0); // disable rotation
	b->world->addRigidBody(b->capsules[b->num_capsules]);
	b->num_capsules += 1;
}
void add_phys_cylinder(Bullet_Objects* b, vec3 position, float radius, float height, float mass = 1)
{
	btCollisionShape* shape = new btCylinderShape(btVector3(radius, height / 2.f, radius));

	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(position.x, position.y, position.z));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	b->cylinders[b->num_cylinders] = new btRigidBody(rbInfo);

	b->world->addRigidBody(b->cylinders[b->num_cylinders]);
	b->num_cylinders += 1;

	//btGeneric6DofSpring2Constraint::setDamping(1, (btScalar)1, true);
}
void add_phys_terrain(Bullet_Objects* b, Heightmap* map)
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

struct Physics_Colliders
{
	Cube_Collider     cubes     [MAX_COLLIDERS]; //  doubled because chassis are cubes
	Cone_Collider     cones     [MAX_COLLIDERS];
	Sphere_Collider   spheres   [MAX_COLLIDERS];
	Capsule_Collider  capsules  [MAX_COLLIDERS];
	Cylinder_Collider cylinders [MAX_COLLIDERS];
};

void get_location(btRigidBody* obj, vec3* pos = NULL, mat3* rot = NULL, vec3* scale = NULL)
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
void check_colliders(Bullet_Objects* b)
{
	uint num_objects = b->world->getNumCollisionObjects();

	print("[phys] %d : ", num_objects);
	for (uint i = 0; i < num_objects; i++)
	{
		// collision object is parent class of rigidbody; can be rigid, soft, or kinematic
		btCollisionObject* obj = b->world->getCollisionObjectArray()[i];

		switch (obj->getCollisionShape()->getShapeType())
		{
		case 0 : print("cu-"); break; // cube
		case 11: print("co-"); break; // cone
		case 8 : print("sp-"); break; // sphere
		case 10: print("ca-"); break; // capsule
		case 13: print("cy-"); break; // cylinder
		case 24: print("te-"); break; // terrain
		case 31: b->chassis[b->num_chassis++] = btRigidBody::upcast(obj); print("ch-"); break; // chassis // what is this actually?
		default: print("new shape : %d-", obj->getCollisionShape()->getShapeType());
		}
	}
}

void update(Bullet_Objects* b, Physics_Colliders* colliders, float dtime)
{
	b->world->stepSimulation(dtime, 10);

	for (uint i = 0; i < b->num_chassis; i++)
	{
		get_location(b->chassis[i],
			&colliders->cubes[b->num_cubes + i].position,
			&colliders->cubes[b->num_cubes + i].transform);

		colliders->cubes[b->num_cubes + i].scale = vec3(1.f, 0.5f, 2.f) * 2.f;
	}

	for (uint i = 0; i < b->num_cubes; i++)
	{
		colliders->cubes[i].scale = vec3(1);
		get_location(b->cubes[i], &colliders->cubes[i].position, &colliders->cubes[i].transform);
	}

	for (uint i = 0; i < b->num_cones; i++)
	{
		colliders->cones[i].radius = .5;
		get_location(b->cones[i], &colliders->cones[i].position, &colliders->cones[i].transform);
	}

	for (uint i = 0; i < b->num_spheres; i++)
	{
		colliders->spheres[i].radius = .5;
		get_location(b->spheres[i], &colliders->spheres[i].position, &colliders->spheres[i].transform);
	}

	for (uint i = 0; i < b->num_capsules; i++)
	{
		colliders->capsules[i].radius = .5;
		colliders->capsules[i].height = 1;
		get_location(b->capsules[i], &colliders->capsules[i].position, &colliders->capsules[i].transform);
		colliders->capsules[i].scale = vec3(1);
	}

	for (uint i = 0; i < b->num_cylinders; i++)
	{
		colliders->cylinders[i].radius = .5;
		colliders->cylinders[i].height = 1;
		get_location(b->cylinders[i], &colliders->cylinders[i].position, &colliders->cylinders[i].transform);
	}

	// vehicle

	for (uint i = 0; i < 4; i++)
	{
		vec3 pos;
		mat3 rot;

		get_location(b->wheels[i], &pos, &rot);

		colliders->cylinders[i + b->num_cylinders].position = pos;
		colliders->cylinders[i + b->num_cylinders].transform = rotate(mat4(rot), PI / 2.f, vec3(0, 0, 1));
		colliders->cylinders[i + b->num_cylinders].radius = .5;
		colliders->cylinders[i + b->num_cylinders].height = .4;
	}

	// ragdoll

	for (uint i = 1; i < BODYPART_COUNT; i++)
	{
		// we could allow capsules that do not fit this formula, but we enforce
		// this rule because it applies in >99% cases and is much easier to render wireframes for
		// TODO : maybe fix this so we can draw every type of capsule; what would you use that for tho?
		float total_height = colliders->capsules[i].height + (2.f * colliders->capsules[i].radius);

		get_location(b->doll.bodies[i],
			&colliders->capsules[i].position,
			&colliders->capsules[i].transform,
			&colliders->capsules[i].scale);

		// maybe make this tied to how 'alive' someone is
		b->doll.bodies[i]->setAngularFactor(.1); // makes ragdolls look more realistic

		//b->doll.bodies[i]->getCollisionShape()->

		colliders->capsules[i].height = .4;
		colliders->capsules[i].radius = .2; // WARNING DIPSHIT YOU HARDCODED SOMETHING BEEP BEEP CHANGE THIS NOW
	}
}

btRigidBody* createRigidBody(btDynamicsWorld* world, float mass, const btTransform& startTransform, btCollisionShape* shape)
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
btRigidBody* localCreateRigidBody(btDynamicsWorld* world, float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f) shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	world->addRigidBody(body);
	return body;
}

void make_phys_vehicle(Bullet_Objects* b)
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
	const float wheel_mass   = 1.0f;
	car_chassis = localCreateRigidBody(b->world, chassis_mass, tr, compound);
	//car_chassis->setDamping(0.2, 0.2);

	btVector3 wheel_positions[4] = {
		start_offset + btVector3(-1., -0.25,  1.25),
		start_offset + btVector3( 1., -0.25,  1.25),
		start_offset + btVector3( 1., -0.25, -1.25),
		start_offset + btVector3(-1., -0.25, -1.25)
	};

	const float wheel_radius         = 0.5f;
	const float wheel_width          = 0.4f;
	const float wheel_friction       = 1110;
	const float suspension_stiffness = 40.f;
	const float suspension_damping   = 2.0f;

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
		b->wheels[i] = createRigidBody(b->world, wheel_mass, tr, wheel_shape);
		btRigidBody* child = b->wheels[i];
		child->setFriction(wheel_friction);
		child->setActivationState(DISABLE_DEACTIVATION);

		// hinge constraint
		btVector3 parent_axis (0.f, 1.f, 0.f);
		btVector3 child_axis  (1.f, 0.f, 0.f);
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
void update_vehicle(Bullet_Objects* b, Keyboard keys)
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
		for (int i = 0; i < 4; i++)
			hinge[i]->setTargetVelocity(5, -1);
	else if (keys.RIGHT.is_pressed)
		for (int i = 0; i < 4; i++)
			hinge[i]->setTargetVelocity(5, 1);
	else
		for (int i = 0; i < 4; i++)
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

void update_terrain(Bullet_Objects* b, Heightmap* map)
{
	b->world->removeRigidBody(b->terrain);
	delete b->terrain;
	add_phys_terrain(b, map);
}

// rendering

struct Collider_Drawable
{
	vec3 position;
	mat3 transform;
	vec3 scale;
};

struct Physics_Renderer
{
	uint num_cubes, num_cones, num_spheres, num_capsules, num_cylinders;

	Collider_Drawable cubes[MAX_COLLIDERS];
	Collider_Drawable cones[MAX_COLLIDERS];
	Collider_Drawable spheres[MAX_COLLIDERS];
	Collider_Drawable capsules[MAX_COLLIDERS];
	Collider_Drawable cylinders[MAX_COLLIDERS];

	Drawable_Mesh_UV cube_mesh, cone_mesh, sphere_mesh, capsule_mesh, cylinder_mesh;
	GLuint texture, material;
	Shader shader;
};

mat3 scale(mat3 m, vec3 v) // move this and use to scale wheels
{
	m[0][0] *= v.x;
	m[1][1] *= v.y;
	m[2][2] *= v.z;

	return m;
}

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
	renderer->texture  = load_texture("assets/textures/palette.bmp"  );
	renderer->material = load_texture("assets/textures/materials.bmp");
}
void update(Physics_Renderer* renderer, Physics_Colliders* colliders)
{
	renderer->num_cubes     = 0;
	renderer->num_cones     = 0;
	renderer->num_spheres   = 0;
	renderer->num_capsules  = 0;
	renderer->num_cylinders = 0;

	for (uint i = 0; i < MAX_COLLIDERS; i++)
	{
		if (colliders->cubes[i].scale.x > 0) // is there a better way to check?
		{
			renderer->num_cubes += 1;
			renderer->cubes[i].position  = colliders->cubes[i].position;
			renderer->cubes[i].transform = colliders->cubes[i].transform;
			renderer->cubes[i].scale = colliders->cubes[i].scale;
		}

		if (colliders->cones[i].radius > 0)
		{
			renderer->num_cones += 1;
			renderer->cones[i].position  = colliders->cones[i].position;
			renderer->cones[i].transform = colliders->cones[i].transform;
			renderer->cones[i].scale = vec3(1);
		}

		if (colliders->spheres[i].radius > 0)
		{
			renderer->num_spheres += 1;
			renderer->spheres[i].position  = colliders->spheres[i].position;
			renderer->spheres[i].transform = colliders->spheres[i].transform;
			renderer->spheres[i].scale = vec3(colliders->spheres[i].radius / .5);
		}

		if (colliders->capsules[i].radius > 0)
		{
			float width = colliders->capsules[i].radius / .5f;

			renderer->num_capsules += 1;
			renderer->capsules[i].position  = colliders->capsules[i].position;
			renderer->capsules[i].transform = colliders->capsules[i].transform;
			renderer->capsules[i].scale = vec3(width, colliders->capsules[i].height, width);
		}

		if (colliders->cylinders[i].radius > 0)
		{
			float width = colliders->cylinders[i].radius / .5f;

			renderer->num_cylinders += 1;
			renderer->cylinders[i].position  = colliders->cylinders[i].position;
			renderer->cylinders[i].transform = colliders->cylinders[i].transform;
			renderer->cylinders[i].scale = vec3(width, colliders->cylinders[i].height, width);
		}
	}

	update(renderer->cube_mesh, sizeof(Collider_Drawable) * renderer->num_cubes, (byte*)(&renderer->cubes));
	update(renderer->cone_mesh, sizeof(Collider_Drawable) * renderer->num_cones, (byte*)(&renderer->cones));
	update(renderer->sphere_mesh, sizeof(Collider_Drawable) * renderer->num_spheres, (byte*)(&renderer->spheres));
	update(renderer->capsule_mesh, sizeof(Collider_Drawable) * renderer->num_capsules, (byte*)(&renderer->capsules));
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
	draw(renderer->cube_mesh, renderer->num_cubes);
	draw(renderer->cone_mesh, renderer->num_cones);
	draw(renderer->sphere_mesh, renderer->num_spheres);
	draw(renderer->capsule_mesh, renderer->num_capsules);
	draw(renderer->cylinder_mesh, renderer->num_cylinders);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);
}

// ragdoll testing grounds

// define ragdol shapes
// setup rigid bodies
// setup constraints

void add_phys_ragdoll(Bullet_Objects* b, float scale, Vec3 position_offset)
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
	btRigidBody** bodies = b->doll.bodies;

	btTransform offset;
	offset.setIdentity();
	offset.setOrigin(position_offset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.f, 1.f, 0.f));
	bodies[BODYPART_PELVIS] = createRigidBody(b->world, 1.f, offset * transform, shapes[BODYPART_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0., 1.2, 0.));
	bodies[BODYPART_SPINE] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_SPINE]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0., 1.6, 0.));
	bodies[BODYPART_HEAD] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_HEAD]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.18, 0.65, 0.));
	bodies[BODYPART_LEFT_UPPER_LEG] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_LEFT_UPPER_LEG]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.18, 0.2, 0.));
	bodies[BODYPART_LEFT_LOWER_LEG] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_LEFT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.18, 0.65, 0.));
	bodies[BODYPART_RIGHT_UPPER_LEG] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_UPPER_LEG]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.18, 0.2, 0.));
	bodies[BODYPART_RIGHT_LOWER_LEG] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_LOWER_LEG]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.35, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, PI * .5f);
	bodies[BODYPART_LEFT_UPPER_ARM] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_LEFT_UPPER_ARM]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(-0.7, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, PI * .5f);
	bodies[BODYPART_LEFT_LOWER_ARM] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_LEFT_LOWER_ARM]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.35, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, -PI * .5f);
	bodies[BODYPART_RIGHT_UPPER_ARM] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_UPPER_ARM]);
	
	transform.setIdentity();
	transform.setOrigin(scale* Vec3(0.7, 1.45, 0.));
	transform.getBasis().setEulerZYX(0, 0, -PI * .5f);
	bodies[BODYPART_RIGHT_LOWER_ARM] = createRigidBody(b->world, 1., offset * transform, shapes[BODYPART_RIGHT_LOWER_ARM]);
	
	// Setup some damping on the bodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		bodies[i]->setDamping(0.05f, 0.85f);
		bodies[i]->setDeactivationTime(0.8f);
		bodies[i]->setSleepingThresholds(1.6f, 2.5f);
	}

	// Now setup the constraints

	btTypedConstraint** joints = b->doll.joints;

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

// rendering
struct Doll_Drawable
{
	vec3 position, color;
	float scale;
};

struct Ragdoll_Renderer
{
	Doll_Drawable dolls[BODYPART_COUNT];

	Drawable_Mesh mesh;
	Shader shader;
};

void init(Ragdoll_Renderer* renderer)
{
	load(&renderer->mesh, "assets/meshes/basic/sphere.mesh", BODYPART_COUNT * sizeof(Point_Light));
	load(&renderer->shader, "assets/shaders/transform/light.vert", "assets/shaders/mesh.frag");

	mesh_add_attrib_vec3 (2, sizeof(Point_Light), 0 * sizeof(vec3)); // position
	mesh_add_attrib_mat3 (3, sizeof(Point_Light), 1 * sizeof(vec3)); // color
	mesh_add_attrib_float(4, sizeof(Point_Light), 2 * sizeof(vec3)); // radiance
}
void update(Ragdoll_Renderer* renderer, Ragdoll* doll)
{
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

	for (uint i = 0; i < BODYPART_COUNT; i++)
	{
		vec3 pos;
		mat3 rot;
		get_location(doll->bodies[i], &pos, &rot);
		renderer->dolls[i] = { pos, vec3(1,0,0), .2 };
	}

	renderer->dolls[0].color = { 1,1,1 };

	renderer->dolls[1].color = { 0,1,0 };
	renderer->dolls[3].color = { 1,0,0 };
	renderer->dolls[2].color = { 1,1,0 };

	renderer->dolls[8].color = { 0,1,1 };
	renderer->dolls[10].color = { 0,1,1 };

	update(renderer->mesh, sizeof(Doll_Drawable) * BODYPART_COUNT, (byte*)(renderer->dolls));
}
void draw(Ragdoll_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh, BODYPART_COUNT);
}