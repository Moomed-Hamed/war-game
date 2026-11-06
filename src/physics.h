#include "drawer.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"

/* ------------------------- *
	-       Physics         -
 * ------------------------- */

// bullet world. one body array, uint handles (0 = none, bodies[0] reserved,
// same rule as mesh ids) ; setUserIndex carries the handle into raycasts.
// transforms read straight from bullet, never mirrored into a cache.

const uint MAX_BODIES = 32768;

// feel : props grip the floor and give a little when they land
const float PHYS_FRICTION    = .7f;
const float PHYS_RESTITUTION = .2f;
const float GROUND_FRICTION  = .1f;

enum { PHYS_BOX = 1, PHYS_SPHERE, PHYS_CAPSULE, PHYS_CYLINDER, PHYS_CONE }; // shape tags stored per body (0 = none)

struct Phys_Body
{
	btRigidBody* body;
	uint shape;
	vec3 dims; // box half extents / sphere radius (world units)
	float mass; // 0 = static
	btQuaternion spawn_rot; // pose it was born in : reset restores it
	vec3 spawn_pos;         // spot it was born in : reset uses it too
	uint wire_mesh; // 0 = shared unit wire, else a per-body capsule wire
};

// debug wires over the scene : collider shapes, constraint frames and a scene
// ray. own shader, own vao - the renderer never hears about it
struct Phys_Debug
{
	bool enabled;     // collider wires
	bool constraints; // joint anchors, pivots and axes
	bool rays;        // the scene's interaction ray and its hit point
	uint next_wire_mesh; // next id for a per-body capsule wire (shared overlay)

	// the interaction ray is filled by a scene each frame
	bool ray_active, ray_hit, ray_target;
	vec3 ray_a, ray_b, ray_point;


	void init() { next_wire_mesh = 20; } // ids 1..9 belong to the shared overlay
	void shutdown() {}

	uint append_capsule_wire(float radius, float height)
	{
		uint slices = 16, cap_segs = 3;
		uint rings = (cap_segs + 1) * 2; // top cap + bottom cap, equators included

		Mesh_Data wire = {};
		wire.num_vertices = rings * (slices + 1);
		wire.num_indices  = (rings - 1) * slices * 8; // 4 edges per quad
		wire.positions = Alloc(vec3, wire.num_vertices);
		wire.normals   = Alloc(vec3, wire.num_vertices);
		wire.uvs       = Alloc(vec2, wire.num_vertices);
		wire.indices   = Alloc(uint, wire.num_indices);

		float half = height * 0.5f;
		uint v = 0;
		for (uint ring = 0; ring < rings; ring++)
		{
			// profile latitude + the cap it swings around, rings top to bottom
			bool top_cap = ring <= cap_segs;
			uint k = top_cap ? ring : ring - cap_segs - 1;
			float phi = (PI * 0.5f) * (k / (float)cap_segs) + (top_cap ? 0.f : PI * 0.5f);
			float y = (top_cap ? half : -half) + cosf(phi) * radius;

			for (uint j = 0; j <= slices; j++)
			{
				float theta = 2.f * PI * j / slices;
				wire.positions[v++] = vec3(sinf(phi) * cosf(theta) * radius, y,
				                           sinf(phi) * sinf(theta) * radius);
			}
		}

		// ring + column edges, same structure as the ball. the band columns come
		// out straight, so the cylinder section of the wire is exact too
		uint e = 0;
		for (uint ring = 0; ring < rings - 1; ring++)
			for (uint j = 0; j < slices; j++)
			{
				uint upper = ring * (slices + 1) + j;
				uint lower = upper + slices + 1;

				wire.indices[e++] = upper;     wire.indices[e++] = upper + 1; // ring edge
				wire.indices[e++] = lower;     wire.indices[e++] = lower + 1; // ring edge
				wire.indices[e++] = upper;     wire.indices[e++] = lower;     // column edge
				wire.indices[e++] = upper + 1; wire.indices[e++] = lower + 1; // column edge
			}

		uint mesh_id = next_wire_mesh++;
		debug_draw.lines.append_geometry(mesh_id, wire);
		wire.release();
		return mesh_id;
	}
};

// joints the scene built. bullet owns the constraints themselves, this is the
// list the ux reads : limits are radians and mean different things per kind
enum { PHYS_HINGE = 1, PHYS_CONETWIST, PHYS_SLIDER, PHYS_FIXED };
const uint MAX_JOINTS = 512;

struct Phys_Joint
{
	uint kind;    // PHYS_HINGE / PHYS_CONETWIST / PHYS_SLIDER / PHYS_FIXED
	uint a, b;    // phys_ids the joint links
	float lo, hi; // hinge / slider : (lo, hi) | cone twist : (swing, twist)
};

// one local frame for a ragdoll joint : euler (x, y, z) plus origin, scaled
btTransform joint_frame(float s, vec3 euler, vec3 at)
{
	btTransform f;
	f.setIdentity();
	f.getBasis().setEulerZYX(euler.x, euler.y, euler.z);
	f.setOrigin(btVector3(at.x * s, at.y * s, at.z * s));
	return f;
}

struct Physics
{
	// world guts : allocated in init(), lives for the process (engine style)
	btSoftBodyRigidBodyCollisionConfiguration* config;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* broadphase;
	btSequentialImpulseConstraintSolver* solver;
	btDefaultSoftBodySolver* soft_solver;
	btSoftRigidDynamicsWorld* world;

	Phys_Body bodies[MAX_BODIES]; // [0] reserved : phys_id 0 = none
	uint num_bodies;
	Phys_Debug debug; // collider, constraint and ray wires over the scene
	Phys_Joint joints[MAX_JOINTS]; // tracked joints, read by the ux tab
	uint num_joints;

	void init()
	{
		num_bodies = 1;
		num_joints = 0;

		config      = new btSoftBodyRigidBodyCollisionConfiguration();
		dispatcher  = new btCollisionDispatcher(config);
		broadphase  = new btDbvtBroadphase();
		solver      = new btSequentialImpulseConstraintSolver();
		soft_solver = new btDefaultSoftBodySolver();
		world       = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver,
		                                          config, soft_solver);
		world->setGravity(btVector3(0, -9.80665f, 0));
		world->getWorldInfo().m_gravity = btVector3(0, -9.80665f, 0);
		world->getWorldInfo().air_density = 1.2f;
		world->getWorldInfo().water_density = 0.f;
		world->getWorldInfo().m_sparsesdf.Initialize();

		debug.init();

		console->add_entry("Init Physics", SUCCESS, PHYS);
	}

	// tear the world down : constraints, then bodies with the shapes and motion
	// states they own, then the world and its helpers. safe before init()
	void shutdown()
	{
		if (!world) return;

		// soft bodies first : they can hold contact references to rigid bodies
		btSoftBodyArray& soft = world->getSoftBodyArray();
		for (int i = soft.size() - 1; i >= 0; i--)
		{
			btSoftBody* sb = soft[i];
			world->removeSoftBody(sb);
			delete sb;
		}

		for (int i = world->getNumConstraints() - 1; i >= 0; i--)
		{
			btTypedConstraint* c = world->getConstraint(i);
			world->removeConstraint(c);
			delete c;
		}

		for (uint i = 0; i < num_bodies; i++)
		{
			btRigidBody* b = bodies[i].body;
			if (!b) continue;

			world->removeRigidBody(b);
			delete b->getMotionState();
			delete b->getCollisionShape();
			delete b;
		}

		debug.shutdown();

		delete world;       world = NULL;
		delete soft_solver; soft_solver = NULL;
		delete solver;      solver = NULL;
		delete broadphase;  broadphase = NULL;
		delete dispatcher;  dispatcher = NULL;
		delete config;      config = NULL;

		memset(bodies, 0, sizeof(bodies));
		memset(joints, 0, sizeof(joints));
		num_bodies = 1;
		num_joints = 0;
	}

	// one body in the world. mass 0 = static. tilt = spawn rotation, radians about
	// each axis. returns phys_id (0 = failed, error already logged)
	uint add_body(uint shape, vec3 pos, vec3 dims, float mass, vec3 tilt = vec3(0))
	{
		// recycle slots freed by streaming : the high-water count only grows when
		// there is no hole left
		uint slot = 0;
		for (uint i = 1; i < num_bodies; i++)
			if (!bodies[i].body) { slot = i; break; }

		if (!slot)
		{
			if (num_bodies >= MAX_BODIES)
			{
				console->add_entry("Physics body slots full (MAX_BODIES)", FIXME, PHYS);
				return 0;
			}
			slot = num_bodies++;
		}

		btCollisionShape* collision_shape = NULL;
		switch (shape)
		{
			case PHYS_BOX:    collision_shape = new btBoxShape(btVector3(dims.x, dims.y, dims.z)); break;
			case PHYS_SPHERE: collision_shape = new btSphereShape(dims.x); break;
			// dims = (radius, cylinder height) : the same convention as the mesh factory
			case PHYS_CAPSULE:  collision_shape = new btCapsuleShape(dims.x, dims.y); break;
			// dims = half extents (r, half height, r), y-aligned like the mesh factory
			case PHYS_CYLINDER: collision_shape = new btCylinderShape(btVector3(dims.x, dims.y, dims.z)); break;
			// dims = (radius, full height, radius), y-aligned like the mesh factory
			case PHYS_CONE:     collision_shape = new btConeShape(dims.x, dims.y); break;
		}
		if (!collision_shape)
		{
			console->add_entry("add_body : unknown shape", FIXME, PHYS);
			return 0;
		}

		btTransform start;
		start.setIdentity();
		start.setOrigin(btVector3(pos.x, pos.y, pos.z));
		if (tilt.x || tilt.y || tilt.z)
		{
			btQuaternion qx(btVector3(1, 0, 0), tilt.x);
			btQuaternion qy(btVector3(0, 1, 0), tilt.y);
			btQuaternion qz(btVector3(0, 0, 1), tilt.z);
			start.setRotation(qz * qy * qx);
		}

		btVector3 inertia(0, 0, 0);
		if (mass > 0) collision_shape->calculateLocalInertia(mass, inertia);

		btDefaultMotionState* motion = new btDefaultMotionState(start);
		btRigidBody::btRigidBodyConstructionInfo info(mass, motion, collision_shape, inertia);
		btRigidBody* body = new btRigidBody(info);

		body->setFriction(mass > 0 ? PHYS_FRICTION : GROUND_FRICTION);
		if (mass > 0) body->setRestitution(PHYS_RESTITUTION);

		body->setUserIndex(slot); // handle rides inside bullet : raycasts read it back
		world->addRigidBody(body);

		bodies[slot] = { body, shape, dims, mass };
		bodies[slot].spawn_rot = start.getRotation();
		bodies[slot].spawn_pos = pos;
		bodies[slot].wire_mesh = (shape == PHYS_CAPSULE)
			? debug.append_capsule_wire(dims.x, dims.y) : 0; // 0 = shared unit shape
		return slot;
	}

	uint add_box(vec3 pos, vec3 half_extents, float mass, vec3 tilt = vec3(0))
	{
		return add_body(PHYS_BOX, pos, half_extents, mass, tilt);
	}

	uint add_sphere(vec3 pos, float radius, float mass)
	{
		return add_body(PHYS_SPHERE, pos, vec3(radius), mass);
	}

	// continuous collision on a fast body : the sweep catches thin geometry
	void enable_ccd(uint phys_id)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return;

		btRigidBody* b = bodies[phys_id].body;
		b->setCcdMotionThreshold(1e-4f);
		b->setCcdSweptSphereRadius(bodies[phys_id].dims.x * .9f);
	}

	// capsule : radius + the cylinder part only, same convention as the mesh factory
	uint add_capsule(vec3 pos, float radius, float height, float mass, vec3 tilt = vec3(0))
	{
		return add_body(PHYS_CAPSULE, pos, vec3(radius, height, 0), mass, tilt);
	}

	// cylinder : y-aligned, half extents like the box (r, half height, r)
	uint add_cylinder(vec3 pos, vec3 half_extents, float mass, vec3 tilt = vec3(0))
	{
		return add_body(PHYS_CYLINDER, pos, half_extents, mass, tilt);
	}

	// cone : y-aligned, base down (radius, full height)
	uint add_cone(vec3 pos, float radius, float height, float mass, vec3 tilt = vec3(0))
	{
		return add_body(PHYS_CONE, pos, vec3(radius, height, radius), mass, tilt);
	}

	// static ground slab, top face at `top` (sits under a 60 x 60 floor)
	uint add_ground(float half_size, float top = 0.f)
	{
		return add_body(PHYS_BOX, vec3(0, top - 1.f, 0), vec3(half_size, 1, half_size), 0);
	}

	// ragdoll : an 11-capsule figure with 10 joints. every part is a regular
	// body, so the ux table, exact wires and wake_all all come free
	uint add_ragdoll(vec3 pos, float scale)
	{
		if (num_bodies + 11 > MAX_BODIES)
		{
			console->add_entry("Not enough room for a ragdoll", FIXME, PHYS);
			return 0;
		}

		// part sizes + spawn offsets (feet at the origin, y up). the head sits
		// at 1.6 and carries mass : a massless head pulls the joints apart
		struct { float r, h; vec3 at; float roll; } parts[11] = {
			{ .15f, .20f, vec3(0, 1.f, 0),       0         }, // pelvis
			{ .15f, .28f, vec3(0, 1.2f, 0),      0         }, // spine
			{ .10f, .05f, vec3(0, 1.6f, 0),      0         }, // head
			{ .07f, .45f, vec3(-.18f, .65f, 0),  0         }, // left  upper leg
			{ .05f, .37f, vec3(-.18f, .2f, 0),   0         }, // left  lower leg
			{ .07f, .45f, vec3(.18f, .65f, 0),   0         }, // right upper leg
			{ .05f, .37f, vec3(.18f, .2f, 0),    0         }, // right lower leg
			{ .05f, .33f, vec3(-.35f, 1.45f, 0), PI * .5f  }, // left  upper arm
			{ .04f, .25f, vec3(-.7f, 1.45f, 0),  PI * .5f  }, // left  lower arm
			{ .05f, .33f, vec3(.35f, 1.45f, 0),  -PI * .5f }, // right upper arm
			{ .04f, .25f, vec3(.7f, 1.45f, 0),   -PI * .5f }, // right lower arm
		};

		uint start = num_bodies;
		for (uint i = 0; i < 11; i++)
			add_capsule(pos + parts[i].at * scale, parts[i].r * scale, parts[i].h * scale,
			            1.f, vec3(0, 0, parts[i].roll));

		for (uint i = 0; i < 11; i++) // limp, heavy-limbed feel
		{
			btRigidBody* b = bodies[start + i].body;
			b->setDamping(.05f, .85f);
			b->setDeactivationTime(.8f);
			b->setSleepingThresholds(1.6f, 2.5f);
			b->setRestitution(0.f);
			b->setAngularFactor(.1f); // slow, meaty rotation
		}

		btRigidBody& pelvis      = *bodies[start].body;
		btRigidBody& spine       = *bodies[start + 1].body;
		btRigidBody& head        = *bodies[start + 2].body;
		btRigidBody& l_upper_leg = *bodies[start + 3].body;
		btRigidBody& l_lower_leg = *bodies[start + 4].body;
		btRigidBody& r_upper_leg = *bodies[start + 5].body;
		btRigidBody& r_lower_leg = *bodies[start + 6].body;
		btRigidBody& l_upper_arm = *bodies[start + 7].body;
		btRigidBody& l_lower_arm = *bodies[start + 8].body;
		btRigidBody& r_upper_arm = *bodies[start + 9].body;
		btRigidBody& r_lower_arm = *bodies[start + 10].body;

		// the joints : hinges at waist/knees/elbows, cone twists at the rest
		add_hinge(pelvis, spine,
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, .15f, 0)),
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, -.15f, 0)), -PI * .25f, PI * .5f);
		add_cone(spine, head,
			joint_frame(scale, vec3(0, 0, PI * .5f), vec3(0, .30f, 0)),
			joint_frame(scale, vec3(0, 0, PI * .5f), vec3(0, -.14f, 0)), PI * .25f, PI * .5f);
		add_cone(pelvis, l_upper_leg,
			joint_frame(scale, vec3(0, 0, -PI * .25f * 5), vec3(-.18f, -.10f, 0)),
			joint_frame(scale, vec3(0, 0, -PI * .25f * 5), vec3(0, .225f, 0)), PI * .25f, 0);
		add_hinge(l_upper_leg, l_lower_leg,
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, -.225f, 0)),
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, .185f, 0)), 0, PI * .5f);
		add_cone(pelvis, r_upper_leg,
			joint_frame(scale, vec3(0, 0, PI * .25f), vec3(.18f, -.10f, 0)),
			joint_frame(scale, vec3(0, 0, PI * .25f), vec3(0, .225f, 0)), PI * .25f, 0);
		add_hinge(r_upper_leg, r_lower_leg,
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, -.225f, 0)),
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, .185f, 0)), 0, PI * .5f);
		add_cone(spine, l_upper_arm,
			joint_frame(scale, vec3(0, 0, PI), vec3(-.2f, .15f, 0)),
			joint_frame(scale, vec3(0, 0, PI * .5f), vec3(0, -.18f, 0)), PI * .5f, 0);
		add_hinge(l_upper_arm, l_lower_arm,
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, .18f, 0)),
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, -.14f, 0)), -PI * .5f, 0);
		add_cone(spine, r_upper_arm,
			joint_frame(scale, vec3(0, 0, 0), vec3(.2f, .15f, 0)),
			joint_frame(scale, vec3(0, 0, PI * .5f), vec3(0, -.18f, 0)), PI * .5f, 0);
		add_hinge(r_upper_arm, r_lower_arm,
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, .18f, 0)),
			joint_frame(scale, vec3(0, PI * .5f, 0), vec3(0, -.14f, 0)), -PI * .5f, 0);

		console->add_entry("Ragdoll spawned : 11 parts, 10 joints", SUCCESS, PHYS);
		return start;
	}

	// one limited hinge joint, straight into the world. a non-zero motor
	// impulse drives the joint toward motor_speed. the handle lets a scene
	// retune the motor or limits after the joint exists
	btHingeConstraint* add_hinge(btRigidBody& a, btRigidBody& b, btTransform fa, btTransform fb,
	                             float lo, float hi, float motor_speed = 0.f,
	                             float motor_impulse = 0.f, bool limited = true)
	{
		btHingeConstraint* j = new btHingeConstraint(a, b, fa, fb);
		if (limited) j->setLimit(lo, hi);
		if (motor_impulse > 0.f) j->enableAngularMotor(true, motor_speed, motor_impulse);
		world->addConstraint(j, true); // no self-collision across the joint
		add_joint(PHYS_HINGE, a, b, lo, hi);
		return j;
	}

	// one limited cone twist (swing, swing, twist)
	void add_cone(btRigidBody& a, btRigidBody& b, btTransform fa, btTransform fb, float swing, float twist)
	{
		btConeTwistConstraint* j = new btConeTwistConstraint(a, b, fa, fb);
		j->setLimit(swing, swing, twist);
		world->addConstraint(j, true);
		add_joint(PHYS_CONETWIST, a, b, swing, twist);
	}

	btSliderConstraint* add_slider(btRigidBody& a, btRigidBody& b,
	                               btTransform fa, btTransform fb,
	                               float lo, float hi)
	{
		btSliderConstraint* j = new btSliderConstraint(a, b, fa, fb, true);
		j->setLowerLinLimit(lo);
		j->setUpperLinLimit(hi);
		j->setLowerAngLimit(0.f);
		j->setUpperAngLimit(0.f);
		world->addConstraint(j, true);
		add_joint(PHYS_SLIDER, a, b, lo, hi);
		return j;
	}

	// real Bullet soft body patch. the caller pins nodes afterwards with
	// setMass(index, 0); the patch is already in the soft-rigid world.
	btSoftBody* add_soft_patch(btVector3 c00, btVector3 c10, btVector3 c01,
	                           btVector3 c11, int rx, int ry, float mass)
	{
		btSoftBody* sb = btSoftBodyHelpers::CreatePatch(world->getWorldInfo(),
		                                                c00, c10, c01, c11,
		                                                rx, ry, 0, true, .01f);
		if (!sb) return NULL;

		sb->setTotalMass(mass, true);
		sb->m_cfg.piterations = 8;
		sb->m_cfg.viterations = 3;
		sb->m_cfg.diterations = 4;
		sb->m_cfg.kDF = .22f;
		sb->m_cfg.kDP = .003f;
		sb->m_cfg.kLF = .05f;
		// CreatePatch has no clusters, so rigid collision must use the SDF path.
		// CL_RS only works for clustered bodies and silently ignores the sphere.
		sb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS |
		                       btSoftBody::fCollision::VF_SS;
		sb->generateBendingConstraints(2, sb->appendMaterial());
		sb->getCollisionShape()->setMargin(.04f);
		sb->getCollisionShape()->setUserPointer((void*)sb);
		world->addSoftBody(sb);
		return sb;
	}

	btFixedConstraint* add_fixed(btRigidBody& a, btRigidBody& b,
	                             btTransform fa, btTransform fb)
	{
		btFixedConstraint* j = new btFixedConstraint(a, b, fa, fb);
		world->addConstraint(j, true);
		add_joint(PHYS_FIXED, a, b, 0.f, 0.f);
		return j;
	}

	// remember a joint for the ux tab : every body carries its phys_id as user index
	void add_joint(uint kind, btRigidBody& a, btRigidBody& b, float lo, float hi)
	{
		if (num_joints >= MAX_JOINTS) return;
		joints[num_joints++] = { kind, (uint)a.getUserIndex(), (uint)b.getUserIndex(), lo, hi };
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

	// raw readouts for the ux body table (statics included)
	vec3 get_position(uint phys_id)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return vec3(0);
		btVector3 origin = bodies[phys_id].body->getWorldTransform().getOrigin();
		return vec3(origin.x(), origin.y(), origin.z());
	}

	vec3 get_velocity(uint phys_id)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return vec3(0);
		btVector3 v = bodies[phys_id].body->getLinearVelocity();
		return vec3(v.x(), v.y(), v.z());
	}

	// how many tracked joints touch this body (ux table column)
	uint joints_of(uint phys_id)
	{
		uint n = 0;
		for (uint i = 0; i < num_joints; i++)
			if (joints[i].a == phys_id || joints[i].b == phys_id) n++;
		return n;
	}

	bool is_awake(uint phys_id)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return false;
		return bodies[phys_id].body->isActive();
	}

	// dynamic bodies move, statics never do
	bool is_dynamic(uint phys_id)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return false;
		return bodies[phys_id].body->getMass() > 0;
	}

	// closest hit along a segment. body_id is the phys_id of what was hit
	// (0 = nothing) : bullets and the ground check read it
	struct Phys_Ray { bool hit; vec3 point, normal; uint body_id; };

	Phys_Ray raycast(vec3 from, vec3 to)
	{
		Phys_Ray r = {};

		btVector3 a(from.x, from.y, from.z), b(to.x, to.y, to.z);
		btCollisionWorld::ClosestRayResultCallback cb(a, b);
		world->rayTest(a, b, cb);
		if (!cb.hasHit()) return r;

		r.hit     = true;
		r.point   = vec3(cb.m_hitPointWorld.x(), cb.m_hitPointWorld.y(), cb.m_hitPointWorld.z());
		r.normal  = vec3(cb.m_hitNormalWorld.x(), cb.m_hitNormalWorld.y(), cb.m_hitNormalWorld.z());
		r.body_id = cb.m_collisionObject ? (uint)cb.m_collisionObject->getUserIndex() : 0;
		return r;
	}

	// one kick through a body's center of mass : statics ignore it
	void apply_impulse(uint phys_id, vec3 impulse)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return;

		btRigidBody* b = bodies[phys_id].body;
		if (b->getMass() == 0) return;

		b->activate(true);
		b->applyCentralImpulse(btVector3(impulse.x, impulse.y, impulse.z));
	}

	// nudge every dynamic body awake : watch the whole scene stir and re-settle
	void wake_all()
	{
		for (uint i = 1; i < num_bodies; i++)
			if (bodies[i].body && bodies[i].body->getMass() > 0)
				bodies[i].body->activate(true);
	}

	// teleport a body back to the pose it was born in : cleared, awake, ready
	void reset(uint phys_id)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return;

		btRigidBody* body = bodies[phys_id].body;

		btTransform start;
		start.setIdentity();
		start.setOrigin(btVector3(bodies[phys_id].spawn_pos.x, bodies[phys_id].spawn_pos.y,
			bodies[phys_id].spawn_pos.z));
		start.setRotation(bodies[phys_id].spawn_rot); // back to the pose it was born in

		body->setWorldTransform(start);
		body->setInterpolationWorldTransform(start);
		body->getMotionState()->setWorldTransform(start);
		body->clearForces();
		body->setLinearVelocity(btVector3(0, 0, 0));
		body->setAngularVelocity(btVector3(0, 0, 0));
		body->activate(true);
	}

	// every dynamic body back to its spawn pose (Phys tab -> reset all)
	void reset_all()
	{
		for (uint i = 1; i < num_bodies; i++)
			if (bodies[i].body && bodies[i].body->getMass() > 0)
				reset(i);
	}

	// append colliders, joint frames and the scene ray to the shared overlay
	void append_debug()
	{
		if (debug.enabled)
		{
			for (uint shape = PHYS_BOX; shape <= PHYS_CONE; shape++)
			{
				for (uint i = 1; i < num_bodies; i++)
				{
					if (!bodies[i].body || bodies[i].shape != shape) continue;

					float m[16];
					bodies[i].body->getWorldTransform().getOpenGLMatrix(m);
					mat4 model = mat4(
						vec4(m[0],  m[1],  m[2],  m[3]),
						vec4(m[4],  m[5],  m[6],  m[7]),
						vec4(m[8],  m[9],  m[10], m[11]),
						vec4(m[12], m[13], m[14], m[15]));

					// unit wires scale to fit ; capsule wires are pre-sized per body
					uint mesh;
					switch (shape)
					{
						case PHYS_SPHERE:   mesh = debug_draw.sphere_mesh;   break;
						case PHYS_CYLINDER: mesh = debug_draw.cylinder_mesh; break;
						case PHYS_CONE:     mesh = debug_draw.cone_mesh;     break;
						case PHYS_CAPSULE:  mesh = bodies[i].wire_mesh;      break;
						default:            mesh = debug_draw.box_mesh;      break;
					}

					if (shape != PHYS_CAPSULE) model = glm::scale(model, bodies[i].dims);

					// colour says what the body is doing : static / awake / asleep
					vec4 color = vec4(.52f, .64f, .76f, 1.f);
					if (bodies[i].mass > 0.f)
						color = is_awake(i) ? vec4(.20f, 1.f, .40f, 1.f)
						                    : vec4(.95f, .72f, .25f, 1.f);

					debug_draw.append_shape(mesh, model, color);
				}
			}
		}

		if (debug.constraints)
		{
			const vec4 constraint_color = vec4(1.f, .25f, .85f, 1.f);

			for (int i = 0; i < world->getNumConstraints(); i++)
			{
				btTypedConstraint* c = world->getConstraint(i);
				btTransform fa, fb;
				bool framed = false;

				if (c->getConstraintType() == HINGE_CONSTRAINT_TYPE)
				{
					btHingeConstraint* h = static_cast<btHingeConstraint*>(c);
					fa = h->getFrameOffsetA();
					fb = h->getFrameOffsetB();
					framed = true;
				}
				else if (c->getConstraintType() == CONETWIST_CONSTRAINT_TYPE)
				{
					btConeTwistConstraint* t = static_cast<btConeTwistConstraint*>(c);
					fa = t->getFrameOffsetA();
					fb = t->getFrameOffsetB();
					framed = true;
				}
				if (!framed) continue;

				btTransform ta = c->getRigidBodyA().getCenterOfMassTransform();
				btTransform tb = c->getRigidBodyB().getCenterOfMassTransform();

				btVector3 pa = ta * fa.getOrigin();
				btVector3 pb = tb * fb.getOrigin();
				vec3 va(pa.x(), pa.y(), pa.z());
				vec3 vb(pb.x(), pb.y(), pb.z());

				debug_draw.append_segment(va, vb, constraint_color);
				debug_draw.append_cross(va, .06f, constraint_color);
				debug_draw.append_cross(vb, .06f, constraint_color);

				// local z is the hinge / twist axis in both constraint types
				btVector3 axis = ta.getBasis() * fa.getBasis().getColumn(2);
				vec3 av(axis.x(), axis.y(), axis.z());
				if (length(av) > 1e-5f)
				{
					vec3 at = normalize(av);
					debug_draw.append_segment(va - at * .35f, va + at * .35f, constraint_color);
				}
			}
		}

		if (debug.rays && debug.ray_active)
		{
			vec4 color = vec4(1.f, .15f, .12f, 1.f); // miss
			if (debug.ray_target)   color = vec4(.20f, 1.f, .40f, 1.f);
			else if (debug.ray_hit) color = vec4(1.f, .65f, .20f, 1.f);

			debug_draw.append_segment(debug.ray_a, debug.ray_b, color);
			if (debug.ray_hit) debug_draw.append_cross(debug.ray_point, .07f, color);
		}
	}
};
