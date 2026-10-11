#include "drawer.h"

/* ------------------------- *
	-       Physics         -
 * ------------------------- */

// bullet world. one body array, uint handles (0 = none, bodies[0] reserved,
// same rule as mesh ids) ; setUserIndex carries the handle into raycasts.
// transforms read straight from bullet, never mirrored into a cache.

const uint MAX_BODIES = 64;

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

// collider wireframes : unit box/sphere/cylinder edges instanced at every
// body's transform, capsules get an exact per-body wire. own shader, own
// vao - the renderer never hears about it
struct Phys_Debug
{
	bool enabled;
	DrawBuffer lines;     // unit shapes + one instance slot per body
	ShaderProgram shader; // same wire shaders as the renderer's overlay
	uint box_mesh, sphere_mesh, cylinder_mesh, cone_mesh;
	uint unit_mesh[PHYS_CONE + 1]; // shape tag -> shared unit wire
	uint next_wire_mesh;  // next id for a per-body capsule wire

	void init()
	{
		shader.create("assets/shaders/wire.vert", "assets/shaders/wire.frag");
		lines.init(); // 1MB default : this buffer barely uses it

		// unit box : 8 corners, 12 edges (an edge joins corners differing by one bit)
		Mesh_Data box = {};
		box.num_vertices = 8;
		box.num_indices  = 24;
		box.positions = Alloc(vec3, 8);
		box.normals   = Alloc(vec3, 8);
		box.uvs       = Alloc(vec2, 8);
		box.indices   = Alloc(uint, 24);

		for (uint i = 0; i < 8; i++)
		{
			box.positions[i] = vec3((i & 1) ? 1.f : -1.f, (i & 2) ? 1.f : -1.f, (i & 4) ? 1.f : -1.f);
			box.normals[i]   = vec3(0, 1, 0);
		}

		uint e = 0;
		for (uint i = 0; i < 8; i++)
			for (uint bit = 1; bit < 8; bit *= 2)
			{
				uint j = i ^ bit;
				if (i < j) { box.indices[e++] = i; box.indices[e++] = j; }
			}

		box_mesh = 1;
		lines.append_geometry(box_mesh, box);
		box.release();

		// unit sphere : ring + column edges of a coarse uv sphere
		uint slices = 16, stacks = 12;
		Mesh_Data src = MeshFactory::generate_uv_sphere(1.f, slices, stacks);
		Mesh_Data ball = {};
		ball.num_vertices = src.num_vertices;
		ball.num_indices  = stacks * slices * 8; // 4 edges per quad
		ball.positions = Alloc(vec3, src.num_vertices);
		ball.normals   = Alloc(vec3, src.num_vertices);
		ball.uvs       = Alloc(vec2, src.num_vertices);
		ball.indices   = Alloc(uint, ball.num_indices);

		for (uint i = 0; i < src.num_vertices; i++)
		{
			ball.positions[i] = src.positions[i];
			ball.normals[i]   = src.normals[i];
			ball.uvs[i]       = src.uvs[i];
		}

		e = 0;
		for (uint i = 0; i < stacks; i++)
			for (uint j = 0; j < slices; j++)
			{
				uint upper = i * (slices + 1) + j;
				uint lower = upper + slices + 1;

				ball.indices[e++] = upper;     ball.indices[e++] = upper + 1; // ring edge
				ball.indices[e++] = lower;     ball.indices[e++] = lower + 1; // ring edge
				ball.indices[e++] = upper;     ball.indices[e++] = lower;     // column edge
				ball.indices[e++] = upper + 1; ball.indices[e++] = lower + 1; // column edge
			}

		src.release();
		sphere_mesh = 2;
		lines.append_geometry(sphere_mesh, ball);
		ball.release();

		// unit cylinder : radius 1, half-height 1. rings only live in the xz plane,
		// so non-uniform instance scaling stays exact and it can share one mesh
		uint cyl_slices = 16;
		Mesh_Data cyl = {};
		cyl.num_vertices = 2 * (cyl_slices + 1);
		cyl.num_indices  = cyl_slices * 6; // ring edges (x2) + verticals
		cyl.positions = Alloc(vec3, cyl.num_vertices);
		cyl.normals   = Alloc(vec3, cyl.num_vertices);
		cyl.uvs       = Alloc(vec2, cyl.num_vertices);
		cyl.indices   = Alloc(uint, cyl.num_indices);

		for (uint ring = 0; ring < 2; ring++)
			for (uint j = 0; j <= cyl_slices; j++)
			{
				float theta = 2.f * PI * j / cyl_slices;
				cyl.positions[ring * (cyl_slices + 1) + j] =
					vec3(cosf(theta), ring ? -1.f : 1.f, sinf(theta));
			}

		e = 0;
		for (uint j = 0; j < cyl_slices; j++)
		{
			uint top = j, bottom = cyl_slices + 1 + j;

			cyl.indices[e++] = top;    cyl.indices[e++] = top + 1;    // top ring
			cyl.indices[e++] = bottom; cyl.indices[e++] = bottom + 1; // bottom ring
			cyl.indices[e++] = top;    cyl.indices[e++] = bottom;     // vertical
		}

		cylinder_mesh = 3;
		lines.append_geometry(cylinder_mesh, cyl);
		cyl.release();

		// unit cone : base ring at y -0.5, apex at y +0.5 (radius 1, height 1)
		uint cone_slices = 16;
		Mesh_Data cone = {};
		cone.num_vertices = cone_slices + 1;
		cone.num_indices  = cone_slices * 4; // base ring edges + one edge to the apex
		cone.positions = Alloc(vec3, cone.num_vertices);
		cone.normals   = Alloc(vec3, cone.num_vertices);
		cone.uvs       = Alloc(vec2, cone.num_vertices);
		cone.indices   = Alloc(uint, cone.num_indices);

		for (uint j = 0; j < cone_slices; j++)
		{
			float theta = 2.f * PI * j / cone_slices;
			cone.positions[j] = vec3(cosf(theta), -.5f, sinf(theta));
		}
		cone.positions[cone_slices] = vec3(0, .5f, 0); // apex

		e = 0;
		for (uint j = 0; j < cone_slices; j++)
		{
			uint next = (j + 1) % cone_slices;
			cone.indices[e++] = j; cone.indices[e++] = next;        // base ring
			cone.indices[e++] = j; cone.indices[e++] = cone_slices; // to the apex
		}

		cone_mesh = 4;
		lines.append_geometry(cone_mesh, cone);
		cone.release();

		unit_mesh[PHYS_BOX]      = box_mesh;
		unit_mesh[PHYS_SPHERE]   = sphere_mesh;
		unit_mesh[PHYS_CYLINDER] = cylinder_mesh;
		unit_mesh[PHYS_CONE]     = cone_mesh;

		next_wire_mesh = 5; // per-body capsule wires take it from here
	}

	// an exact wire for one capsule body : non-uniform scaling would squash the
	// caps, so each body gets its own mesh (radius + cylinder height, same
	// convention as bullet and the mesh factory)
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
		lines.append_geometry(mesh_id, wire);
		wire.release();
		return mesh_id;
	}
};

// one hinge2 car : chassis + 4 wheels, suspension and drive on the hinges
struct Phys_Vehicle
{
	uint chassis_id;   // phys_id of the chassis (0 = no car built)
	uint wheel_ids[4]; // front pair first : the steered axle
	btHinge2Constraint* hinges[4];
	float spin_cmd, steer_cmd; // last input asked for : ux readout
};

// joints the scene built. bullet owns the constraints themselves, this is the
// list the ux reads : limits are radians and mean different things per kind
enum { PHYS_HINGE = 1, PHYS_CONETWIST, PHYS_HINGE2 };
const uint MAX_JOINTS = 32;

struct Phys_Joint
{
	uint kind;    // PHYS_HINGE / PHYS_CONETWIST / PHYS_HINGE2
	uint a, b;    // phys_ids the joint links
	float lo, hi; // hinge : (lo, hi) | cone twist : (swing, twist) | hinge2 : +-steer
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
	btDefaultCollisionConfiguration* config;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* broadphase;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* world;

	Phys_Body bodies[MAX_BODIES]; // [0] reserved : phys_id 0 = none
	uint num_bodies;
	Phys_Debug debug; // collider wireframes over the scene
	Phys_Vehicle vehicle; // the demo car, driven from update_vehicle()
	Phys_Joint joints[MAX_JOINTS]; // tracked joints, read by the ux tab
	uint num_joints;

	void init()
	{
		num_bodies = 1;
		num_joints = 0;

		config     = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(config);
		broadphase = new btDbvtBroadphase();
		solver     = new btSequentialImpulseConstraintSolver();
		world      = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
		world->setGravity(btVector3(0, -9.80665f, 0));

		debug.init();

		console->add_entry("Init Physics", SUCCESS, PHYS);
	}

	// one body in the world. mass 0 = static. tilt = spawn rotation, radians about
	// each axis. returns phys_id (0 = failed, error already logged)
	uint add_body(uint shape, vec3 pos, vec3 dims, float mass, vec3 tilt = vec3(0))
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

		body->setUserIndex(num_bodies); // handle rides inside bullet : raycasts read it back
		world->addRigidBody(body);

		bodies[num_bodies] = { body, shape, dims, mass };
		bodies[num_bodies].spawn_rot = start.getRotation();
		bodies[num_bodies].spawn_pos = pos;
		bodies[num_bodies].wire_mesh = (shape == PHYS_CAPSULE)
			? debug.append_capsule_wire(dims.x, dims.y) : 0; // 0 = shared unit shape
		return num_bodies++;
	}

	uint add_box(vec3 pos, vec3 half_extents, float mass, vec3 tilt = vec3(0))
	{
		return add_body(PHYS_BOX, pos, half_extents, mass, tilt);
	}

	uint add_sphere(vec3 pos, float radius, float mass)
	{
		return add_body(PHYS_SPHERE, pos, vec3(radius), mass);
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

	// static ground slab, top face exactly at y 0 (sits under a 60 x 60 floor)
	uint add_ground(float half_size)
	{
		return add_body(PHYS_BOX, vec3(0, -1, 0), vec3(half_size, 1, half_size), 0);
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

	// one limited hinge joint, straight into the world
	void add_hinge(btRigidBody& a, btRigidBody& b, btTransform fa, btTransform fb, float lo, float hi)
	{
		btHingeConstraint* j = new btHingeConstraint(a, b, fa, fb);
		j->setLimit(lo, hi);
		world->addConstraint(j, true); // no self-collision across the joint
		add_joint(PHYS_HINGE, a, b, lo, hi);
	}

	// one limited cone twist (swing, swing, twist)
	void add_cone(btRigidBody& a, btRigidBody& b, btTransform fa, btTransform fb, float swing, float twist)
	{
		btConeTwistConstraint* j = new btConeTwistConstraint(a, b, fa, fb);
		j->setLimit(swing, swing, twist);
		world->addConstraint(j, true);
		add_joint(PHYS_CONETWIST, a, b, swing, twist);
	}

	// remember a joint for the ux tab : every body carries its phys_id as user index
	void add_joint(uint kind, btRigidBody& a, btRigidBody& b, float lo, float hi)
	{
		if (num_joints >= MAX_JOINTS) return;
		joints[num_joints++] = { kind, (uint)a.getUserIndex(), (uint)b.getUserIndex(), lo, hi };
	}

	// vehicle : hinge2 suspension car - chassis box + 4 rolled cylinder wheels
	// with drive and steer motors on the hinges. returns chassis phys_id
	uint add_vehicle(vec3 pos)
	{
		if (num_bodies + 5 > MAX_BODIES)
		{
			console->add_entry("Not enough room for a vehicle", FIXME, PHYS);
			return 0;
		}

		vehicle.chassis_id = add_box(pos, vec3(1.f, .5f, 2.f), 2.f);
		if (!vehicle.chassis_id) return 0;

		btRigidBody* chassis = bodies[vehicle.chassis_id].body;
		chassis->setActivationState(DISABLE_DEACTIVATION); // a parked car stays ready

		vec3 at[4] = { vec3(-1, -.25f, 1.25f), vec3(1, -.25f, 1.25f), // front axle first
		               vec3(1, -.25f, -1.25f), vec3(-1, -.25f, -1.25f) };

		for (uint i = 0; i < 4; i++)
		{
			// y cylinder rolled a quarter turn : axle on world x for the hinge.
			// dims = (radius, half width, radius), the shared cylinder wire fits
			vehicle.wheel_ids[i] = add_cylinder(pos + at[i], vec3(.5f, .2f, .5f), 1.f,
			                                    vec3(0, 0, -PI * .5f));
			btRigidBody* wheel = bodies[vehicle.wheel_ids[i]].body;
			wheel->setFriction(1110.f);  // hinge2 cars grip through friction alone
			wheel->setRestitution(0.f);
			wheel->setActivationState(DISABLE_DEACTIVATION);

			btVector3 anchor(pos.x + at[i].x, pos.y + at[i].y, pos.z + at[i].z);
			btVector3 up(0, 1, 0), axle(1, 0, 0); // hinge2 reads axes in world space
			btHinge2Constraint* h = new btHinge2Constraint(*chassis, *wheel, anchor, up, axle);

			h->setParam(BT_CONSTRAINT_CFM, .15f, 2); // suspension give
			h->setParam(BT_CONSTRAINT_ERP, .35f, 2);
			h->setDamping(2, 2.f);
			h->setStiffness(2, 40.f);

			h->enableMotor(3, true); // spin : the wheels roll
			h->setMaxMotorForce(3, 1000);
			h->setTargetVelocity(3, 0);

			h->enableMotor(5, true); // steer : rate toward the +-45 lock
			h->setMaxMotorForce(5, 1000);
			h->setTargetVelocity(5, 0);

			world->addConstraint(h, true);
			vehicle.hinges[i] = h;
			add_joint(PHYS_HINGE2, *chassis, *wheel, -PI * .25f, PI * .25f); // ctor's +-45 steer lock
		}

		console->add_entry("Vehicle spawned : arrows drive", DEBUG, PHYS);
		return vehicle.chassis_id;
	}

	// drive : up/down spin the wheels, left/right steer the front pair
	void update_vehicle(GameWindow* window)
	{
		if (!vehicle.chassis_id) return;
		if (window->focus_status != GAME) return;      // menus own the input
		if (ImGui::GetIO().WantCaptureKeyboard) return; // typing into the console

		float spin = window->keys.UP.is_pressed ? -4.f : window->keys.DOWN.is_pressed ? 4.f : 0.f;
		float steer = window->keys.LEFT.is_pressed ? -1.f : window->keys.RIGHT.is_pressed ? 1.f : 0.f;
		vehicle.spin_cmd = spin;   // the ux shows what the driver is asking for
		vehicle.steer_cmd = steer;

		for (uint i = 0; i < 4; i++)
		{
			vehicle.hinges[i]->setTargetVelocity(3, spin);
			if (i < 2) vehicle.hinges[i]->setTargetVelocity(5, steer); // steered pair
		}
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

	// ---- live vehicle numbers for the ux ----

	// chassis speed along its own forward (the car's front is local z)
	float vehicle_speed()
	{
		if (!vehicle.chassis_id) return 0.f;
		btRigidBody* c = bodies[vehicle.chassis_id].body;
		btVector3 fwd = c->getWorldTransform().getBasis().getColumn(2);
		return (float)c->getLinearVelocity().dot(fwd);
	}

	// front hinge angles : steer (about the vertical) and wheel spin (about the axle)
	float vehicle_steer() { return vehicle.chassis_id ? (float)vehicle.hinges[0]->getAngle1() : 0.f; }
	float vehicle_spin()  { return vehicle.chassis_id ? (float)vehicle.hinges[0]->getAngle2() : 0.f; }

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

	// colliders as wireframes over the scene : every body, current transform
	void draw_debug(mat4 proj_view)
	{
		if (!debug.enabled) return;

		// after lighting the backbuffer depth is nobody's : start clean
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_DEPTH_BUFFER_BIT);

		// fresh instance stream : one CONTIGUOUS run per shape (the draw reads
		// num_instances from base_instance, so shapes can't interleave)
		debug.lines.clear_instances();

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
				if (shape == PHYS_CAPSULE) mesh = bodies[i].wire_mesh;
				else
				{
					model = glm::scale(model, bodies[i].dims);
					mesh  = debug.unit_mesh[shape];
				}

				Instance inst = INSTANCE_DEFAULT; // the wire shader ignores the material
				inst.model = model;
				debug.lines.append_instances(mesh, 1, &inst);
			}
		}

		glBindVertexArray(debug.lines.vao);
		debug.shader.bind();
		glUniformMatrix4fv(glGetUniformLocation(debug.shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);
		glUniform4f(glGetUniformLocation(debug.shader.id, "color"), 0.2f, 1.f, 0.4f, 1.f);

		draw_debug_shape(debug.box_mesh);
		draw_debug_shape(debug.sphere_mesh);
		draw_debug_shape(debug.cylinder_mesh);
		draw_debug_shape(debug.cone_mesh);

		// capsule wires are per-body meshes, one tiny draw each
		for (uint i = 1; i < num_bodies; i++)
			if (bodies[i].body && bodies[i].shape == PHYS_CAPSULE)
				draw_debug_shape(bodies[i].wire_mesh);
	}

	// one shape's line draw, straight from the buffer's bookkeeping
	void draw_debug_shape(uint mesh)
	{
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (debug.lines.mesh_info[i].mesh_id != mesh)
				continue;

			auto& mi = debug.lines.mesh_info[i];
			if (mi.num_instances)
				glDrawElementsInstancedBaseVertexBaseInstance(GL_LINES, mi.num_indices, GL_UNSIGNED_INT,
					(void*)mi.base_index, mi.num_instances, mi.base_vertex, mi.base_instance);
			return;
		}
	}
};
