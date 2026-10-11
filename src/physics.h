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
const float GROUND_FRICTION  = .1f;

enum { PHYS_BOX = 1, PHYS_SPHERE, PHYS_CAPSULE, PHYS_CYLINDER }; // shape tags stored per body (0 = none)

struct Phys_Body
{
	btRigidBody* body;
	uint shape;
	vec3 dims; // box half extents / sphere radius (world units)
	float mass; // 0 = static
	btQuaternion spawn_rot; // pose it was born in : reset restores it
	uint wire_mesh; // 0 = shared unit wire, else a per-body capsule wire
};

// collider wireframes : a tiny line buffer holding unit box, sphere and
// cylinder edges, instanced at every body's transform. capsules get their own
// exact per-body wire (see append_capsule_wire). physics draws its own debug
// (own shader, own vao) - the renderer never hears about it
struct Phys_Debug
{
	bool enabled;
	DrawBuffer lines;     // unit shapes + one instance slot per body
	ShaderProgram shader; // same wire shaders as the renderer's overlay
	uint box_mesh, sphere_mesh, cylinder_mesh;
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

		next_wire_mesh = 4; // per-body capsule wires take it from here
	}

	// an exact wire for one capsule body, in local space. non-uniform instance
	// scaling would squash the hemispherical caps, so every capsule body gets
	// its own little mesh sized to its true radius + cylinder height (the same
	// convention as bullet's capsule shape and the mesh factory's generator)
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

	void init()
	{
		num_bodies = 1;

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

	// teleport a body back to a start pose : cleared, awake, ready to fall again
	void reset(uint phys_id, vec3 pos)
	{
		if (phys_id == 0 || phys_id >= num_bodies || !bodies[phys_id].body) return;

		btRigidBody* body = bodies[phys_id].body;

		btTransform start;
		start.setIdentity();
		start.setOrigin(btVector3(pos.x, pos.y, pos.z));
		start.setRotation(bodies[phys_id].spawn_rot); // back to the pose it was born in

		body->setWorldTransform(start);
		body->setInterpolationWorldTransform(start);
		body->getMotionState()->setWorldTransform(start);
		body->clearForces();
		body->setLinearVelocity(btVector3(0, 0, 0));
		body->setAngularVelocity(btVector3(0, 0, 0));
		body->activate(true);
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

		for (uint shape = PHYS_BOX; shape <= PHYS_CYLINDER; shape++)
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

				// unit shapes scale to fit : the sphere by radius, the box and
				// cylinder by half extents. capsule wires are pre-sized already
				uint mesh;
				if (shape == PHYS_SPHERE)
				{
					model = glm::scale(model, vec3(bodies[i].dims.x));
					mesh  = debug.sphere_mesh;
				}
				else if (shape == PHYS_CAPSULE)
				{
					mesh = bodies[i].wire_mesh;
				}
				else
				{
					model = glm::scale(model, bodies[i].dims);
					mesh  = (shape == PHYS_CYLINDER) ? debug.cylinder_mesh : debug.box_mesh;
				}

				debug.lines.append_instances(mesh, 1, &model);
			}
		}

		glBindVertexArray(debug.lines.vao);
		debug.shader.bind();
		glUniformMatrix4fv(glGetUniformLocation(debug.shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);
		glUniform4f(glGetUniformLocation(debug.shader.id, "color"), 0.2f, 1.f, 0.4f, 1.f);

		draw_debug_shape(debug.box_mesh);
		draw_debug_shape(debug.sphere_mesh);
		draw_debug_shape(debug.cylinder_mesh);

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
