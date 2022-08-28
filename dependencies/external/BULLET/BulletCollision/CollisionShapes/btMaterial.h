/// This file was created by Alex Silverman

#ifndef BT_MATERIAL_H
#define BT_MATERIAL_H

// Material class to be used by btMultimaterialTriangleMeshShape to store triangle properties
class btMaterial
{
	// public members so that materials can change due to world events
public:
	btScalar m_friction;
	btScalar m_restitution;
	int pad[2];

	btMaterial() {}
	btMaterial(btScalar fric, btScalar rest)
	{
		m_friction = fric;
		m_restitution = rest;
	}
};

#endif  // BT_MATERIAL_H
