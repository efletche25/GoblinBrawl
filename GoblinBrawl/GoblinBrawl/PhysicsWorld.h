#pragma once
#include "btBulletDynamicsCommon.h"

class PhysicsWorld {
public:
	PhysicsWorld();
	~PhysicsWorld();
	bool init();
private:
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
};

