#pragma once
#include "btBulletDynamicsCommon.h"

class PhysicsDebugDrawer;

class PhysicsWorld {
public:
	PhysicsWorld();
	~PhysicsWorld();
	bool init();
	void setupDemo();
	void runDemo();
private:
	void cleanUpDemo();
	btDefaultCollisionConfiguration*		collisionConfiguration;
	btCollisionDispatcher*					dispatcher;
	btBroadphaseInterface*					overlappingPairCache;
	btSequentialImpulseConstraintSolver*	solver;
	btDiscreteDynamicsWorld*				dynamicsWorld;

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;
	PhysicsDebugDrawer*						debugDrawer;
};

