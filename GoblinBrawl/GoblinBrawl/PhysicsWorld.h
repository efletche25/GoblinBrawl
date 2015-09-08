#pragma once
#include "DirectX_11_1_Includes.h"
#include "btBulletDynamicsCommon.h"

#define PHYSICS_DEBUG_MODE

class PhysicsDebugDrawer;

class PhysicsWorld {
public:
	PhysicsWorld();
	~PhysicsWorld();
	bool init();
	bool init( ID3D11DeviceContext* device );
	void setupDemo();
	void runDemo();
	inline void addCollisionShape( btCollisionShape* shape);
	inline void addRigidBody( btRigidBody* rb );
	void XM_CALLCONV drawDebug( DirectX::FXMMATRIX viewProj );
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

