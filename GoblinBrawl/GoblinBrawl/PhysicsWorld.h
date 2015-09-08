#pragma once
#include "DirectX_11_1_Includes.h"
#include "btBulletDynamicsCommon.h"

#define PHYSICS_DEBUG_MODE

class PhysicsDebugDrawer;

class PhysicsWorld {
public:
	PhysicsWorld();
	~PhysicsWorld();
	bool Init();
	bool Init( ID3D11DeviceContext* device );
	void SetupDemo();
	void RunDemo();
	void Update( float dt );
	inline void AddCollisionShape( btCollisionShape* shape);
	inline void AddRigidBody( btRigidBody* rb );
	void XM_CALLCONV DrawDebug( DirectX::FXMMATRIX viewProj );
private:
	void CleanUpDemo();
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

