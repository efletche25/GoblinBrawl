#include "stdafx.h"
#include "PhysicsDebugDrawer.h"

PhysicsDebugDrawer::PhysicsDebugDrawer() : ctx(nullptr) 
{}

PhysicsDebugDrawer::~PhysicsDebugDrawer() {}

bool PhysicsDebugDrawer::Init( ID3D11DeviceContext* device ) {
	this->ctx = device;

	return true;
}

void PhysicsDebugDrawer::Begin() {
	ctx->OMGetBlendState(&oldBlendState, oldBlendFactor,  NULL);
	ctx->OMGetDepthStencilState(&oldStencilState, &oldStencilRef);
	ctx->RSGetState( &oldRasterizerState );
}

void PhysicsDebugDrawer::End() {
	ctx->OMSetBlendState( oldBlendState, oldBlendFactor, 0xffffffff );
	ctx->OMSetDepthStencilState( oldStencilState, oldStencilRef );
}