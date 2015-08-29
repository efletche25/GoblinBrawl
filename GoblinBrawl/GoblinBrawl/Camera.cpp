#include "stdafx.h"
#include "Camera.h"

Camera::Camera() :
targetOffset( 1.F ),
nearZ( 1.f ),
farZ( 10000.f ),
fovAngleY( XM_PIDIV4 ) {}

Camera::~Camera() {}

void Camera::Init( float aspectRatio ) {
	view = XMMatrixIdentity();
	up = XMVectorSet( 0.f, 1.f, 0.f, 0.f );
	proj = XMMatrixPerspectiveFovLH( fovAngleY, aspectRatio, nearZ, farZ );
}

void XM_CALLCONV Camera::Update( FXMVECTOR _pos, FXMVECTOR dir ) {
	pos = _pos;
	XMVECTOR normDir = XMVector3Normalize( dir );

	XMVECTOR target = XMVectorAdd( pos, XMVectorScale( normDir, targetOffset ) );

	//FIXME
	target = XMVectorSet( 0.f, 0.f, 0.f, 0.f );

	view = XMMatrixLookAtLH( pos, target, up );
	viewProj = view*proj;
}

XMMATRIX XM_CALLCONV Camera::GetViewProj() {
	return viewProj;
}

XMVECTOR XM_CALLCONV Camera::GetPos() {
	return pos;
}