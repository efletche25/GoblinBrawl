#include "stdafx.h"
#include "Skeleton.h"
#include <vector>

Skeleton::Skeleton() : numBones( 0 ) {}

Skeleton::~Skeleton() {
	for( auto it = idxBones.begin(); it!=idxBones.end(); ++it ) {
		delete it->second;
	}
}

void Skeleton::AddBone( Bone* newBone ) {
	++numBones;
	idxBones.insert( std::pair<int, Bone*>( newBone->idx, newBone ) );
	nameBones.insert( std::pair<std::string, Bone*>( newBone->name, newBone ) );
}

Bone* Skeleton::GetBoneByName( std::string name ) {
	auto nameIt = nameBones.find( name );
	if( nameIt==nameBones.end() ) {
		return nullptr;
	}
	return nameIt->second;
}

void XM_CALLCONV Skeleton::UpdateTransformByName( FXMVECTOR translate, FXMVECTOR rotQuat, FXMVECTOR scale, std::string name ) {
	Bone* bone = nameBones[name];
	XMVECTOR zeroVec = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	bone->localTransform = XMMatrixAffineTransformation(
		scale,
		zeroVec,
		rotQuat,
		translate
		);
}

void XM_CALLCONV Skeleton::UpdateTransformByIndex( FXMVECTOR translate, FXMVECTOR rotQuat, FXMVECTOR scale, int index ) {
	Bone* bone = idxBones[index];
	XMVECTOR zeroVec = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	bone->localTransform = XMMatrixAffineTransformation(
		scale,
		zeroVec,
		rotQuat,
		translate
		);
}

DirectX::XMFLOAT4X4* Skeleton::GetFinalTransforms() {
	//std::vector<XMFLOAT4X4> finalTransforms( numBones );
	std::vector<XMMATRIX> toRoot( numBones );
	auto it = idxBones.begin();
	Bone* bone = it->second;
	XMMATRIX localTransform = bone->localTransform;
	toRoot[it->second->idx] = localTransform;
	XMMATRIX offset = bone->offset;
	XMMATRIX finalTransform = offset*localTransform;
	//XMFLOAT4X4 temp;
	XMStoreFloat4x4( &finalTransformData[0], finalTransform );
	//finalTransforms[bone->idx] = finalTransform;
	++it;
	for( ; it!=idxBones.end(); ++it ) {
		bone = it->second;
		XMMATRIX localTransform = bone->localTransform;
		XMMATRIX parentToRoot = toRoot[bone->parentIdx];
		XMMATRIX boneToRoot = localTransform*parentToRoot;
		toRoot[bone->idx] = boneToRoot;
		XMMATRIX offset = bone->offset;
		XMMATRIX finalTransform = offset*boneToRoot;
		//XMFLOAT4X4 temp;
		XMStoreFloat4x4( &finalTransformData[bone->idx], finalTransform );
		//finalTransforms[bone->idx] = temp;
	}
	//finalTransformData = finalTransforms.data()[0];
	return finalTransformData;
}