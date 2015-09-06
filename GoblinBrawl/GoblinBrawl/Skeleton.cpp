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

Bone* Skeleton::GetBoneByIndex( int index ) {
	auto indexIt = idxBones.find( index );
	if( indexIt==idxBones.end() ) {
		return nullptr;
	}
	return indexIt->second;
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
	toRoot.resize( numBones );
	Bone* root = GetBoneByName( "Skeleton_Root" );
	UpdateTransforms( root );
	return finalTransformData;
}

void Skeleton::UpdateTransforms(Bone* bone) {
	XMMATRIX localTransform = bone->localTransform;
	XMMATRIX offset = bone->offset;
	XMMATRIX finalTransform;
	if( bone->parentIdx == -1 ) {
		// The root bone
		toRoot[bone->idx] = localTransform;
		finalTransform = offset*localTransform;
	} else {
		XMMATRIX parentToRoot = toRoot[bone->parentIdx];
		XMMATRIX boneToRoot = localTransform*parentToRoot;
		toRoot[bone->idx] = boneToRoot;
		finalTransform = offset*boneToRoot;
	}
	XMStoreFloat4x4( &finalTransformData[bone->idx], finalTransform );
	if( bone->children.size()==0 ) {return;}
	for( Bone* childBone : bone->children) {
		UpdateTransforms(childBone);
	}
}