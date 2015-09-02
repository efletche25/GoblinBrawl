#include "stdafx.h"
#include "ModelLoader.h"
#include <map>
#include <assert.h>
#include "assimp/DefaultLogger.hpp"
#include "Mesh.h"
#include "Skeleton.h"

ModelLoader::ModelLoader( ID3D11Device* device, std::string modelDir, std::string textureDir ) :
device( device ),
modelDir( modelDir ),
textureDir( textureDir ),
scene( nullptr ) {
	Assimp::DefaultLogger::create( "assimp.log", Assimp::Logger::VERBOSE );
}

ModelLoader::~ModelLoader() {
	Assimp::DefaultLogger::kill();
}

bool ModelLoader::Load( std::string filename, Vertex::VERTEX_TYPE type ) {
	Assimp::Importer importer;
	std::string file = modelDir+filename;
	Assimp::DefaultLogger::get()->info( "Importing: "+file );
	scene = importer.ReadFile( file,
		//aiProcess_CalcTangentSpace|
		aiProcess_MakeLeftHanded|
		aiProcess_FlipWindingOrder|
		aiProcess_Triangulate|
		aiProcess_JoinIdenticalVertices|
		aiProcess_SortByPType|
		aiProcess_FlipUVs
		);

	if( !scene ) {
		Assimp::DefaultLogger::get()->error( importer.GetErrorString() );
		return false;
	}
	if( !scene->HasMeshes() ) {
		Assimp::DefaultLogger::get()->error( "File contains no mesh" );
		return false;
	}
	aiMesh* sceneMesh = scene->mMeshes[0];
	CreateVertexBuffer( sceneMesh, type );
	CreateIndexBuffer( sceneMesh->mFaces, sceneMesh->mNumFaces );
	if( scene->HasAnimations() ) {
		CreateSkeleton(sceneMesh->mBones, sceneMesh->mNumBones);
	}
	return true;
}

Mesh* ModelLoader::GetMesh() {
	Mesh* mesh = new Mesh();
	mesh->SetVB( vb );
	mesh->SetIB( ib, indexCount );
	return mesh;
}

Skeleton* ModelLoader::GetSkeleton() {
	return skeleton;
}

std::vector<PointLight> ModelLoader::GetPointLights() {
	return pointLights;
}

void ModelLoader::CreateIndexBuffer( const aiFace* indices, UINT count ) {
	indexCount = count*3;
	std::vector<USHORT> indexData( count*3 );
	for( UINT faceIndex = 0, dataIndex = 0; faceIndex<count; ++faceIndex, dataIndex += 3 ) {
		assert( indices[faceIndex].mNumIndices==3 ); //mesh should be triangulated
		indexData[dataIndex] = (USHORT)indices[faceIndex].mIndices[0];
		indexData[dataIndex+1] = (USHORT)indices[faceIndex].mIndices[1];
		indexData[dataIndex+2] = (USHORT)indices[faceIndex].mIndices[2];
	}

	D3D11_BUFFER_DESC ibd;
	ibd.Usage = D3D11_USAGE_IMMUTABLE;
	ibd.ByteWidth = sizeof( USHORT ) * indexData.size();
	ibd.BindFlags = D3D11_BIND_INDEX_BUFFER;
	ibd.CPUAccessFlags = 0;
	ibd.MiscFlags = 0;
	ibd.StructureByteStride = 0;

	D3D11_SUBRESOURCE_DATA iinitData;
	iinitData.pSysMem = indexData.data();

	HR( device->CreateBuffer( &ibd, &iinitData, &ib ) );
}

void ModelLoader::CreateVertexBuffer( aiMesh* mesh, Vertex::VERTEX_TYPE type ) {
	UINT count = mesh->mNumVertices;
	aiVector3D* vertices = mesh->mVertices;
	switch( type ) {
	case Vertex::SIMPLE:
	{
		std::vector<Vertex::SimpleVertex> vertData( count );
		for( UINT i = 0; i<count; ++i ) {
			vertData[i].Pos = XMFLOAT3( vertices[i].x, vertices[i].y, vertices[i].z );
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	case Vertex::CHARACTER:
	case Vertex::TERRAIN:
	{
		aiVector3D* normals = mesh->mNormals;
		aiVector3D* texCoords = mesh->mTextureCoords[0];
		std::vector<Vertex::TerrainVertex> vertData( count );
		for( UINT i = 0; i<count; ++i ) {
			vertData[i].Pos = XMFLOAT3( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Normal = XMFLOAT3( normals[i].x, normals[i].y, normals[i].z );
			vertData[i].Tex = XMFLOAT2( texCoords[i].x, texCoords[i].y );
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	case Vertex::CHARACTER_SKINNED:
	{
		std::multimap<int, BoneWeight> vertexBoneWeight;
		for( int boneIndex = 0; boneIndex<mesh->mNumBones; ++boneIndex ) {
			auto bone = mesh->mBones[boneIndex];
			for( int i = 0; i<bone->mNumWeights; ++i ) {
				auto boneWeight = BoneWeight( boneIndex, bone->mWeights[i].mWeight );
				vertexBoneWeight.insert( std::pair<int, BoneWeight>( bone->mWeights[i].mVertexId, boneWeight ) );
			}
		}
		aiVector3D* normals = mesh->mNormals;
		aiVector3D* texCoords = mesh->mTextureCoords[0];
		std::vector<Vertex::CharacterSkinnedVertex> vertData( count );
		for( UINT i = 0; i<count; ++i ) {
			vertData[i].Pos = XMFLOAT3( vertices[i].x, vertices[i].y, vertices[i].z );
			vertData[i].Normal = XMFLOAT3( normals[i].x, normals[i].y, normals[i].z );
			vertData[i].Tex = XMFLOAT2( texCoords[i].x, texCoords[i].y );
			
			BYTE boneIndices[4] = { 0, 0, 0, 0 };
			float weights[4] = { 0, 0, 0, 0 };
			int j = 0;
			auto itlow = vertexBoneWeight.lower_bound( i );
			auto itup = vertexBoneWeight.upper_bound( i );
			for( auto it = itlow; it!=itup; ++it ) {
				assert(j<4); // each vertex should not be influenced by more than 4 bones
				boneIndices[j] = it->second.boneIndex;
				weights[j] = it->second.weight;
				++j;
			}
			vertData[i].BoneIndicies[0] = boneIndices[0];
			vertData[i].BoneIndicies[1] = boneIndices[1];
			vertData[i].BoneIndicies[2] = boneIndices[2];
			vertData[i].BoneIndicies[3] = boneIndices[3];
			vertData[i].Weights = XMFLOAT4(weights);
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	}
}

void ModelLoader::CreateSkeleton( aiBone** bones, int numBones ) {
	skeleton = new Skeleton();
	for( int i = 0; i<numBones; ++i ) {
		Bone* newBone = new Bone();
		aiBone* bone = bones[i];
		newBone->idx = i; // plus one because the root is not in the bones array
		newBone->name = bone->mName.data;
		auto boneOffset = bone->mOffsetMatrix;
		newBone->offset = XMMATRIX( 
			boneOffset.a1, boneOffset.a2, boneOffset.a3, boneOffset.a4,
			boneOffset.b1, boneOffset.b2, boneOffset.b3, boneOffset.b4,
			boneOffset.c1, boneOffset.c2, boneOffset.c3, boneOffset.c4,
			boneOffset.d1, boneOffset.d2, boneOffset.d3, boneOffset.d4 );
		std::string parentName = scene->mRootNode->FindNode( bone->mName )->mParent->mName.data;
		Bone* parentNode = skeleton->GetBoneByName( parentName );
		if( parentNode!=nullptr ) {
			newBone->parentIdx = skeleton->GetBoneByName( parentName )->idx;
		} else {
			newBone->parentIdx = -1;
		}
		skeleton->AddBone( newBone );
	}
}