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
		aiProcess_ImproveCacheLocality|
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
		CreateSkeleton( sceneMesh->mBones, sceneMesh->mNumBones );
		CreateBoneHierarchy();
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
			assert( itlow!=itup ); // every vertex should have some influence
			for( auto it = itlow; it!=itup; ++it ) {
				if( j>=4 ) {
					break;
				}
				assert( j<4 ); // each vertex should not be influenced by more than 4 bones
				boneIndices[j] = it->second.boneIndex;
				weights[j] = it->second.weight;
				++j;
			}
			vertData[i].BoneIndicies[0] = boneIndices[0];
			vertData[i].BoneIndicies[1] = boneIndices[1];
			vertData[i].BoneIndicies[2] = boneIndices[2];
			vertData[i].BoneIndicies[3] = boneIndices[3];
			vertData[i].Weights = XMFLOAT4( weights );
			fprintf( stdout, "Total Weight: %f\n", weights[0]+weights[1]+weights[2]+weights[3] );

			assert( weights[0]+weights[1]+weights[2]+weights[3]<1.0001f );
			assert( weights[0]+weights[1]+weights[2]+weights[3]>0.9999f );
		}
		SetVertices( device, count, vertData.data() );
		break;
	}
	}
}

void ModelLoader::CreateSkeleton( aiBone** bones, int numBones ) {
	XMMATRIX fbxScale = XMMatrixScaling( 0.01f, 0.01f, 0.01f );
	XMMATRIX lefthandedConversion(
		1.f, 0.f, 0.f, 0.f,
		0.f, 1.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f,
		0.f, 0.f, 0.f, 1.f );
	XMMATRIX fbxConversionMatrix(
		1.f, 0.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f,
		0.f, 1.f, 0.f, 0.f,
		0.f, 0.f, 0.f, 1.f );
	XMMATRIX fbxQuatInverse(
		0.f, -1.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f,
		-1.f, 0.f, 0.f, 0.f,
		0.f, 0.f, 0.f, 0.f );
	skeleton = new Skeleton();
	for( int i = 0; i<numBones; ++i ) {
		Bone* newBone = new Bone();
		aiBone* bone = bones[i];
		newBone->idx = i;
		newBone->name = bone->mName.data;
		auto boneOffset = bone->mOffsetMatrix;
		XMMATRIX convertedOffset = ConvertMatrix( boneOffset );
		XMVECTOR determinant = XMMatrixDeterminant( convertedOffset );
		XMMATRIX convertedOffsetInverse = XMMatrixInverse( &determinant, convertedOffset );

		auto newBoneOffsetMatrix = XMMATRIX(
			boneOffset.a1, boneOffset.a2, boneOffset.a3, boneOffset.a4,
			boneOffset.b1, boneOffset.b2, boneOffset.b3, boneOffset.b4,
			boneOffset.c1, boneOffset.c2, boneOffset.c3, boneOffset.c4,
			boneOffset.d1, boneOffset.d2, boneOffset.d3, boneOffset.d4 );
		auto transNewBoneOffsetMatrix = XMMATRIX(
			boneOffset.a1, boneOffset.b1, boneOffset.c1, boneOffset.d1,
			boneOffset.a2, boneOffset.b2, boneOffset.c2, boneOffset.d2,
			boneOffset.a3, boneOffset.b3, boneOffset.c3, boneOffset.d3,
			boneOffset.a4, boneOffset.b4, boneOffset.c4, boneOffset.d4 );
		XMMATRIX scale = XMMatrixScaling( 1.f, 1.f, 1.f );
		XMMATRIX rotX = XMMatrixRotationRollPitchYaw( 0.f, 0.f, 0.f );
		XMMATRIX translate = XMMatrixTranslation( 0.f, 0.f, 0.f );
		XMMATRIX world = scale * rotX * translate;

		XMVECTOR scaleDecomp;
		XMVECTOR rotQuatDecomp;
		XMVECTOR transDecomp;
		bool x = XMMatrixDecompose( &scaleDecomp, &rotQuatDecomp, &transDecomp, transNewBoneOffsetMatrix );

		XMVECTOR testRot = XMQuaternionRotationRollPitchYaw( -XM_PIDIV4, 0.f, 0.f );
		testRot = XMQuaternionRotationRollPitchYaw( 0.f, -XM_PIDIV4, 0.f );
		testRot = XMQuaternionRotationRollPitchYaw( 0.f, 0.f, -XM_PIDIV4 );
		testRot = XMQuaternionRotationRollPitchYaw( -XM_PIDIV4, 0.f, -XM_PIDIV4 );
		testRot = XMQuaternionRotationRollPitchYaw( -XM_PIDIV4, 0.f, XM_PIDIV4 );
		testRot = XMQuaternionRotationRollPitchYaw( XM_PIDIV4, 0.f, -XM_PIDIV4 );
		testRot = XMQuaternionRotationRollPitchYaw( XM_PIDIV4, 0.f, -XM_PIDIV4 );

		XMMATRIX testMat = transNewBoneOffsetMatrix*fbxQuatInverse;
		XMMATRIX testMat2 = fbxQuatInverse*transNewBoneOffsetMatrix;

		XMVECTOR axis;
		float angle;
		XMQuaternionToAxisAngle( &axis, &angle, rotQuatDecomp );

		XMVECTOR w_scaleDecomp;
		XMVECTOR w_rotQuatDecomp;
		XMVECTOR w_transDecomp;
		x = XMMatrixDecompose( &w_scaleDecomp, &w_rotQuatDecomp, &w_transDecomp, world );

		XMVECTOR quatInverse = XMQuaternionInverse( rotQuatDecomp );
		XMVECTOR fbxQuatInverseVector = XMLoadFloat4( &XMFLOAT4( 0.5, 0.5, -0.5, 0.5 ) );
		XMVECTOR quatFinal = XMQuaternionMultiply( rotQuatDecomp, fbxQuatInverseVector );

		XMVECTOR zero = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 0.f ) );
		XMMATRIX finalTransform = XMMatrixAffineTransformation( scaleDecomp, zero, XMQuaternionIdentity(), transDecomp );

		newBoneOffsetMatrix = XMMatrixMultiply( newBoneOffsetMatrix, fbxQuatInverse );

		newBone->offset = convertedOffset;

		skeleton->AddBone( newBone );
	}
}

void ModelLoader::CreateBoneHierarchy() {
	aiNode* root = scene->mRootNode->FindNode( "Skeleton_Root" );
	FindBoneChildren( root, -1 );
}

void ModelLoader::FindBoneChildren( aiNode* node, int parentIdx ) {
	Bone* bone = skeleton->GetBoneByName( node->mName.data );
	bone->parentIdx = parentIdx;
	if( node->mNumChildren==0 ) { return; }
	for( int i = 0; i<node->mNumChildren; ++i ) {
		aiNode* childNode = node->mChildren[i];
		std::string childName = childNode->mName.data;
		Bone* childBone = skeleton->GetBoneByName( childName );
		if( childBone==nullptr ) {
			// Bones with no skin influence will be missing from the previous list
			childBone = new Bone();
			childBone->idx = skeleton->BoneCount();
			childBone->name = childName;
			XMMATRIX transform = ConvertMatrix( childNode->mTransformation );
			XMMATRIX parentOffset = bone->offset;
			childBone->offset = DirectX::XMMatrixMultiply( transform, parentOffset );
			skeleton->AddBone( childBone );
		}
		bone->children.push_back( childBone );
		FindBoneChildren( childNode, bone->idx );
	}
}

DirectX::XMMATRIX XM_CALLCONV ModelLoader::ConvertMatrix( aiMatrix4x4 inMat ) {
	/*DirectX::XMMATRIX outMat = XMMATRIX(
		inMat.a1, inMat.a2, inMat.a3, inMat.a4,
		inMat.b1, inMat.b2, inMat.b3, inMat.b4,
		inMat.c1, inMat.c2, inMat.c3, inMat.c4,
		inMat.d1, inMat.d2, inMat.d3, inMat.d4);*/
	DirectX::XMMATRIX transposed = XMMATRIX(
		inMat.a1, inMat.b1, inMat.c1, inMat.d1,
		inMat.a2, inMat.b2, inMat.c2, inMat.d2,
		inMat.a3, inMat.b3, inMat.c3, inMat.d3,
		inMat.a4, inMat.b4, inMat.c4, inMat.d4 );
	DirectX::XMMATRIX fbxConvertTranslate = XMMATRIX(
		1.f, 0.f, 0.f, 0.f,
		0.f, -1.f, 0.f, 0.f,
		0.f, 0.f, -1.f, 0.f,
		0.f, 0.f, 0.f, 1.f);

	XMVECTOR scaleDecomp;
	XMVECTOR rotQuatDecomp;
	XMVECTOR transDecomp;
	XMMatrixDecompose( &scaleDecomp, &rotQuatDecomp, &transDecomp, transposed );

	XMMATRIX rotMat = XMMatrixRotationQuaternion(rotQuatDecomp);
	XMMATRIX xRot = XMMatrixRotationX( XM_PIDIV2 );
	XMMATRIX yRot = XMMatrixRotationY( 0 );
	XMMATRIX zRot = XMMatrixRotationZ( 0 );
	XMMATRIX rot = rotMat*xRot*zRot;

	XMMATRIX scale = XMMatrixScalingFromVector( scaleDecomp );
	XMMATRIX translate = XMMatrixTranslationFromVector( transDecomp );
	XMMATRIX convertedTranslate = translate*fbxConvertTranslate;

	//XMVECTOR zero = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 0.f ) );
	//XMMATRIX finalTransform = XMMatrixAffineTransformation( scaleDecomp, zero, XMQuaternionIdentity(), transDecomp );
	//XMMATRIX testTransform = fbxConvert*transposed;
	XMMATRIX finalTransform = scale*rot*convertedTranslate;
	
	return finalTransform;
}