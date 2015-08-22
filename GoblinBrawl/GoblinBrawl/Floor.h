#pragma once
class ModelLoader;
class Mesh;
class ID3D11DeviceContext;
class Floor {
public:
	Floor();
	~Floor();
	bool Init( ModelLoader* modelLoader );
	void Draw( ID3D11DeviceContext*	context );
private:
	Mesh* mesh;
};

