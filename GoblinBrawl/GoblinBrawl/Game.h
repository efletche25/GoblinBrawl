#pragma once
#include <vector>
#include <memory>
#include "DirectX_11_1_Includes.h"
#include "GameTimer.h"
#include "Camera.h"
#include "Floor.h"
#include "Lava.h"
#include "Walls.h"
#include "FirePlinth.h"
#include "Goblin.h"
#include "Lighting.h"
#include "Keyboard.h"
#include "GamePad.h"
#include "Audio.h"
#include <random>

class PhysicsWorld;

#define MAX_LOADSTRING 100

class Game {
public:
	Game( HINSTANCE hInstance, int clientWidth, int clientHeight );
	~Game();
	bool Init();
	void OnResize();
	void PlaySound();
	LRESULT MsgProc( HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam );
	int Run();
private:
	bool InitMainWindow();
	bool InitDirect3D();
	bool LoadGameObjects();
	void DisplayWinError( LPTSTR lpszFunction );
	void CalculateFrameStats();
	void Update(float dt);
	void Draw();
	float AspectRatio();
	TCHAR								wndTitle[MAX_LOADSTRING];					
	TCHAR								wndClass[MAX_LOADSTRING];	
	HINSTANCE							hAppInstance;
	HWND								hMainWnd;
	ID3D11Device*						d3DDevice;
	ID3D11DeviceContext*				d3DImmediateContext;
	ID3D11RasterizerState*				d3DRasterizerState;
	UINT								msaaQuality;
	bool								enable4xMSAA;
	int									clientWidth;
	int									clientHeight;
	IDXGISwapChain*						swapChain;
	ID3D11Texture2D*					depthStencilBuffer;
	ID3D11RenderTargetView*				renderTargetView;
	ID3D11DepthStencilView*				depthStencilView;
	D3D11_VIEWPORT						screenViewport;
	GameTimer							timer;
	bool								paused;
	bool								resizing;
	Camera								camera;
	Floor								floor;
	Walls								walls;
	Lava								lava;
	FirePlinth							firePlinth;
	Lighting							lighting;
	Goblin								goblin;
	PhysicsWorld*						physicsWorld;
	std::unique_ptr<Keyboard>			keyboard;
	std::unique_ptr<
		Keyboard::KeyboardStateTracker>	kbTracker;
	std::unique_ptr<GamePad>			gamePad;
	std::unique_ptr<DirectX::AudioEngine> m_audEngine;
	bool m_retryAudio;
	std::unique_ptr<DirectX::SoundEffect> m_explode;
	std::unique_ptr<DirectX::SoundEffect> m_ambient;
	std::unique_ptr<std::mt19937> m_random;
	float explodeDelay;

};

