#include "stdafx.h"
#include "Game.h"
#include "resource.h"
#include "d3dUtil.h"
#include <assert.h>
#include "Strsafe.h"
#include <sstream>
#include "ModelLoader.h"
#include "Floor.h"
#include "Effects.h"
#include "Vertex.h"

#define DISPLAY_FPS

namespace {
	// This is just used to forward Windows messages from a global window
	// procedure to our member function window procedure because we cannot
	// assign a member function to WNDCLASS::lpfnWndProc.
	Game* game = 0;
}

LRESULT CALLBACK MainWndProc( HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam ) {
	// Forward hwnd on because we can get messages (e.g., WM_CREATE)
	// before CreateWindow returns, and thus before mhMainWnd is valid.
	return game->MsgProc( hwnd, msg, wParam, lParam );
}

Game::Game( HINSTANCE hInstance, int clientWidth, int clientHeight ) :
hAppInstance( hInstance ),
hMainWnd( nullptr ),
clientWidth( clientWidth ),
clientHeight( clientHeight ),
d3DDevice( nullptr ),
d3DImmediateContext( nullptr ),
enable4xMSAA( false ),
swapChain( nullptr ),
depthStencilBuffer( nullptr ),
renderTargetView( nullptr ),
depthStencilView( nullptr ),
paused( false ) {
	ZeroMemory( &screenViewport, sizeof( D3D11_VIEWPORT ) );
	game = this;
}

Game::~Game() {
	ReleaseCOM( renderTargetView );
	ReleaseCOM( depthStencilView );
	ReleaseCOM( swapChain );
	ReleaseCOM( depthStencilBuffer );
	if( d3DImmediateContext ) {
		d3DImmediateContext->ClearState();
	}
	ReleaseCOM( d3DImmediateContext );
	ReleaseCOM( d3DDevice );
}

bool Game::Init() {
	camera = Camera();
	if( !InitMainWindow() ) {
		return false;
	}
	if( !InitDirect3D() ) {
		return false;
	}
	Effects::InitAll( d3DDevice );
	InputLayouts::InitAll( d3DDevice );
	if( !LoadGameObjects() ) {
		return false;
	}
	return true;
}

bool Game::InitMainWindow() {
	// Initialize global strings
	LoadString( hAppInstance, IDS_APP_TITLE, wndClass, MAX_LOADSTRING );
	LoadString( hAppInstance, IDC_GOBLINBRAWL, wndTitle, MAX_LOADSTRING );

	WNDCLASSEX wcex;
	wcex.cbSize = sizeof( WNDCLASSEX );

	wcex.style = CS_HREDRAW|CS_VREDRAW;
	wcex.lpfnWndProc = MainWndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hAppInstance;
	wcex.hIcon = LoadIcon( hAppInstance, MAKEINTRESOURCE( IDI_GOBLINBRAWL ) );
	wcex.hCursor = LoadCursor( NULL, IDC_ARROW );
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName = MAKEINTRESOURCE( IDC_GOBLINBRAWL );
	wcex.lpszClassName = wndClass;
	wcex.hIconSm = LoadIcon( wcex.hInstance, MAKEINTRESOURCE( IDI_SMALL ) );

	if( !RegisterClassEx( &wcex ) ) {
		DisplayWinError( L"RegisterClassEX" );
		return false;
	}

	RECT R = { 0, 0, clientWidth, clientHeight };
	AdjustWindowRect( &R, WS_OVERLAPPEDWINDOW, false );
	int width = R.right-R.left;
	int height = R.bottom-R.top;

	hMainWnd = CreateWindow( wndClass, wndTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, width, height, NULL, NULL, hAppInstance, NULL );

	if( !hMainWnd ) {
		DisplayWinError( L"CreateWindow" );
		return false;
	}

	ShowWindow( hMainWnd, SW_SHOW );
	UpdateWindow( hMainWnd );

	return true;
}

bool Game::InitDirect3D() {
	UINT createDeviceFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)
	createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
	D3D_FEATURE_LEVEL featureLevel;
	HRESULT hr = D3D11CreateDevice(
		0,					// default adaptor
		D3D_DRIVER_TYPE_HARDWARE,
		0,					// no software device
		createDeviceFlags,
		0, 0,				// default feature level array
		D3D11_SDK_VERSION,
		&d3DDevice,
		&featureLevel,
		&d3DImmediateContext );
	if( FAILED( hr ) ) {
		MessageBox( 0, L"D3D11CreateDevice Failed.", 0, 0 );
		return false;
	}
	if( featureLevel!=D3D_FEATURE_LEVEL_11_0 ) {
		MessageBox( 0, L"Direct3D Feature Level 11 unsupported.", 0, 0 );
		return false;
	}

	HR( d3DDevice->CheckMultisampleQualityLevels( DXGI_FORMAT_B8G8R8A8_UNORM, 4, &msaaQuality ) );
	assert( msaaQuality>0 );

	DXGI_SWAP_CHAIN_DESC sd;
	sd.BufferDesc.Width = clientWidth;
	sd.BufferDesc.Height = clientHeight;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	sd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;

	if( enable4xMSAA ) {
		sd.SampleDesc.Count = 4;
		sd.SampleDesc.Quality = msaaQuality-1;
	} else {
		sd.SampleDesc.Count = 1;
		sd.SampleDesc.Quality = 0;
	}

	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.BufferCount = 1;
	sd.OutputWindow = hMainWnd;
	sd.Windowed = true;
	sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	sd.Flags = 0;

	IDXGIDevice* dxgiDevice = 0;
	HR( d3DDevice->QueryInterface( __uuidof(IDXGIDevice), (void**)&dxgiDevice ) );

	IDXGIAdapter* dxgiAdaptor = 0;
	HR( dxgiDevice->GetParent( __uuidof(IDXGIAdapter), (void**)&dxgiAdaptor ) );

	IDXGIFactory* dxgiFactory = 0;
	HR( dxgiAdaptor->GetParent( __uuidof(IDXGIFactory), (void**)&dxgiFactory ) );

	HR( dxgiFactory->CreateSwapChain( d3DDevice, &sd, &swapChain ) );

	ReleaseCOM( dxgiDevice );
	ReleaseCOM( dxgiAdaptor );
	ReleaseCOM( dxgiFactory );

	OnResize();

	return true;
}

void Game::OnResize() {
	assert( d3DImmediateContext );
	assert( d3DDevice );
	assert( swapChain );

	//release old views
	ReleaseCOM( renderTargetView );
	ReleaseCOM( depthStencilView );
	ReleaseCOM( depthStencilBuffer );

	HR( swapChain->ResizeBuffers( 1, clientWidth, clientHeight, DXGI_FORMAT_R8G8B8A8_UNORM, 0 ) );
	ID3D11Texture2D* backbuffer;
	HR( swapChain->GetBuffer( 0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&backbuffer) ) );
	HR( d3DDevice->CreateRenderTargetView( backbuffer, 0, &renderTargetView ) );
	ReleaseCOM( backbuffer );

	D3D11_TEXTURE2D_DESC depthStencilDesc;
	depthStencilDesc.Width = clientWidth;
	depthStencilDesc.Height = clientHeight;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	if( enable4xMSAA ) {
		depthStencilDesc.SampleDesc.Count = 4;
		depthStencilDesc.SampleDesc.Quality = msaaQuality-1;
	} else {
		depthStencilDesc.SampleDesc.Count = 1;
		depthStencilDesc.SampleDesc.Quality = 0;
	}
	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0;
	depthStencilDesc.MiscFlags = 0;
	HR( d3DDevice->CreateTexture2D( &depthStencilDesc, 0, &depthStencilBuffer ) );
	HR( d3DDevice->CreateDepthStencilView( depthStencilBuffer, 0, &depthStencilView ) );

	// bind the views to the output merger stage
	d3DImmediateContext->OMSetRenderTargets( 1, &renderTargetView, depthStencilView );

	D3D11_VIEWPORT vp;
	vp.TopLeftX = 0.f;
	vp.TopLeftY = 0.f;
	vp.Width = static_cast<float>(clientWidth);
	vp.Height = static_cast<float>(clientHeight);
	vp.MinDepth = 0.f;
	vp.MaxDepth = 1.f;
	d3DImmediateContext->RSSetViewports( 1, &vp );

	camera.Init( AspectRatio() );
}

LRESULT Game::MsgProc( HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam ) {
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch( msg ) {
	case WM_ACTIVATE:
		if( LOWORD( wParam )==WA_INACTIVE ) {
			paused = true;
			timer.Stop();
		} else {
			paused = false;
			timer.Start();
		}
		return 0;
	case WM_ENTERSIZEMOVE:
		paused = true;
		resizing = true;
		timer.Stop();
		return 0;
	case WM_EXITSIZEMOVE:
		paused = false;
		resizing = false;
		OnResize();
		timer.Start();
		return 0;
	case WM_SIZE:
		clientWidth = LOWORD( lParam );
		clientHeight = HIWORD( lParam );
		if( d3DDevice ) {
			if( wParam==SIZE_MINIMIZED ) {
				paused = true;
			} else if( wParam==SIZE_MAXIMIZED ) {
				OnResize();
			} else if( wParam==SIZE_RESTORED ) {
				// is resizing the user is dragging the resize bar
				// do not recreate swap chain
				if( !resizing ) {
					// api calls like SetWindowPos or swapChain->SetFullScreenState
					OnResize();
				}
			}
		}
		return 0;
	case WM_GETMINMAXINFO:
		((MINMAXINFO*)lParam)->ptMinTrackSize.x = 480;
		((MINMAXINFO*)lParam)->ptMinTrackSize.y = 320;
		return 0;
	case WM_MENUCHAR:
		// don't beep when we alt-enter to change fullscreen
		return MAKELRESULT( 0, MNC_CLOSE );
	case WM_DESTROY:
		PostQuitMessage( 0 );
		return 0;
	default:
		return DefWindowProc( hwnd, msg, wParam, lParam );
	}
	return 0;
}

int Game::Run() {
	MSG msg = { 0 };

	timer.Reset();

	while( msg.message!=WM_QUIT ) {
		// If there are Window messages then process them.
		if( PeekMessage( &msg, 0, 0, 0, PM_REMOVE ) ) {
			TranslateMessage( &msg );
			DispatchMessage( &msg );
		}
		// Otherwise, do animation/game stuff.
		else {
			timer.Tick();

			if( !paused ) {
#ifdef DISPLAY_FPS
				CalculateFrameStats();
#endif
				Update( timer.DT() );
				Draw();
			} else {
				Sleep( 100 );
			}
		}
	}

	return (int)msg.wParam;
}

void Game::DisplayWinError( LPTSTR lpszFunction ) {
	// Retrieve the system error message for the last-error code

	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER|
		FORMAT_MESSAGE_FROM_SYSTEM|
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID( LANG_NEUTRAL, SUBLANG_DEFAULT ),
		(LPTSTR)&lpMsgBuf,
		0, NULL );

	// Display the error message and exit the process

	lpDisplayBuf = (LPVOID)LocalAlloc( LMEM_ZEROINIT,
		(lstrlen( (LPCTSTR)lpMsgBuf )+lstrlen( (LPCTSTR)lpszFunction )+40) * sizeof( TCHAR ) );
	StringCchPrintf( (LPTSTR)lpDisplayBuf, LocalSize( lpDisplayBuf )/sizeof( TCHAR ), TEXT( "%s failed with error %d: %s" ), lpszFunction, dw, lpMsgBuf );
	MessageBox( NULL, (LPCTSTR)lpDisplayBuf, TEXT( "Error" ), MB_OK );

	LocalFree( lpMsgBuf );
	LocalFree( lpDisplayBuf );
	ExitProcess( dw );
}

void Game::CalculateFrameStats() {
	static int frameCount = 0;
	static float timeElapsed = 0.0f;
	frameCount++;
	// calculate frames over 1 second
	if( timer.TotalTime()-timeElapsed>=1.0f ) {
		float fps = (float)frameCount; // frameCount / 1
		float mspf = 1000.f/fps; //milliseconds per frame
		std::wostringstream outs;
		outs.precision( 4 );
		outs<<wndTitle<<L"     "<<L"FPS: "<<fps<<L"    Frame Time: "<<mspf<<L"ms";
		SetWindowText( hMainWnd, outs.str().c_str() );
		frameCount = 0;
		timeElapsed += 1;
	}
}

float Game::AspectRatio() {
	return static_cast<float>(clientWidth)/clientHeight;
}

bool Game::LoadGameObjects() {
	ModelLoader loader( d3DDevice, "./art/models/", "/art/textures/" );
	floor = Floor();
	if( !floor.Init( &loader, d3DDevice ) ) {
		fprintf( stderr, "Error initiating floor" );
		return false;
	}
}

void Game::Update( float dt ) {
	XMVECTOR pos = XMVectorSet( 50.f, 50.f, 0.f, 1.f );
	XMVECTOR dir = XMVectorSet( -1.f, -0.5f, 0.f, 0.f );
	camera.Update(pos, dir);
}

void Game::Draw() {
	XMMATRIX viewProj = camera.GetViewProj();
	float clearColor[4] = { 0.0f, 0.125f, 0.3f, 1.0f }; 
	d3DImmediateContext->ClearRenderTargetView( renderTargetView, clearColor );
	d3DImmediateContext->ClearDepthStencilView( depthStencilView, D3D11_CLEAR_DEPTH|D3D11_CLEAR_STENCIL, 1.0f, 0 );

	floor.Draw( viewProj, d3DImmediateContext );
	swapChain->Present( 0, 0 );
}