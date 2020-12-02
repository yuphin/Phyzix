#include "RenderTexture.h"

RenderTextureClass::RenderTextureClass() {
	render_target_tex = 0;
	render_target_view = 0;
	srv = 0;
}

RenderTextureClass::RenderTextureClass(const RenderTextureClass& other) {}

RenderTextureClass::~RenderTextureClass() {}

bool RenderTextureClass::init(ID3D11Device* device, int textureWidth, int textureHeight) {
	D3D11_TEXTURE2D_DESC textureDesc;
	HRESULT result;
	D3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc;
	D3D11_SHADER_RESOURCE_VIEW_DESC shaderResourceViewDesc;


	// Initialize the render target texture description.
	ZeroMemory(&textureDesc, sizeof(textureDesc));

	// Setup the render target texture description.
	textureDesc.Width = textureWidth;
	textureDesc.Height = textureHeight;
	textureDesc.MipLevels = 1;
	textureDesc.ArraySize = 1;
	textureDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
	textureDesc.SampleDesc.Count = 1;
	textureDesc.Usage = D3D11_USAGE_DEFAULT;
	textureDesc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
	textureDesc.CPUAccessFlags = 0;
	textureDesc.MiscFlags = 0;

	// Create the render target texture.
	result = device->CreateTexture2D(&textureDesc, NULL, &render_target_tex);
	if(FAILED(result)) {
		return false;
	}

	// Setup the description of the render target view.
	renderTargetViewDesc.Format = textureDesc.Format;
	renderTargetViewDesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
	renderTargetViewDesc.Texture2D.MipSlice = 0;

	// Create the render target view.
	result = device->CreateRenderTargetView(render_target_tex, &renderTargetViewDesc, &render_target_view);
	if(FAILED(result)) {
		return false;
	}

	// Setup the description of the shader resource view.
	shaderResourceViewDesc.Format = textureDesc.Format;
	shaderResourceViewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	shaderResourceViewDesc.Texture2D.MostDetailedMip = 0;
	shaderResourceViewDesc.Texture2D.MipLevels = 1;

	// Create the shader resource view.
	result = device->CreateShaderResourceView(render_target_tex, &shaderResourceViewDesc, &srv);
	if(FAILED(result)) {
		return false;
	}

	return true;
}

void RenderTextureClass::shutdown() {
	if(srv) {
		srv->Release();
		srv = 0;
	}

	if(render_target_view) {
		render_target_view->Release();
		render_target_view = 0;
	}

	if(render_target_tex) {
		render_target_tex->Release();
		render_target_tex = 0;
	}

	return;
}

void RenderTextureClass::set_render_target(ID3D11DeviceContext* deviceContext, ID3D11DepthStencilView* depthStencilView) {
	// Bind the render target view and depth stencil buffer to the output render pipeline.
	deviceContext->OMSetRenderTargets(1, &render_target_view, depthStencilView);

	return;
}

void RenderTextureClass::clear_render_target(ID3D11DeviceContext* deviceContext, ID3D11DepthStencilView* depthStencilView,
											 float red, float green, float blue, float alpha) {
	float color[4];


	// Setup the color to clear the buffer to.
	color[0] = red;
	color[1] = green;
	color[2] = blue;
	color[3] = alpha;

	// Clear the back buffer.
	deviceContext->ClearRenderTargetView(render_target_view, color);

	// Clear the depth buffer.
	deviceContext->ClearDepthStencilView(depthStencilView, D3D11_CLEAR_DEPTH, 1.0f, 0);

	return;
}

ID3D11ShaderResourceView* RenderTextureClass::get_srv() {
	return srv;
}