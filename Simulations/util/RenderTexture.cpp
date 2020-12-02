#include "RenderTexture.h"

Texture::Texture() {
	tex = 0;
	render_target_view = 0;
	srv = 0;
}

Texture::Texture(const Texture& other) {}

Texture::~Texture() {}

bool Texture::init(ID3D11Device* device, int width, int height,
				   bool is_render_texture, DXGI_FORMAT format,
				   const void* data,
				   UINT row_pitch,
				   ID3D11DeviceContext* context,
				   D3D11_USAGE usage, UINT bind_flags) {
	D3D11_TEXTURE2D_DESC texture_desc;
	HRESULT result;
	D3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc;
	D3D11_SHADER_RESOURCE_VIEW_DESC srv_desc;


	// Initialize the render target texture description.
	ZeroMemory(&texture_desc, sizeof(texture_desc));

	// Setup the render target texture description.
	texture_desc.Width = width;
	texture_desc.Height = height;
	texture_desc.MipLevels = 1;
	texture_desc.ArraySize = 1;
	texture_desc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
	texture_desc.SampleDesc.Count = 1;
	texture_desc.Usage = D3D11_USAGE_DEFAULT;
	texture_desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
	texture_desc.CPUAccessFlags = 0;
	texture_desc.MiscFlags = 0;

	// Create the render target texture.
	result = device->CreateTexture2D(&texture_desc, NULL, &tex);
	if(FAILED(result)) {
		return false;
	}
	if(is_render_texture) {
		// Setup the description of the render target view.
		renderTargetViewDesc.Format = texture_desc.Format;
		renderTargetViewDesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
		renderTargetViewDesc.Texture2D.MipSlice = 0;

		// Create the render target view.
		result = device->CreateRenderTargetView(tex, &renderTargetViewDesc, &render_target_view);
		if(FAILED(result)) {
			return false;
		}
	}
	if(!data) {
		context->UpdateSubresource(tex, 0, nullptr, data, row_pitch, 0);
	}

	// Setup the description of the shader resource view.
	srv_desc.Format = texture_desc.Format;
	srv_desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	srv_desc.Texture2D.MostDetailedMip = 0;
	srv_desc.Texture2D.MipLevels = 1;

	// Create the shader resource view.
	result = device->CreateShaderResourceView(tex, &srv_desc, &srv);
	if(FAILED(result)) {
		return false;
	}

	return true;
}

void Texture::shutdown() {
	if(srv) {
		srv->Release();
		srv = 0;
	}

	if(render_target_view) {
		render_target_view->Release();
		render_target_view = 0;
	}

	if(tex) {
		tex->Release();
		tex = 0;
	}
}

void Texture::set_render_target(ID3D11DeviceContext* deviceContext, ID3D11DepthStencilView* depthStencilView) {
	// Bind the render target view and depth stencil buffer to the output render pipeline.
	deviceContext->OMSetRenderTargets(1, &render_target_view, depthStencilView);

	return;
}

void Texture::clear_render_target(ID3D11DeviceContext* deviceContext, ID3D11DepthStencilView* depthStencilView,
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

ID3D11ShaderResourceView* Texture::get_srv() {
	return srv;
}