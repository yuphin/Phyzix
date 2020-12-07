#include "RenderTexture.h"
#include <stdio.h>
Texture::Texture() {
	tex = 0;
	render_target_view = 0;
	srv = 0;
}

Texture::Texture(const Texture& other) {}

Texture::~Texture() {}

bool Texture::init(ID3D11Device* device, int width, int height,
				   bool is_render_texture, DXGI_FORMAT format,
				   void* data,
				   UINT row_pitch,
				   ID3D11DeviceContext* context,
				   D3D11_USAGE usage, UINT bind_flags) {
	D3D11_TEXTURE2D_DESC texture_desc = {};
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
	texture_desc.Format = format;
	texture_desc.SampleDesc.Count = 1;
	texture_desc.SampleDesc.Quality = 0;
	texture_desc.Usage = D3D11_USAGE_DEFAULT;
	texture_desc.BindFlags = is_render_texture ? bind_flags | D3D11_BIND_RENDER_TARGET
		: bind_flags;
	texture_desc.CPUAccessFlags = 0;
	texture_desc.MiscFlags = 0;

	// Create the render target texture.
	ID3D11Texture2D* staging_tex;
	if(data) {
		D3D11_TEXTURE2D_DESC staging_desc = {};
		staging_desc.Width = width;
		staging_desc.Height = height;
		staging_desc.MipLevels = 1;
		staging_desc.ArraySize = 1;
		staging_desc.Format = format;
		staging_desc.SampleDesc.Count = 1;
		staging_desc.Usage = D3D11_USAGE_STAGING;
		staging_desc.BindFlags = 0;
		staging_desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		staging_desc.MiscFlags = 0;
		result = device->CreateTexture2D(&staging_desc, NULL, &staging_tex);
		//context->UpdateSubresource(tex, 0, nullptr, data, row_pitch, 0);
		if(FAILED(result)) {
			printf("Failed to create a staging texture\n");
			return false;
		}
		D3D11_MAPPED_SUBRESOURCE mapped_resource;
		ZeroMemory(&mapped_resource, sizeof(D3D11_MAPPED_SUBRESOURCE));
		context->Map(staging_tex, 0, D3D11_MAP_WRITE, 0, &mapped_resource);
		BYTE* mapped_data = reinterpret_cast<BYTE*>(mapped_resource.pData);
		for(int i = 0; i < height; i++) {
			memcpy(mapped_data, data, row_pitch);
			mapped_data += mapped_resource.RowPitch;
			data = (char*) data + row_pitch;
		}
		context->Unmap(staging_tex, 0);
		result = device->CreateTexture2D(&texture_desc, NULL, &tex);
		if(FAILED(result)) {
			printf("Failed to create texture\n");
			return false;
		}
		context->CopyResource(tex, staging_tex);
		staging_tex->Release();
	} else {
		result = device->CreateTexture2D(&texture_desc, NULL, &tex);
		if(FAILED(result)) {
			printf("Failed to create texture\n");
			return false;
		}
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
	//
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