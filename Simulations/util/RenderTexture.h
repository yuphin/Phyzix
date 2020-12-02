#ifndef _RENDERTEXTURECLASS_H_
#define _RENDERTEXTURECLASS_H_

#include <d3d11.h>

class Texture
{
public:
	Texture();
	Texture(const Texture&);
	~Texture();

	bool init(ID3D11Device* device, int width, int height,
			  bool is_render_texture = true, 
			  DXGI_FORMAT format = DXGI_FORMAT_R32G32B32A32_FLOAT,
			  const void* data = nullptr,
			  UINT row_pitch = 0,
			  ID3D11DeviceContext* context = nullptr,
			  D3D11_USAGE usage = D3D11_USAGE_DEFAULT, 
			  UINT bind_flags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE);
	void shutdown();

	void set_render_target(ID3D11DeviceContext*, ID3D11DepthStencilView*);
	void clear_render_target(ID3D11DeviceContext*, ID3D11DepthStencilView*, float, float, float, float);
	ID3D11ShaderResourceView* get_srv();

private:
	ID3D11Texture2D* tex;
	ID3D11RenderTargetView* render_target_view;
	ID3D11ShaderResourceView* srv;
};

#endif