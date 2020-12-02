#ifndef _RENDERTEXTURECLASS_H_
#define _RENDERTEXTURECLASS_H_

#include <d3d11.h>

class RenderTextureClass
{
public:
	RenderTextureClass();
	RenderTextureClass(const RenderTextureClass&);
	~RenderTextureClass();

	bool init(ID3D11Device*, int, int);
	void shutdown();

	void set_render_target(ID3D11DeviceContext*, ID3D11DepthStencilView*);
	void clear_render_target(ID3D11DeviceContext*, ID3D11DepthStencilView*, float, float, float, float);
	ID3D11ShaderResourceView* get_srv();

private:
	ID3D11Texture2D* render_target_tex;
	ID3D11RenderTargetView* render_target_view;
	ID3D11ShaderResourceView* srv;
};

#endif