#pragma once
#include "DrawingUtilitiesClass.h"
#include "util/RenderTexture.h"
#include "util/vector2d.h"
#include "util/util.h"
#include <wrl/client.h>
#include "Scene.h"
#define eprintf(...) fprintf(stderr, __VA_ARGS__);
using namespace Microsoft::WRL;
class PathTracer {
public:
	PathTracer(ID3D11Device* device, ID3D11DeviceContext* context,
			   CModelViewerCamera* camera, Scene* scene);
	void init();
	void present();
	void render();
	void update();
	~PathTracer();
private:
	void draw_quad(ID3D11VertexShader* vs, ID3D11PixelShader* ps);
	void restore_back_buffer_render_target();
	void clear_render_target();
	void unbind_ps();
	void set_viewport(int x, int y, int width, int height);
	ID3D11Device* device;
	ID3D11DeviceContext* context;

	CModelViewerCamera* camera;
	Scene* scene;

	ComPtr<ID3D11VertexShader> vertex_shader;
	ComPtr<ID3D11PixelShader> pt_shader;
	ComPtr<ID3D11PixelShader> acc_shader;
	ComPtr<ID3D11PixelShader> tile_output_shader;
	ComPtr<ID3D11PixelShader> output_shader;
	Texture path_trace_tex;
	Texture path_trace_tmp_tex;
	Texture accum_tex;
	Texture bvh_tex;
	Texture bb_min_tex;
	Texture bb_max_tex;
	Texture idx_tex;
	Texture vert_tex;
	Texture normals_tex;
	Texture mat_tex;
	Texture transforms_tex;
	Texture lights_tex;


	std::vector<Texture> output_tex;
	ID3D11DepthStencilView* dsv;
	ID3D11RenderTargetView* rtv;
	ID3D11SamplerState* pt_sampler;
	ID3D11SamplerState* pt_sampler_lowres;
	Vec2i screen_size = { 1200, 700 };
	const int tile_width = 100;
	const int tile_height = 100;
	const float pixel_ratio = 0.25f;
	int num_tiles_x = screen_size.x / tile_width;
	int num_tiles_y = screen_size.y / tile_width;
	int tile_x = -1;
	int tile_y = 0;
	int num_lights = 0;
	uint32_t sample_count = 0;
	uint8_t output_idx = 0;
};

