#include "PathTracer.h"

PathTracer::PathTracer(ID3D11Device* device, ID3D11DeviceContext* context,
					   CModelViewerCamera* camera) :
	device(device), context(context), camera(camera), output_tex(2) {
	dsv = DXUTGetD3D11DepthStencilView();
	rtv = DXUTGetD3D11RenderTargetView();
}

void PathTracer::init() {
	HRESULT hr = S_OK;
	// Create shaders
	ID3DBlob* err_blob = nullptr;
	ID3DBlob* vs_blob = nullptr;

	hr = D3DCompileFromFile(L"path_trace.hlsl", NULL, NULL, "VS", "vs_5_0", 
					   0, 0, &vs_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load vertex shader");
	}
	ID3DBlob* pt_blob = nullptr;
	ID3DBlob* acc_blob = nullptr;
	ID3DBlob* tile_out_blob = nullptr;
	ID3DBlob* out_blob = nullptr;

	hr = D3DCompileFromFile(L"path_trace.hlsl", NULL, NULL, "PS", "ps_5_0", 
					   0, 0, &pt_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load path trace shader");
	}
	hr = D3DCompileFromFile(L"accumulation.hlsl", NULL, NULL, "PS", "ps_5_0",
							0, 0, &acc_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load accumulation shader");
	}
	hr = D3DCompileFromFile(L"tile_output.hlsl", NULL, NULL, "PS", "ps_5_0",
							0, 0, &tile_out_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load merging shader");
	}
	hr = D3DCompileFromFile(L"output.hlsl", NULL, NULL, "PS", "ps_5_0",
							0, 0, &out_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load final pass shader");
	}

	device->CreateVertexShader(vs_blob->GetBufferPointer(), 
							   vs_blob->GetBufferSize(), NULL, &vertex_shader);
	device->CreatePixelShader(pt_blob->GetBufferPointer(),
							  pt_blob->GetBufferSize(), NULL, &pt_shader);
	device->CreatePixelShader(acc_blob->GetBufferPointer(),
							  acc_blob->GetBufferSize(), NULL, &acc_shader);
	device->CreatePixelShader(tile_out_blob->GetBufferPointer(),
							  tile_out_blob->GetBufferSize(), NULL, &tile_output_shader);
	device->CreatePixelShader(out_blob->GetBufferPointer(),
							  out_blob->GetBufferSize(), NULL, &output_shader);
	// Create textures
	path_trace_tex.init(device, tile_width, tile_height);
	path_trace_tmp_tex.init(device, tile_width * pixel_ratio,
									   tile_width * pixel_ratio);
	accum_tex.init(device, screen_size.x, screen_size.y);
	for(auto& tex : output_tex) {
		tex.init(device, screen_size.x, screen_size.y);
	}
	// Create samplers
	create_sampler(device, &pt_sampler);
	create_sampler(device, &pt_sampler_lowres, D3D11_FILTER_MIN_MAG_MIP_POINT, D3D11_TEXTURE_ADDRESS_CLAMP);
}



void PathTracer::render() {
	// Render a tile
	path_trace_tex.set_render_target(context, dsv);
	set_viewport(0, 0, tile_width, tile_height);
	draw_quad(vertex_shader.Get(), pt_shader.Get());
	// Copy this tile to correct offset 
	accum_tex.set_render_target(context, dsv);
	set_viewport(tile_width * tile_x, tile_height * tile_y, tile_width, tile_height);
	context->PSSetSamplers(0, 1, &pt_sampler);
	auto srv = path_trace_tex.get_srv();
	context->PSSetShaderResources(0, 1, &srv);
	clear_render_target();
	draw_quad(vertex_shader.Get(), acc_shader.Get());
	// Move this tile to original viewport
	output_tex[output_idx].set_render_target(context, dsv);
	set_viewport(0, 0, screen_size.x, screen_size.y);
	context->PSSetSamplers(0, 1, &pt_sampler);
	srv = accum_tex.get_srv();
	context->PSSetShaderResources(0, 1, &srv);
	draw_quad(vertex_shader.Get(), tile_output_shader.Get());
	restore_back_buffer_render_target();
	clear_render_target();
}

void PathTracer::present() {
	// For PostFX
	context->PSSetSamplers(0, 1, &pt_sampler);
	auto srv = output_tex[output_idx].get_srv();
	context->PSSetShaderResources(0, 1, &srv);
	draw_quad(vertex_shader.Get(), output_shader.Get());
}

void PathTracer::update() {
	tile_x++;
	if(tile_x >= num_tiles_x) {
		tile_x = 0;
		tile_y++;
		if(tile_y >= num_tiles_y) {
			tile_x = 0;
			tile_y = 0;
			sample_count++;
			output_idx ^= 1;
		}
	}
}

void PathTracer::draw_quad(ID3D11VertexShader* vs, ID3D11PixelShader* ps) {
	context->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	context->VSSetShader(vs, nullptr, 0);
	context->PSSetShader(ps, nullptr, 0);
	context->Draw(4, 0);
}

void PathTracer::restore_back_buffer_render_target() {
	context->OMSetRenderTargets(1, &rtv, dsv);
}

void PathTracer::clear_render_target() {
	// Clear render target and depth stencil
	float ClearColor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	context->ClearRenderTargetView(rtv, ClearColor);
	context->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH, 1.0f, 0);
}

void PathTracer::unbind_ps() {
	ID3D11ShaderResourceView* null_srv[1] = { nullptr };
	ID3D11Buffer* null_cb[1] = { nullptr };
	ID3D11SamplerState* null_ss[1] = { nullptr };

	context->PSSetShaderResources(0, 1, null_srv);
	context->PSSetConstantBuffers(0, 1, null_cb);
	context->PSSetSamplers(0, 1, null_ss);
}

void PathTracer::set_viewport(int x, int y, int width, int height) {
	D3D11_VIEWPORT viewport = {x, y, width, height, 0, 1};
	context->RSSetViewports(1, &viewport);
}

PathTracer::~PathTracer() {
	path_trace_tex.shutdown();
	path_trace_tmp_tex.shutdown();
	accum_tex.shutdown();
	for(auto& tex : output_tex) {
		tex.shutdown();
	}
	if(pt_sampler) pt_sampler->Release();
	if(pt_sampler_lowres) pt_sampler_lowres->Release();
}
