#include "PathTracer.h"
#include "util/vector4d.h"
PathTracer::PathTracer(ID3D11Device* device, ID3D11DeviceContext* context,
					   CModelViewerCamera* camera, Scene* scene) :
	device(device), context(context), camera(camera), output_tex(2),
	scene(scene) {
	dsv = DXUTGetD3D11DepthStencilView();
	rtv = DXUTGetD3D11RenderTargetView();
}

void PathTracer::init() {
	HRESULT hr = S_OK;
	// Create shaders
	ID3DBlob* err_blob = nullptr;
	ID3DBlob* vs_blob = nullptr;

	hr = D3DCompileFromFile(L"quad.hlsl", NULL, NULL, "VS", "vs_5_0",
							D3DCOMPILE_DEBUG, D3DCOMPILE_SKIP_OPTIMIZATION, &vs_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load vertex shader");
	}
	ID3DBlob* pt_blob = nullptr;
	ID3DBlob* acc_blob = nullptr;
	ID3DBlob* tile_out_blob = nullptr;
	ID3DBlob* out_blob = nullptr;
#if 1
	hr = D3DCompileFromFile(L"path_trace.hlsl", NULL, NULL, "PS", "ps_5_0",
							D3DCOMPILE_DEBUG, D3DCOMPILE_SKIP_OPTIMIZATION, &pt_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load path trace shader %s\n", err_blob->GetBufferPointer());
	}
	hr = D3DCompileFromFile(L"accumulation.hlsl", NULL, NULL, "PS", "ps_5_0",
							D3DCOMPILE_SKIP_OPTIMIZATION, D3DCOMPILE_DEBUG, &acc_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load accumulation shader");
	}
	hr = D3DCompileFromFile(L"tile_output.hlsl", NULL, NULL, "PS", "ps_5_0",
							D3DCOMPILE_DEBUG, D3DCOMPILE_SKIP_OPTIMIZATION, &tile_out_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load merging shader");
	}
	hr = D3DCompileFromFile(L"output.hlsl", NULL, NULL, "PS", "ps_5_0",
							D3DCOMPILE_SKIP_OPTIMIZATION, D3DCOMPILE_DEBUG, &out_blob, &err_blob);
	if(FAILED(hr) && err_blob) {
		eprintf("Failed to load final pass shader");
	}
#else
	hr = D3DCompileFromFile(L"path_trace.hlsl", NULL, NULL, "PS", "ps_5_0",
		0, 0, &pt_blob, &err_blob);
	if (FAILED(hr) && err_blob) {
		eprintf("Failed to load path trace shader %s\n", err_blob->GetBufferPointer());
	}
	hr = D3DCompileFromFile(L"accumulation.hlsl", NULL, NULL, "PS", "ps_5_0",
		0, 0, &acc_blob, &err_blob);
	if (FAILED(hr) && err_blob) {
		eprintf("Failed to load accumulation shader");
	}
	hr = D3DCompileFromFile(L"tile_output.hlsl", NULL, NULL, "PS", "ps_5_0",
		0, 0, &tile_out_blob, &err_blob);
	if (FAILED(hr) && err_blob) {
		eprintf("Failed to load merging shader");
	}
	hr = D3DCompileFromFile(L"output.hlsl", NULL, NULL, "PS", "ps_5_0",
		0, 0, &out_blob, &err_blob);
	if (FAILED(hr) && err_blob) {
		eprintf("Failed to load final pass shader");
	}
#endif

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
	// Create textures for rendering
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

	// Create texture for BVH
	int row_pitch = sizeof(RadeonRays::BvhTranslator::Node) * scene->bvhTranslator.nodes.size()
		/ (scene->bvhTranslator.node_tex_width * 3 );
	bvh_tex.init(device, scene->bvhTranslator.node_tex_width, scene->bvhTranslator.node_tex_width,
				 false, DXGI_FORMAT_R32G32B32_SINT, scene->bvhTranslator.nodes.data(),
				 scene->bvhTranslator.node_tex_width * 3 * sizeof(int),
				 context);

	// Create textures for bounding box
	row_pitch = sizeof(float) * 3 * scene->bvhTranslator.bboxmin.size() /
		(scene->bvhTranslator.node_tex_width * 3);
	bb_min_tex.init(device, scene->bvhTranslator.node_tex_width,
					scene->bvhTranslator.node_tex_width,
					false, DXGI_FORMAT_R32G32B32_FLOAT,
					scene->bvhTranslator.bboxmin.data(),
					scene->bvhTranslator.node_tex_width * 3 * sizeof(float),
					context);
	row_pitch = sizeof(float) * 3 * scene->bvhTranslator.bboxmax.size() /
		(scene->bvhTranslator.node_tex_width * 3 );
	bb_max_tex.init(device, scene->bvhTranslator.node_tex_width,
					scene->bvhTranslator.node_tex_width,
					false, DXGI_FORMAT_R32G32B32_FLOAT,
					scene->bvhTranslator.bboxmax.data(),
					scene->bvhTranslator.node_tex_width * 3 * sizeof(float),
					context);
	// Create texture for indices
	row_pitch = sizeof(Indices) * scene->vert_indices.size() /
		(scene->indices_tex_width * 3 );
	idx_tex.init(device, scene->indices_tex_width, scene->indices_tex_width, false,
				 DXGI_FORMAT_R32G32B32_SINT, scene->vert_indices.data(),
				 scene->indices_tex_width * 3 * sizeof(float),
				 context);
	// Create texture for vertices
	row_pitch = sizeof(float) * 4 * scene->vertices_uvx.size() /
		(scene->vert_tex_width * 4 );
	auto t = sizeof(nVec4f);
	vert_tex.init(device, scene->vert_tex_width, scene->vert_tex_width, false,
				  DXGI_FORMAT_R32G32B32A32_FLOAT, scene->vertices_uvx.data(),
				  scene->vert_tex_width * 4 * sizeof(float),
				  context);
	// Create texture for normals
	row_pitch = sizeof(float) * 4 * scene->normals_uvy.size() /
		(scene->vert_tex_width * 4 );
	normals_tex.init(device, scene->vert_tex_width, scene->vert_tex_width, false,
					 DXGI_FORMAT_R32G32B32A32_FLOAT, scene->normals_uvy.data(),
					 scene->vert_tex_width * 4 * sizeof(float),
					 context);
	// Create texture for materials
	// Material struct is 16 floats
	auto mat_tex_size = (sizeof(Material) / sizeof(float) * 4) * scene->materials.size();
	// This is just a row texture
	row_pitch = sizeof(float) * 16 * scene->materials.size() /
		(mat_tex_size * 4 );
	mat_tex.init(device, mat_tex_size, 1, false,
					 DXGI_FORMAT_R32G32B32A32_FLOAT, scene->materials.data(),
				 mat_tex_size * 4 * sizeof(float),
					 context);
	// Create texture for transforms
	// Row texture
	auto transforms_tex_size = 4 * scene->transforms.size();
	row_pitch = sizeof(float) * 16 * scene->transforms.size() /
		(transforms_tex_size * 4 );
	transforms_tex.init(device, transforms_tex_size, 1, false,
						DXGI_FORMAT_R32G32B32A32_FLOAT, scene->transforms.data(),
						transforms_tex_size * 4 * sizeof(float),
						context);
	num_lights = scene->lights.size();
	if(num_lights > 0) {
		auto light_tex_size = (sizeof(Light) / sizeof(Vec3f)) * scene->lights.size();
		row_pitch =  sizeof(Light)* scene->lights.size() /
			(light_tex_size * 3 );
		lights_tex.init(device, light_tex_size, 1, false,
						DXGI_FORMAT_R32G32B32_FLOAT,
						scene->lights.data(), 
						light_tex_size * 3 * sizeof(float),
						context);
	}

	// Create constant buffer for PT
	CD3D11_BUFFER_DESC cb_desc_pt(
		sizeof(PathTraceCB),
		D3D11_BIND_CONSTANT_BUFFER
	);
	CD3D11_BUFFER_DESC cb_desc_frame(
		sizeof(FrameCB),
		D3D11_BIND_CONSTANT_BUFFER
	);
	ptcb.bvh_root_idx = scene->bvhTranslator.top_level_idx_packed_xy;
	ptcb.idx_buf_tex_size = scene->indices_tex_width;
	ptcb.num_lights = num_lights;
	auto screen_size_vec = screen_size.to_dx_vector();
	auto inv_num_tiles_vec = Vec2(1.0f / num_tiles_x, 1.0f / num_tiles_y).to_dx_vector();
	XMStoreFloat2(&ptcb.resolution, screen_size_vec);
	XMStoreFloat2(&ptcb.inv_num_tiles, inv_num_tiles_vec);
	D3D11_SUBRESOURCE_DATA init_data;
	init_data.pSysMem = &ptcb;
	init_data.SysMemPitch = 0;
	init_data.SysMemSlicePitch = 0;
	hr = device->CreateBuffer(
		&cb_desc_frame,
		nullptr,
		frame_cb.GetAddressOf());
	if(FAILED(hr)) {
		eprintf("Failed to create frame CB %x\n", hr);
	}
	hr = device->CreateBuffer(
		&cb_desc_pt,
		&init_data,
		path_tracer_cb.GetAddressOf());
	if(FAILED(hr)) {
		eprintf("Failed to create PT CB %x\n",hr);
	}

	
}

void PathTracer::render() {
	ID3D11ShaderResourceView* const pSRV[1] = { NULL };
	// Render a tile
	path_trace_tex.set_render_target(context, dsv);
	set_viewport(0, 0, tile_width, tile_height);
	context->PSSetSamplers(0, 1, &pt_sampler_lowres);
	context->PSSetSamplers(1, 1, &pt_sampler);
	auto srv = accum_tex.get_srv();
	context->PSSetShaderResources(0, 1, &srv);

	// Bind scene related resources
	bind_scene_resources();
	
	draw_quad(vertex_shader.Get(), pt_shader.Get());
	context->PSSetShaderResources(0, 1, pSRV);
	//ID3D11RenderTargetView* nullRTV = nullptr; context->OMSetRenderTargets(1, &nullRTV, nullptr);

	// Copy this tile to correct offset 
	accum_tex.set_render_target(context, dsv);
	set_viewport(tile_width * tile_x, tile_height * tile_y, tile_width, tile_height);
	context->PSSetSamplers(0, 1, &pt_sampler);
	srv = path_trace_tex.get_srv();
	context->PSSetShaderResources(0, 1, &srv);
	clear_render_target();
	draw_quad(vertex_shader.Get(), acc_shader.Get());
	context->PSSetShaderResources(0, 1, pSRV);

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
	float v1, v2, v3;
	// TODO: Change sampling scheme
	v1 = ((float) rand() / (RAND_MAX));
	v2 = ((float) rand() / (RAND_MAX));
	v3 = ((float) rand() / (RAND_MAX));

	frcb.max_depth = scene->options.max_depth;
	frcb.fov = scene->camera->fov;
	auto rand_vec = XMVectorSet(v1, v2, v3, 1);
	XMStoreFloat3(&frcb.rand_vec, rand_vec);
	frcb.tile_x = tile_x;
	frcb.tile_y = tile_y;
	auto cam_right_v = scene->camera->right.toDirectXVector();
	auto cam_up_v = scene->camera->up.toDirectXVector();
	auto cam_forward_v = scene->camera->forward.toDirectXVector();
	auto cam_pos_v = scene->camera->pos.toDirectXVector();
	XMStoreFloat4(&frcb.camera.pos, cam_pos_v);
	XMStoreFloat4(&frcb.camera.right, cam_right_v);
	XMStoreFloat4(&frcb.camera.up, cam_up_v);
	XMStoreFloat4(&frcb.camera.forward, cam_forward_v);
	context->UpdateSubresource(
		frame_cb.Get(),
		0,
		nullptr,
		&frcb,
		0,
		0
	);
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

void PathTracer::bind_scene_resources() {
	auto srv = bvh_tex.get_srv();
	context->PSSetShaderResources(1, 1, &srv);
	srv = bb_min_tex.get_srv();
	context->PSSetShaderResources(2, 1, &srv);
	srv = bb_max_tex.get_srv();
	context->PSSetShaderResources(3, 1, &srv);
	srv = idx_tex.get_srv();
	context->PSSetShaderResources(4, 1, &srv);
	srv = vert_tex.get_srv();
	context->PSSetShaderResources(5, 1, &srv);
	srv = normals_tex.get_srv();
	context->PSSetShaderResources(6, 1, &srv);
	srv = mat_tex.get_srv();
	context->PSSetShaderResources(7, 1, &srv);
	srv = transforms_tex.get_srv();
	context->PSSetShaderResources(8, 1, &srv);
	srv = lights_tex.get_srv();
	context->PSSetShaderResources(9, 1, &srv);

	// Bind constant buffers
	context->PSSetConstantBuffers(0, 1, path_tracer_cb.GetAddressOf());
	context->PSSetConstantBuffers(1, 1, frame_cb.GetAddressOf());
}

void PathTracer::set_viewport(int x, int y, int width, int height) {
	D3D11_VIEWPORT viewport = { x, y, width, height, 0, 1 };
	context->RSSetViewports(1, &viewport);
}

PathTracer::~PathTracer() {
	path_trace_tex.shutdown();
	path_trace_tmp_tex.shutdown();
	accum_tex.shutdown();
	bvh_tex.shutdown();
	bb_min_tex.shutdown();
	bb_max_tex.shutdown();
	idx_tex.shutdown();
	vert_tex.shutdown();
	normals_tex.shutdown();
	mat_tex.shutdown();
	transforms_tex.shutdown();
	lights_tex.shutdown();
	for(auto& tex : output_tex) {
		tex.shutdown();
	}
	if(pt_sampler) pt_sampler->Release();
	if(pt_sampler_lowres) pt_sampler_lowres->Release();
}
