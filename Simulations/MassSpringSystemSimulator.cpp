#include "MassSpringSystemSimulator.h"
#include <d3dcompiler.h>
#pragma comment(lib,"D3dcompiler.lib")

// @yuphin : Refactor this mess
// @yuphin : Switching between CS and CPU implementation is crashing the simulator, fix

static HRESULT create_structured_buffer(ID3D11Device* device, 
                                      UINT element_size, 
                                      UINT count, 
                                      void* init_data, ID3D11Buffer** buf_ptr) {
    *buf_ptr = nullptr;
    D3D11_BUFFER_DESC desc = {};
    desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    desc.ByteWidth = element_size * count;
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = element_size;

    if(init_data) {
        D3D11_SUBRESOURCE_DATA s_init_data;
        s_init_data.pSysMem = init_data;
        return device->CreateBuffer(&desc, &s_init_data, buf_ptr);
    } else
        return device->CreateBuffer(&desc, nullptr, buf_ptr);
}

static HRESULT create_raw_buffer(ID3D11Device* device,
                                        UINT element_size,
                                        UINT count,
                                        void* init_data, ID3D11Buffer** buf_ptr) {
    *buf_ptr = nullptr;
    D3D11_BUFFER_DESC desc = {};
    desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_VERTEX_BUFFER;
    desc.ByteWidth = element_size * count;
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
    desc.StructureByteStride = element_size;

    if(init_data) {
        D3D11_SUBRESOURCE_DATA s_init_data;
        s_init_data.pSysMem = init_data;
        return device->CreateBuffer(&desc, &s_init_data, buf_ptr);
    } else
        return device->CreateBuffer(&desc, nullptr, buf_ptr);
}


static ID3D11Buffer* CreateAndCopyToDebugBuf(ID3D11Device* pDevice, ID3D11DeviceContext* pd3dImmediateContext, ID3D11Buffer* pBuffer) {
    ID3D11Buffer* debugbuf = nullptr;

    D3D11_BUFFER_DESC desc = {};
    pBuffer->GetDesc(&desc);
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = 0;
    desc.MiscFlags = 0;
    if(SUCCEEDED(pDevice->CreateBuffer(&desc, nullptr, &debugbuf))) {
#if defined(_DEBUG) || defined(PROFILE)
        debugbuf->SetPrivateData(WKPDID_D3DDebugObjectName, sizeof("Debug") - 1, "Debug");
#endif

        pd3dImmediateContext->CopyResource(debugbuf, pBuffer);
    }

    return debugbuf;
}

static HRESULT create_srv(ID3D11Device* device, 
                               ID3D11Buffer* buffer, 
                               ID3D11ShaderResourceView** srv_ptr) {
    D3D11_BUFFER_DESC desc_buf = {};
    buffer->GetDesc(&desc_buf);

    D3D11_SHADER_RESOURCE_VIEW_DESC desc = {};
    desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
    desc.BufferEx.FirstElement = 0;

    if(desc_buf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS) {
        // This is a Raw Buffer

        desc.Format = DXGI_FORMAT_R32_TYPELESS;
        desc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
        desc.BufferEx.NumElements = desc_buf.ByteWidth / 4;
    } else
        if(desc_buf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_STRUCTURED) {
            // This is a Structured Buffer

            desc.Format = DXGI_FORMAT_UNKNOWN;
            desc.BufferEx.NumElements = desc_buf.ByteWidth / desc_buf.StructureByteStride;
        } else {
            return E_INVALIDARG;
        }

    return device->CreateShaderResourceView(buffer, &desc, srv_ptr);
}

HRESULT create_uav(ID3D11Device* device, 
                   ID3D11Buffer* buffer, 
                   ID3D11UnorderedAccessView** uav_ptr) {
    D3D11_BUFFER_DESC desc_buf = {};
    buffer->GetDesc(&desc_buf);

    D3D11_UNORDERED_ACCESS_VIEW_DESC desc = {};
    desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = 0;

    if(desc_buf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS) {
        // This is a Raw Buffer

        desc.Format = DXGI_FORMAT_R32_TYPELESS; // Format must be DXGI_FORMAT_R32_TYPELESS, when creating Raw Unordered Access View
        desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
        desc.Buffer.NumElements = desc_buf.ByteWidth / 4;
    } else
        if(desc_buf.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_STRUCTURED) {
            // This is a Structured Buffer

            desc.Format = DXGI_FORMAT_UNKNOWN;      // Format must be must be DXGI_FORMAT_UNKNOWN, when creating a View of a Structured Buffer
            desc.Buffer.NumElements = desc_buf.ByteWidth / desc_buf.StructureByteStride;
        } else {
            return E_INVALIDARG;
        }
    return device->CreateUnorderedAccessView(buffer, &desc, uav_ptr);
}



MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_iTestCase = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 1, Demo 2, Demo 3, Demo 4 CS, Demo 4 CPU, Demo 5";
}
 
void MassSpringSystemSimulator::reset() {
    mouse.x = mouse.y = 0;
    trackmouse.x = trackmouse.y = 0;
    old_trackmouse.x = old_trackmouse.y = 0;
    external_force = 0;
    mouse_force = 0;
    springs.clear();
    mass_points.clear();

    initScene();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* context) {

    uint32_t stride = sizeof(MassPointVertex);
    uint32_t offset = 0;
   
       
    switch(m_iTestCase) {
        case 3 : 
        case 4 :
        {
            D3D11_RASTERIZER_DESC grid_state = {};
            grid_state.AntialiasedLineEnable = false;
            grid_state.CullMode = D3D11_CULL_NONE;
            grid_state.DepthBias = 0;
            grid_state.DepthBiasClamp = 0.0f;
            grid_state.DepthClipEnable = true;
            grid_state.FillMode = D3D11_FILL_SOLID;
            grid_state.FrontCounterClockwise = false;
            grid_state.MultisampleEnable = false;
            grid_state.ScissorEnable = false;
            grid_state.SlopeScaledDepthBias = 0.0f;
            context->RSGetState(&rasterizer_old);
            DUC->g_ppd3Device->CreateRasterizerState(&grid_state, &rasterizer_grid);
            context->RSSetState(rasterizer_grid.Get());

            // TODO: Move these to an input callback
            XMMATRIX model = DUC->g_camera.GetWorldMatrix();
            XMMATRIX view = DUC->g_camera.GetViewMatrix();
            XMMATRIX proj = DUC->g_camera.GetProjMatrix();
            DirectX::XMStoreFloat4x4(
                &cloth_cb.model,
                model
            );
            DirectX::XMStoreFloat4x4(
                &cloth_cb.view,
                view
            );
            DirectX::XMStoreFloat4x4(
                &cloth_cb.projection,
                proj
            );
            auto mvp = model * view * proj;
            DirectX::XMStoreFloat4x4(
                &cloth_cb.mvp,
                mvp
            );

            context->UpdateSubresource(
                cloth_buffer.Get(),
                0,
                nullptr,
                &cloth_cb,
                0,
                0
            );
            if(m_iTestCase == 4) {

                update_vertex_data();
                context->UpdateSubresource(
                    vertex_buffer.Get(),
                    0,
                    nullptr,
                    vertices.data(),
                    sizeof(MassPointVertex) * vertices.size(),
                    0
                );
                context->IASetVertexBuffers(
                    0,
                    1,
                    vertex_buffer.GetAddressOf(),
                    &stride,
                    &offset
                );
            } else {
                // 3
                context->IASetVertexBuffers(
                    0,
                    1,
                    vertex_buffer.GetAddressOf(),
                    &stride,
                    &offset
                );

            }
           
           
          

            context->IASetIndexBuffer(
                index_buffer.Get(),
                DXGI_FORMAT_R32_UINT,
                0
            );


            context->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
            context->IASetInputLayout(input_layout.Get());

            context->VSSetShader(
                vertex_shader.Get(),
                nullptr,
                0
            );

            

            context->VSSetConstantBuffers(
                0,
                1,
                cloth_buffer.GetAddressOf()
            );

            context->PSSetShader(
                pixel_shader.Get(),
                nullptr,
                0
            );

            context->DrawIndexed(
                index_count,
                0,
                0
            );

            context->RSSetState(rasterizer_old.Get());
            break;
        }
        default:
        {
            std::mt19937 eng;
            std::uniform_real_distribution<float> randCol(0.0f, 1.0f);

            for (auto& mp : mass_points) {
                DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
                DUC->drawSphere(mp.position, Vec3(0.1, 0.1, 0.1));
            }

            for (const auto& spring : springs) {
                DUC->beginLine();
                DUC->drawLine(mass_points[spring.mp1].position, Vec3(1, 0, 0), mass_points[spring.mp2].position, Vec3(1, 0, 0));
                DUC->endLine();
            }
            break;
        }
    }
   

}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;

    TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &mass, "step=0.01 min=0.0001");
    TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &stiffness, "step=0.1 min=0.0001");
    TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &damping, "step=0.01 min=0.0001");

    TwType TW_TYPE_INTEGRATORTYPE = TwDefineEnumFromString("IntegrationType", "EULER,LEAPFROG,MIDPOINT");
    TwAddVarRW(DUC->g_pTweakBar, "IntegrationType", TW_TYPE_INTEGRATORTYPE, &integrator, "");

    ID3D11Device* device = DUC->g_ppd3Device;
    HRESULT hr = S_OK;
    if(!vertex_shader) {
        // Create shaders
        ID3DBlob* vs_blob = nullptr;
        D3DCompileFromFile(L"../Simulations/cloth.hlsl", NULL, NULL, "VS", "vs_5_0", 0, 0, &vs_blob, NULL);
        ID3DBlob* ps_blob = nullptr;
        D3DCompileFromFile(L"../Simulations/cloth.hlsl", NULL, NULL, "PS", "ps_5_0", 0, 0, &ps_blob, NULL);
        ID3DBlob* cs_blob = nullptr;
        ID3DBlob* err_blob = nullptr;
        hr = D3DCompileFromFile(L"../Simulations/update.hlsl", NULL, NULL, "CS", "cs_5_0", 0, 0, &cs_blob, &err_blob);
        if(FAILED(hr)) {
            if(err_blob) {
                printf("Error %s\n", err_blob->GetBufferPointer());
            }
        }

        device->CreateVertexShader(vs_blob->GetBufferPointer(), vs_blob->GetBufferSize(), NULL, &vertex_shader);
        device->CreatePixelShader(ps_blob->GetBufferPointer(), ps_blob->GetBufferSize(), NULL, &pixel_shader);
        device->CreateComputeShader(cs_blob->GetBufferPointer(), cs_blob->GetBufferSize(), NULL, &compute_shader);
        D3D11_INPUT_ELEMENT_DESC iaDesc[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        };

        hr = device->CreateInputLayout(
            iaDesc,
            ARRAYSIZE(iaDesc),
            vs_blob->GetBufferPointer(),
            vs_blob->GetBufferSize(),
            &input_layout
        );

        CD3D11_BUFFER_DESC cbDesc(
            sizeof(ClothCB),
            D3D11_BIND_CONSTANT_BUFFER
        );
        
        // Maybe check for errors?
        hr = device->CreateBuffer(
            &cbDesc,
            nullptr,
            cloth_buffer.GetAddressOf()
        );

        hr = device->CreateBuffer(
            &cbDesc,
            nullptr,
            simulation_buffer.GetAddressOf()
        );
       
    }

    if(!vertex_buffer && (m_iTestCase == 4 || m_iTestCase == 3)) {
        std::vector<uint32_t> indices;
        // Updates MassPointVertex vector
        m_iTestCase == 3 ? update_vertex_extended() : update_vertex_data();
        CD3D11_BUFFER_DESC v_desc(
            sizeof(MassPointVertex) * mass_points.size(),
            D3D11_BIND_VERTEX_BUFFER
        );

        D3D11_SUBRESOURCE_DATA v_data;
        ZeroMemory(&v_data, sizeof(D3D11_SUBRESOURCE_DATA));
        v_data.pSysMem = vertices.data();
        v_data.SysMemPitch = 0;
        v_data.SysMemSlicePitch = 0;
        hr = device->CreateBuffer(
            &v_desc,
            &v_data,
            &vertex_buffer
        );
        for(int i = 0; i < GRIDY - 1; i++) {
            for(int j = 0; j < GRIDX; j++) {
                // CCW?
                indices.push_back((i) * GRIDX + j);
                indices.push_back((i + 1) * GRIDX + j);
            }
            // Primitive restart
            indices.push_back(-1);
        }

        CD3D11_BUFFER_DESC iDesc(
            sizeof(uint32_t) * indices.size(),
            D3D11_BIND_INDEX_BUFFER
        );
        this->index_count = indices.size();

        D3D11_SUBRESOURCE_DATA iData;
        ZeroMemory(&iData, sizeof(D3D11_SUBRESOURCE_DATA));
        iData.pSysMem = indices.data();
        iData.SysMemPitch = 0;
        iData.SysMemSlicePitch = 0;
        hr = device->CreateBuffer(
            &iDesc,
            &iData,
            &index_buffer
        );

        if(m_iTestCase == 3) {
            // First time we create this buffer we want to fill it
            create_structured_buffer(device, sizeof(MassPointVertex), mass_points.size(), 
                                     vertices.data(), buffer_in.GetAddressOf());
            create_structured_buffer(device, sizeof(MassPointVertex), mass_points.size(), 0,
                                     buffer_out.GetAddressOf());
            create_srv(device, buffer_in.Get(), &srv);
            create_uav(device, buffer_out.Get(), &uav);

        }
    }
}

void MassSpringSystemSimulator::initScene()
{
    running = true;

    if (m_iTestCase >= 0 && m_iTestCase <= 2) {
        this->mass = 10.0f;
        this->damping = 0.0f;
        this->stiffness = 40.0f;
        this->applyExternalForce(Vec3(0, 0, 0));
        int p0 = this->addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
        int p1 = this->addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
        this->addSpring(p0, p1, 1.0);
    }

    switch (m_iTestCase)
    {
    case 0:
    {
        this->setIntegrator(EULER);
        *timestep = 0.1f;
        break;
    }
    case 1:
    {
        this->setIntegrator(EULER);
        *timestep = 0.005f;
        break;
    }
    case 2:
    {
        this->setIntegrator(MIDPOINT);
        *timestep = 0.005f;
        break;
    }
    case 3:
        // Fall thru
    case 4:
    {
        this->setIntegrator(LEAPFROG);
        *timestep = 0.005f;
        // Initialize the grid
        this->mass = 10.0f;
        this->damping = 3.0f;
        this->stiffness = 100000.0f;
        Vec3 initial_velocity(0, -0.05, 0);
        Vec3 initial_force(0, 0, 0);
        float spring_length = 1.0f / GRIDX; // Assume uniform grid for now
        this->applyExternalForce(Vec3(0, -10, 0));
        int GRIDSIZE = GRIDX * GRIDY;
        float dx = 1.0f / GRIDX;
        float dy = 1.0f / GRIDY;
        matrix4x4<float> t;
        this->mass_points.resize(GRIDX * GRIDY);
        this->springs.reserve(GRIDX * GRIDY * 2);
        t.initTranslation(-0.5f, 0.5f, -0.5f);
        for(int i = 0; i < GRIDY; i++) {
            for(int j = 0; j < GRIDX; j++) {
                auto curr_idx = i * GRIDX + j;
                Vec3 pos(dx * j, 0.2f, dy * i);
                auto translated_pos = t.transformVector(pos);
                mass_points[curr_idx] = {
                 translated_pos,
                 initial_velocity,
                 initial_force,
                 i == 0 ? true : false
                };
                auto right = j < GRIDX-1 ? i * GRIDX + j + 1 : curr_idx;
                auto bottom = i < GRIDY-1 ? (i + 1) * GRIDX + j : curr_idx;
                if(curr_idx != right) {
                    springs.push_back({ curr_idx,  right, spring_length });
                }
                if(curr_idx != bottom) {
                    springs.push_back({ curr_idx,  bottom, spring_length });
                }
            }
        }
    }
        break;
    case 5:
        cout << "Demo 5 !\n";
        break;
    default:
        break;
    }
}

void MassSpringSystemSimulator::update_vertex_data() {

    // Here we assume that mass points are initialized
    assert(mass_points.size()); 
    vertices.clear();
    vertices.resize(mass_points.size());
    for(int i = 0; i < mass_points.size(); i++) {
        DirectX::XMVECTOR pos = mass_points[i].position.toDirectXVector();
        XMStoreFloat3(&vertices[i].pos, pos);
        // TODO?
        // vertices[i].normal = DirectX::XMFLOAT3(0.0, 1.0, 0.0);
    }
}



void MassSpringSystemSimulator::update_vertex_extended() {
    assert(mass_points.size());
    vertices.clear();
    vertices.resize(mass_points.size());
    for(int i = 0; i < mass_points.size(); i++) {
        DirectX::XMVECTOR pos = mass_points[i].position.toDirectXVector();
        DirectX::XMVECTOR vel = mass_points[i].velocity.toDirectXVector();
        DirectX::XMVECTOR f = mass_points[i].force.toDirectXVector();
        XMStoreFloat3(&vertices[i].pos, pos);
        XMStoreFloat3(&vertices[i].vel, vel);
        XMStoreFloat3(&vertices[i].f, f);
        vertices[i].is_fixed = static_cast<uint32_t>(mass_points[i].is_fixed);
        // TODO?
        // vertices[i].normal = DirectX::XMFLOAT3(0.0, 1.0, 0.0);
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    reset();

    switch (m_iTestCase)
    {
    case 0:
        cout << "Demo 1 !\n";
        break;
    case 1:
        cout << "Demo 2 !\n";
        break;
    case 2:
        cout << "Demo 3 !\n";
        break;
    case 3:
        cout << "Demo 4 Compute Shader !\n";
        break;
    case 4:
        cout << "Demo 4 CPU !\n";
        break;
    case 5:
        cout << "Demo 5 !\n";
        break;
    default:
        cout << "Demo n !\n";
        break;
    }
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
    Point2D mouseDiff;
    mouseDiff.x = trackmouse.x - old_trackmouse.x;
    mouseDiff.y = trackmouse.y - old_trackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        // find a proper scale!
        float inputScale = 0.01f;
        inputWorld = inputWorld * inputScale;
        mouse_force = inputWorld;
    }
    else {
        mouse_force = {};
    }
}


MassSpringSystemSimulator::~MassSpringSystemSimulator() {
    if(srv) srv->Release();
    if(uav) uav->Release();
}

void MassSpringSystemSimulator::compute_elastic_force(const Spring& s) {
    auto& mp1 = mass_points[s.mp1];
    auto& mp2 = mass_points[s.mp2];

    Vec3 spring_vec = mp1.position - mp2.position;
    float x = norm(spring_vec) - s.initial_length;
    normalize(spring_vec);
    Vec3 f = -stiffness * x * spring_vec;
    mp1.force += f - damping * mp1.velocity;
    mp2.force += -f  - damping * mp2.velocity;
}

void MassSpringSystemSimulator::simulateTimestep(float time_step) {
    if (!running) {
        return;
    }
    if(m_iTestCase == 3) {
        auto context = DUC->g_pd3dImmediateContext;
        simulation_cb.delta = time_step;
        simulation_cb.sphere_radius = 1.0f;
        simulation_cb.mass = mass;
        simulation_cb.stiffness = stiffness;
        simulation_cb.damping = damping;
        simulation_cb.initial_len = 1.0f / GRIDX;
        
        auto spd = sphere_pos.toDirectXVector();
        auto grid_dim = DirectX::XMVectorSet(GRIDX, GRIDY, 0, 0);
        auto ef = (external_force + mouse_force).toDirectXVector();

        DirectX::XMStoreFloat3(&simulation_cb.sphere_pos, spd);
        DirectX::XMStoreUInt2(&simulation_cb.grid_dim, grid_dim);
        DirectX::XMStoreFloat3(&simulation_cb.external_force, ef);

        context->UpdateSubresource(
            simulation_buffer.Get(),
            0,
            nullptr,
            &simulation_cb,
            0,
            0
        );
            
        context->CSSetShader(compute_shader.Get(), nullptr, 0);
        context->CSSetShaderResources(0, 1, &srv);
        context->CSSetUnorderedAccessViews(0, 1, &uav, nullptr);
        context->CSSetConstantBuffers(
            0,
            1,
            simulation_buffer.GetAddressOf()
        );

        context->Dispatch(GRIDX / NUM_THREADS_X , GRIDY / NUM_THREADS_Y, 1);
        context->CopyResource(buffer_in.Get(), buffer_out.Get());
        context->CopyResource(vertex_buffer.Get(), buffer_out.Get());
        // For debugging CS:
        //{
        //    ID3D11Buffer* debugbuf = CreateAndCopyToDebugBuf(DUC->g_ppd3Device, 
        //                                                     context, buffer_out.Get());
        //    D3D11_MAPPED_SUBRESOURCE MappedResource;
        //    MassPointVertex* p;
        //    context->Map(debugbuf, 0, D3D11_MAP_READ, 0, &MappedResource);
        //    p = (MassPointVertex*) MappedResource.pData;
        //    std::vector<MassPointVertex> as;
        //    as.assign(p, p + mass_points.size());
        //    context->UpdateSubresource(
        //        vertex_buffer.Get(),
        //        0,
        //        nullptr,
        //        p,
        //        sizeof(MassPointVertex) * vertices.size(),
        //        0
        //    );
        //    int a = 4;
        //}
        return;
    }

    // Clear forces + add external forces 
    // Compute elastic forces
    // Apply integrator

    for(auto& mp : mass_points) {
        mp.force = this->external_force + mouse_force;
    }
    for(const auto& spring : springs) {
        compute_elastic_force(spring);
    }

    switch(integrator) {
        case EULER:
        {
            for(auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;
                if (!mp.is_fixed) {
                    mp.position = mp.position + time_step * mp.velocity;
                }
                mp.velocity = mp.velocity + time_step * accel;
            }
            break;
        }
        case LEAPFROG:
        {
            for(auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;
                // Not tested
                mp.velocity = mp.velocity + time_step * accel;
                if (!mp.is_fixed) {
                    mp.position = mp.position + time_step * mp.velocity;
                }
            }
            break;
        }
        case MIDPOINT:
        {
            old_positions.clear();
            old_velocities.clear();

            for (auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;

                old_positions.push_back(mp.position);
                old_velocities.push_back(mp.velocity);

                if (!mp.is_fixed) {
                    mp.position = mp.position + time_step / 2.0f * mp.velocity;
                }
                mp.velocity = mp.velocity + time_step / 2.0f * accel;
            }

            for (auto& mp : mass_points) {
                mp.force = this->external_force;
            }

            for (const auto& spring : springs) {
                compute_elastic_force(spring);
            }

            for (int i = 0; i < mass_points.size(); i++) {
                Vec3 accel = mass_points[i].force / mass;

                if (!mass_points[i].is_fixed) {
                    mass_points[i].position = old_positions[i] + time_step * mass_points[i].velocity;
                }
                mass_points[i].velocity = old_velocities[i] + time_step * accel;
            }

            break;
        }
        default:
            break;
    }

    if (m_iTestCase == 0) {
        int index = 0;
        for (int i = 0; i < mass_points.size(); i++) {
            auto& mp = mass_points[i];
            cout << "Point " << i + 1 << " position: " << mp.position << "\n";
            cout << "Point " << i + 1 << " velocity: " << mp.velocity << "\n";
        }
        running = false;
    }

    for (auto& mp : mass_points) {
        mp.position.x = clamp(mp.position.x, -2, 2);
        mp.position.y = clamp(mp.position.y, -1, 2);
        mp.position.z = clamp(mp.position.z, -2, 2);
    }
}

void MassSpringSystemSimulator::onClick(int x, int y) {
    trackmouse.x = x;
    trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
    old_trackmouse.x = x;
    old_trackmouse.y = y;
    trackmouse.x = x;
    trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
    this->mass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
    this->stiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
    this->damping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
    MassPoint mp = {};
    mp.position = position;
    mp.velocity = Velocity;
    mp.is_fixed = isFixed;
    mass_points.emplace_back(mp);
    return mass_points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
    springs.push_back({ masspoint1, masspoint2, initialLength });
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
    return mass_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
    return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
    return mass_points[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
    return mass_points[index].velocity;
}

void MassSpringSystemSimulator::passTimestepVariable(float& time_step)
{
    timestep = &time_step;
}
