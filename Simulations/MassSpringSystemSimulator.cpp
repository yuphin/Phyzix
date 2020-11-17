#include "MassSpringSystemSimulator.h"
#include <d3dcompiler.h>
#pragma comment(lib,"D3dcompiler.lib")
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_iTestCase = 0;
    // For debugging:
    /*this->mass = 10.0f;
    this->damping = 0.0f;
    this->stiffness = 40.0f;
    this->applyExternalForce(Vec3(0, 0, 0));
    int p0 = this->addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
    int p1 = this->addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
    this->addSpring(p0, p1, 1.0);
    this->setIntegrator(EULER);*/


}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 2, Demo 3, Demo 4, Demo 5, Demo Solid";
}
 
void MassSpringSystemSimulator::reset() {
    mouse.x = mouse.y = 0;
    trackmouse.x = trackmouse.y = 0;
    old_trackmouse.x = old_trackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* context) {

    uint32_t stride = sizeof(Vertex);
    uint32_t offset = 0;
   
       
    switch(m_iTestCase) {
        case 4:
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
                &constant_buffer_data.model,
                model
            );
            DirectX::XMStoreFloat4x4(
                &constant_buffer_data.view,
                view
            );
            DirectX::XMStoreFloat4x4(
                &constant_buffer_data.projection,
                proj
            );
            auto mvp = model * view * proj;
            DirectX::XMStoreFloat4x4(
                &constant_buffer_data.mvp,
                mvp
            );

            context->UpdateSubresource(
                constant_buffer.Get(),
                0,
                nullptr,
                &constant_buffer_data,
                0,
                0
            );

            context->IASetVertexBuffers(
                0,
                1,
                vertex_buffer.GetAddressOf(),
                &stride,
                &offset
            );

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
                constant_buffer.GetAddressOf()
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
        }
        break;

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
        device->CreateVertexShader(vs_blob->GetBufferPointer(), vs_blob->GetBufferSize(), NULL, &vertex_shader);
        device->CreatePixelShader(ps_blob->GetBufferPointer(), ps_blob->GetBufferSize(), NULL, &pixel_shader);
        D3D11_INPUT_ELEMENT_DESC iaDesc[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },

            { "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },

            { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT,
            0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        };

        hr = device->CreateInputLayout(
            iaDesc,
            ARRAYSIZE(iaDesc),
            vs_blob->GetBufferPointer(),
            vs_blob->GetBufferSize(),
            &input_layout
        );

        CD3D11_BUFFER_DESC cbDesc(
            sizeof(ConstantBufferStruct),
            D3D11_BIND_CONSTANT_BUFFER
        );

        hr = device->CreateBuffer(
            &cbDesc,
            nullptr,
            constant_buffer.GetAddressOf()
        );
       
    }

    if(!vertex_buffer) {
        // Create vertex buffers (for demo purposes)
        constexpr int GRIDX = 100;
        constexpr int GRIDY = 100;
        constexpr int GRIDSIZE = GRIDX * GRIDY;
        constexpr float dx = 1.0f / GRIDX;
        constexpr float dy = 1.0f / GRIDY;
        std::vector<Vertex> vertices(GRIDX * GRIDY);
        std::vector<uint32_t> indices;
        matrix4x4<float> t;
        t.initTranslation(-0.5f, -0.5f, -0.5f);
        auto translation = t.toDirectXMatrix();

        // TODO : Get these vertices from MassPoint vector
        for(int i = 0; i < GRIDY; i++) {
            for(int j = 0; j < GRIDX; j++) {
                XMVECTOR pos = XMVectorSet(dx * j, 0.2, dy * i, 1.0f);
                pos = XMVector3Transform(pos, translation);
                vertices[i * GRIDX + j] = {
                    DirectX::XMFLOAT3(XMVectorGetX(pos), XMVectorGetY(pos), XMVectorGetZ(pos)),
                    DirectX::XMFLOAT3(0,0,0),
                    DirectX::XMFLOAT3(0,1.0,0),
                };
            }
        }
        CD3D11_BUFFER_DESC vDesc(
            sizeof(Vertex) * vertices.size(),
            D3D11_BIND_VERTEX_BUFFER
        );
        D3D11_SUBRESOURCE_DATA vData;
        ZeroMemory(&vData, sizeof(D3D11_SUBRESOURCE_DATA));
        vData.pSysMem = vertices.data();
        vData.SysMemPitch = 0;
        vData.SysMemSlicePitch = 0;
        hr = device->CreateBuffer(
            &vDesc,
            &vData,
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

    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase)
    {
    case 0:
        cout << "Demo 2 !\n";
        break;
    case 1:
        cout << "Demo 3 !\n";
        break;
    case 2:
        cout << "Demo 4 !\n";
        break;
    case 3:
        cout << "Demo 5 !\n";
        break;
    case 4:
        cout << "Demo Solid !\n";
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
        float inputScale = 0.001f;
        inputWorld = inputWorld * inputScale;
        
        for (auto& masspoint : mass_points) {
            masspoint.force += inputWorld;
        }
    }
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
    // Clear forces + add external forces 
    // Compute elastic forces
    // Apply integrator

    for(auto& mp : mass_points) {
        mp.force = this->external_force;
    }
    for(const auto& spring : springs) {
        compute_elastic_force(spring);
    }

    switch(integrator) {
        case EULER:
        {
            for(auto& mp : mass_points) {
                Vec3 accel = mp.force / mass;
                mp.position = mp.position + time_step * mp.velocity;
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
                mp.position = mp.position + time_step * mp.velocity;
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

                mp.position = mp.position + time_step / 2.0f * mp.velocity;
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

                mass_points[i].position = old_positions[i] + time_step * mass_points[i].velocity;
                mass_points[i].velocity = old_velocities[i] + time_step * accel;
            }

            break;
        }
        default:
            break;
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
