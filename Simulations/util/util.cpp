#include "util.h"
#include <Windows.h>
#include <DXUT.h>

std::wstring GetExePath()
{
	// get full path to .exe
	const size_t bufferSize = 1024;
	wchar_t buffer[bufferSize];
	if(0 == GetModuleFileNameW(nullptr, buffer, bufferSize))
	{
		return std::wstring(L"");
	}
	std::wstring path(buffer);
	// extract path (remove filename)
	size_t posSlash = path.find_last_of(L"/\\");
	if(posSlash != std::wstring::npos)
	{
		path = path.substr(0, posSlash + 1);
	}
	return path;
}


void UpdateWindowTitle(const std::wstring& appName)
{
	// check if we should update the window title
	bool update = false;

	// update if window size changed
	static int s_windowWidth = 0;
	static int s_windowHeight = 0;
	if (s_windowWidth != DXUTGetWindowWidth() || s_windowHeight != DXUTGetWindowHeight()) {
		s_windowWidth = DXUTGetWindowWidth();
		s_windowHeight = DXUTGetWindowHeight();
		update = true;
	}

	// update if fps changed (updated once per second by DXUT)
	static float s_fps = 0.0f;
	static float s_mspf = 0.0f;
	if (s_fps != DXUTGetFPS()) {
		s_fps = DXUTGetFPS();
		s_mspf = 1000.0f / s_fps;
		update = true;
	}

	// update window title if something relevant changed
	if (update) {
		const size_t len = 512;
		wchar_t str[len];
		swprintf_s(str, len, L"%s %ux%u @ %.2f fps / %.2f ms", appName.c_str(), s_windowWidth, s_windowHeight, s_fps, s_mspf);
		SetWindowText(DXUTGetHWND(), str);
	}
}

HRESULT create_structured_buffer(ID3D11Device* device,
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

HRESULT create_raw_buffer(ID3D11Device* device,
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


ID3D11Buffer* CreateAndCopyToDebugBuf(ID3D11Device* pDevice, ID3D11DeviceContext* pd3dImmediateContext, ID3D11Buffer* pBuffer) {
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

HRESULT create_srv(ID3D11Device* device,
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

HRESULT create_sampler(ID3D11Device* device, ID3D11SamplerState** sampler_state_ptr) {
    D3D11_SAMPLER_DESC desc = {};
    desc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    desc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
    desc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
    desc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
    return device->CreateSamplerState(&desc, sampler_state_ptr);
}
