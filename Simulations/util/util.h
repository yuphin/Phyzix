#ifndef __util_h__
#define __util_h__


#include <string>
#include <DXUT.h>
#include <DDSTextureLoader.h>
#include <d3dcompiler.h>
#pragma comment(lib,"D3dcompiler.lib")


HRESULT create_structured_buffer(ID3D11Device* device,
                         UINT element_size,
                         UINT count,
                         void* init_data, ID3D11Buffer** buf_ptr);
HRESULT create_raw_buffer(ID3D11Device* device,
                  UINT element_size,
                  UINT count,
                  void* init_data, ID3D11Buffer** buf_ptr);

ID3D11Buffer* CreateAndCopyToDebugBuf(ID3D11Device* device, 
                                      ID3D11DeviceContext* context, 
                                      ID3D11Buffer* buffer);


HRESULT create_srv(ID3D11Device* device,
                   ID3D11Buffer* buffer,
                   ID3D11ShaderResourceView** srv_ptr);

HRESULT create_uav(ID3D11Device* device,
                   ID3D11Buffer* buffer,
                   ID3D11UnorderedAccessView** uav_ptr);
HRESULT create_sampler(ID3D11Device* device, ID3D11SamplerState** sampler_state_ptr,
                       D3D11_FILTER filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR,
                       D3D11_TEXTURE_ADDRESS_MODE mode = D3D11_TEXTURE_ADDRESS_WRAP);


std::wstring GetExePath();

void UpdateWindowTitle(const std::wstring& appName);


#endif
