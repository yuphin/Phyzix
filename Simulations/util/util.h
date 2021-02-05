#ifndef __util_h__
#define __util_h__


#include <string>
#include <DXUT.h>
#include <DDSTextureLoader.h>
#include <d3dcompiler.h>
#include "../DrawingUtilitiesClass.h"
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
HRESULT create_sampler(ID3D11Device* device, ID3D11SamplerState** sampler_state_ptr);


std::wstring GetExePath();

void UpdateWindowTitle(const std::wstring& appName);


inline Vec3 operator*(Vec3& v, Mat4& m) {
    auto res = XMVector3Transform(v.toDirectXVector(), m.toDirectXMatrix());
    return Vec3(res);
}

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}
template <typename T>
constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

inline XMFLOAT3 operator*(const XMFLOAT3& v, const Real s) {
    return XMFLOAT3(v.x * s, v.y * s, v.z * s);
}

inline XMFLOAT3 operator+(const XMFLOAT3& v, const XMFLOAT3& v2) {
    return XMFLOAT3(v.x + v2.x, v.y + v2.y,  v.z + v2.z);
}


#endif