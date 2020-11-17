cbuffer MVPConstantBuffer : register(b0) {
    matrix model;
    matrix view;
    matrix projection;
    matrix mvp;
};

// Wang hash for visualization purposes
uint hash(uint seed) {
    seed = (seed ^ 61) ^ (seed >> 16);
    seed *= 9;
    seed = seed ^ (seed >> 4);
    seed *= 0x27d4eb2d;
    seed = seed ^ (seed >> 15);
    return seed;
}


struct VS_IN {
    float3 pos : POSITION;
    float3 normal : NORMAL;
    uint id : SV_VertexID;
};

struct VS_OUT {
    float4 position : SV_POSITION;
    float3 normal : NORMAL0;
    float3 col:  COLOR0;
};


VS_OUT VS(VS_IN input) {
    uint hv = hash(input.id);
    float3 col = float3(float(hv & 255), float((hv >> 8) & 255), float((hv >> 16) & 255)) / 255.0;
    VS_OUT output;
    output.position = mul(mvp, float4(input.pos, 1.0));
    output.normal = input.normal;
    output.col = col;
    return output;
}


float4 PS(VS_OUT input) : SV_Target{
    return float4(input.col, 1.0);
}