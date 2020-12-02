
struct VS_OUT {
	float4 position: SV_POSITION;
	float2 uv: TEXCOORD0;
};

VS_OUT VS(uint vtx_id : SV_VertexID) {
	VS_OUT output;
	output.uv = float2(vtx_id & 1, vtx_id >> 1);
	// Top left v is 0 in DX
	output.position = float4((output.uv.x - 0.5f) * 2, -(output.uv.y - 0.5f) * 2, 0, 1);
	return output;
}

float4 PS(VS_OUT input) : SV_Target{
	return float4(input.uv.x, input.uv.y, 0.0, 1.0);
}