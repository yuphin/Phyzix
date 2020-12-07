SamplerState smplr_lowres : register(s0);
SamplerState smplr : register(s1);
Texture2D accum_tex : register(t0);
Texture2D<int3> bvh_tex : register(t1);
Texture2D<float3> bb_min_tex : register(t2);
Texture2D<float3> bb_max_tex : register(t3);
Texture2D<int3> idx_tex : register(t4);
Texture2D<float4> vert_tex : register(t5);
Texture2D<float4> normals_tex : register(t6);
Texture2D<float4> mat_tex : register(t7);
Texture2D<float4> transforms_tex : register(t8);
Texture2D<float4> lights_tex : register(t9);
#define INF  1000000.0
#define PI 3.14159265359
#define PI_2 1.57079632679
#define PI2 6.28318530718
#define PI4 12.5663706144
#define PI_4 0.78539816339
#define EPS 0.001
struct VS_OUT {
    float4 position: SV_POSITION;
    float2 uv: TEXCOORD0;
};
struct CameraCB {
	float4 pos;
	float4 right;
	float4 up;
	float4 forward;
};

cbuffer PathTraceCB : register(b0) {
	int bvh_root_idx;
	int idx_buf_tex_size;
	int num_lights;
	float2 resolution;
	float2 inv_num_tiles;
};

cbuffer FrameCB : register(b1) {
	float3 rand_vec;
	int tile_x;
	int tile_y;
	int max_depth;
	float fov;
	CameraCB camera;
};


// =================================
// ======================
// ======================
// =================================

struct Ray {
	float3 origin;
	float3 dir;
};
struct HitInfo {
	float dist;
	float3 p; // Hit point
	float3 n; // Normal
	//float3 ffn; // Face forward normal
	int depth;
	float2 tex_coord;
	float3 bary;
	int3 tri_idx;
	int mat_idx;
    float4x4 transform;
    float3 tex_coords;
	//bool spec_bounce;
	bool is_emitter;
};
struct BSDFSampleInfo { 
	float3 bsdf_dir; 
	float pdf; 
};
struct LightSampleInfo { 
	float3 surface_pos; 
	float3 n; 
	float3 emission; 
	float pdf; 
};

float map(float value, float low1, float high1, float low2, float high2) {
	return low2 + ((value - low1) * (high2 - low2)) / (high1 - low1);
}

float4x4 inverse(float4x4 m) {
    float n11 = m[0][0], n12 = m[1][0], n13 = m[2][0], n14 = m[3][0];
    float n21 = m[0][1], n22 = m[1][1], n23 = m[2][1], n24 = m[3][1];
    float n31 = m[0][2], n32 = m[1][2], n33 = m[2][2], n34 = m[3][2];
    float n41 = m[0][3], n42 = m[1][3], n43 = m[2][3], n44 = m[3][3];

    float t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44;
    float t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44;
    float t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44;
    float t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;

    float det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;
    float idet = 1.0f / det;

    float4x4 ret;

    ret[0][0] = t11 * idet;
    ret[0][1] = (n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44) * idet;
    ret[0][2] = (n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44) * idet;
    ret[0][3] = (n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43) * idet;

    ret[1][0] = t12 * idet;
    ret[1][1] = (n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44) * idet;
    ret[1][2] = (n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44) * idet;
    ret[1][3] = (n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43) * idet;

    ret[2][0] = t13 * idet;
    ret[2][1] = (n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44) * idet;
    ret[2][2] = (n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44) * idet;
    ret[2][3] = (n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43) * idet;

    ret[3][0] = t14 * idet;
    ret[3][1] = (n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34) * idet;
    ret[3][2] = (n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34) * idet;
    ret[3][3] = (n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33) * idet;

    return ret;
}

float rand(inout float2 seed) {
	seed -= rand_vec.xy;
	return frac(sin(dot(seed, float2(12.7255, 78.203))) * 13249.3292);
}
// Fast, Minimum Storage Ray/Triangle Intersection (1997)
// https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
float4 intersect_tri(float3 ro, float3 rd, float3 v0, float3 v1, float3 v2, in HitInfo info) {
    float3 e1 = v1 - v0;
    float3 e2 = v2 - v0;
    
    float3 d_cross_e2 = cross(rd, e2);
    float det = dot(e1, d_cross_e2);
    if(det > -EPS && det < EPS) {
        return float4(0, 0, 0, 0);
    }
    float inv_det = 1.0 / det;
    float3 tvec = ro - v0;
    float u = inv_det * dot(tvec, d_cross_e2);
    if(u < 0.0 || u >  1.0) {
        return float4(0, 0, 0, 0);
    }
    float3 tvec_cross_e1 = cross(tvec, e1);
    float v = inv_det * dot(rd, tvec_cross_e1);
    if(v < 0.0 || u + v > 1.0) {
        return float4(0,0,0,0);
    }
    float t = dot(e2, tvec_cross_e1) * inv_det;
    return float4(u, v, t, 1.0);

    
}

bool intersect_aabb(float3 bb_min, float3 bb_max, in Ray r, out float t) {
    float3 inv_dir = 1.0 / r.dir;
    float3 n = (bb_min - r.origin) * inv_dir;
    float3 f = (bb_max - r.origin) * inv_dir;

    float3 tmax = max(f, n);
    float3 tmin = min(f, n);

    float t1 = min(tmax.x, min(tmax.y, tmax.z));
    float t0 = max(tmin.x, max(tmin.y, tmin.z));

    bool hit = (t1 >= 0.0f) && t1 >= t0;  //(t1 >= t0) ? (t0 > 0 ? t0 : t1) : -1.0;
    if (hit) {
        t = t0;
    }
    return hit;
}

float intersect_scene(in Ray r, out HitInfo info, in LightSampleInfo light_info) {
    info = (HitInfo) 0;
    float t = INF;
	float d;
	for(int i = 0; i < num_lights; i++) {
		// Fill light related data
	}
    int cnt = 0;
    int stack[64];// = (int[64])0;
	int ptr = 0;
    stack[ptr++] = -1;

    int idx = bvh_root_idx;

    int cur_mat_id = 0;
    bool mesh_bvh = false;

    Ray ray_mesh;
    float4x4 temp_transform;
    ray_mesh.origin = r.origin;
    ray_mesh.dir = r.dir;
    int deferred = -1;
    [loop] 
    while(idx > -1 || mesh_bvh) {
       
        cnt++;
        int n = idx;

        if(mesh_bvh && idx < 0) {
            mesh_bvh = false;
            idx = stack[--ptr];
            ray_mesh.origin = r.origin;
            ray_mesh.dir = r.dir;
            continue;
        }

        int3 index = int3(n >> 12, n & 0x00000FFF, 0);
        int3 bvh_node = bvh_tex.Load(index).xyz;

        int left_idx = int(bvh_node.x);
        int right_idx = int(bvh_node.y);
        int leaf = int(bvh_node.z);
        [branch]
        if(leaf > 0) {
            info.n = float3(idx, ptr, cnt);
            return t;
            for(int i = 0; i < right_idx; i++) {
                // Left idx is the starting idx for this mesh
                int3 index = int3((left_idx + i) % idx_buf_tex_size, (left_idx + i) / idx_buf_tex_size, 0);
                int3 vert_indices = idx_tex.Load(index).xyz;

                float4 v0 = vert_tex.Load(int3(vert_indices.x >> 12, vert_indices.x & 0x00000FFF, 0));
                float4 v1 = vert_tex.Load(int3(vert_indices.y >> 12, vert_indices.y & 0x00000FFF, 0));
                float4 v2 = vert_tex.Load(int3(vert_indices.z >> 12, vert_indices.z & 0x00000FFF, 0));
                float4 tri_hit = intersect_tri(ray_mesh.origin, ray_mesh.dir, v0.xyz, v1.xyz, v2.xyz, info);
                if(tri_hit.w && tri_hit.z < t) {
                    t = tri_hit.z;
                    info.p = ray_mesh.origin + ray_mesh.dir * t;
                    info.bary = tri_hit.xyz;
                    // We need to transform from local object space back into world space...
                    info.p = mul(float4(info.p, 1.0), temp_transform).xyz;
                    // ...and save the model matrix into state
                    info.transform = temp_transform;
                    info.is_emitter = false;
                    info.tri_idx = vert_indices;
                    info.mat_idx = cur_mat_id;
                    // Get the first half of tex coords here, we will take the other half later
                    info.tex_coords = float3(v0.w, v1.w, v2.w);
                }
            }
        } else if(leaf < 0) {
          
            // Mesh transformation
            // Coordinates are stored in left index
            // Materials are right idx
           
            idx = left_idx;
           
            float4 r1 = transforms_tex.Load(int3((-leaf - 1) * 4 + 0, 0, 0)).xyzw;
            float4 r2 = transforms_tex.Load(int3((-leaf - 1) * 4 + 1, 0, 0)).xyzw;
            float4 r3 = transforms_tex.Load(int3((-leaf - 1) * 4 + 2, 0, 0)).xyzw;
            float4 r4 = transforms_tex.Load(int3((-leaf - 1) * 4 + 3, 0, 0)).xyzw;

            temp_transform = float4x4(r1, r2, r3, r4);

            ray_mesh.origin = mul(float4(r.origin, 1.0), inverse(temp_transform)).xyz;
            ray_mesh.dir = mul(float4(r.dir, 1.0), inverse(temp_transform)).xyz;
            stack[ptr++] = -1;
            mesh_bvh = true;
            cur_mat_id = right_idx;
            /*if (cnt > 0) {
                info.n = float3(idx, ptr, cnt);
                return t;
            }*/
            continue;
        } else {
          
            // No leafs, continue traversal
            int3 lc = int3(left_idx >> 12, left_idx & 0x00000FFF, 0);
            int3 rc = int3(right_idx >> 12, right_idx & 0x00000FFF, 0);
            float left_hit, right_hit;
            bool lh = intersect_aabb(bb_min_tex.Load(lc).xyz, 
                                      bb_max_tex.Load(lc).xyz, ray_mesh, left_hit);
            float rh = intersect_aabb(bb_min_tex.Load(rc).xyz, 
                                       bb_max_tex.Load(rc).xyz, ray_mesh, right_hit);
            //info.n = float3(left_hit, right_hit, cnt);
            [branch]
            if(lh && rh ) {
                [branch]
                if(left_hit > right_hit) {
                    idx = right_idx;
                    deferred = left_idx;
                } else {
                    idx = left_idx;
                    deferred = right_idx;
                }
                stack[ptr++] = deferred;
                continue;
            } else if(lh) {
                idx = left_idx;
                continue;
            } else if(rh) {
                idx = right_idx;
                continue;
            }
        }
        idx = stack[--ptr];
    }
    info.dist = t * cnt * 0.25;//t;
    return t;
}

float3 path_trace(Ray ray) {
	float3 color = 0.0;
	float3 throughput = 1.0;
	int i = 0;
	HitInfo info;
	LightSampleInfo light_info;
	BSDFSampleInfo bsdf_info;
	for(; i < max_depth; i++) {
		float t = intersect_scene(ray, info, light_info);
		info.depth = i;
	}
	return float3(info.n);
}

float4 PS(VS_OUT input) : SV_Target{
	// Note : Bottom left is -1,-1 in a local tile
	//return float4(vert_tex.Sample(smplr, input.uv).rgb, 1.0);
	float2 tile_coords = input.uv;
	float2 global_tex_coords;
	float offset_x = -1.0 + 2.0 * inv_num_tiles.x * float(tile_x);
	float offset_y = -1.0 + 2.0 * inv_num_tiles.y * float(tile_y);
	// -1 - 1
	tile_coords.x = map(tile_coords.x, 0.0, 1.0, offset_x, offset_x + 2.0 * inv_num_tiles.x);
	tile_coords.y = map(tile_coords.y, 0.0, 1.0, offset_y, offset_y + 2.0 * inv_num_tiles.y);
	// from 0-1 tile space to 0 - 1 global screen space
	global_tex_coords.x = map(input.uv.x, 0.0, 1.0,
					   inv_num_tiles.x * float(tile_x),
					   inv_num_tiles.x * (float(tile_x) + 1));
	global_tex_coords.y = map(input.uv.y, 0.0, 1.0,
					   inv_num_tiles.y * float(tile_y),
					   inv_num_tiles.y * (float(tile_y) + 1));
	float2 seed = global_tex_coords;
	// Inverse transform sampling of tent filter
	float r_1 = 2 * rand(seed);
	float r_2 = 2 * rand(seed);
	float2 d;
	d.x = r_1 < 1 ? sqrt(r_1) - 1 : 1 - sqrt(2 - r_1);
	d.y = r_2 < 1 ? sqrt(r_2) - 1 : 1 - sqrt(2 - r_2);
	d *= 1.0 / (resolution * 0.5);
	float2 px = tile_coords + d;
    float fov_scale = tan(fov * 0.5);
	px.y *= resolution.y * (1.0 / resolution.x) * fov_scale;
    px.x *= fov_scale;
	Ray ray;
	ray.dir = normalize(px.x * camera.right + px.y * camera.up + camera.forward).xyz;
    //return float4(ray.dir, 1.0);
	ray.origin = camera.pos.xyz;
    float3 accum_col = float3(0, 0, 0);//accum_tex.Sample(smplr, input.uv).rgb;

	float3 curr_col = path_trace(ray);

	return float4(curr_col + accum_col, 1.0);
}