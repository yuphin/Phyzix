cbuffer StateCB : register(b0) {
    float3 sphere_pos;
    float delta;
    uint2 grid_dim;
    float sphere_radius;
    float mass;
    float stiffness;
    float damping;
    float initial_len;
    float3 external_force;
};

struct MassPoint {
    float3 pos;
    float3 vel;
    float3 f;
    uint is_fixed;
};

StructuredBuffer<MassPoint> buf_in;
RWStructuredBuffer<MassPoint> buf_out;

float3 compute_elastic_force(float3 mp1, float3 mp2) {
    float3 spring_vec = mp1 - mp2;
    return stiffness * (length(spring_vec) - initial_len) * normalize(spring_vec);
}

[numthreads(1,1,1)]
void CS(uint3 id : SV_DispatchThreadID) {
    uint idx = id.y * grid_dim.x + id.x;
    if(idx > grid_dim.x * grid_dim.y) {
        return;
    }
    float3 force = external_force; 
    // Instead of springs, we traverse in grid. I.e look for neighbors

    float3 curr_pos = buf_in[idx].pos;
    float3 curr_vel = buf_in[idx].vel;
    // Left
    if(id.x > 0) {
        force += compute_elastic_force(buf_in[idx - 1].pos, curr_pos);
    }
    // Right
    if(id.x < grid_dim.x - 1) {
        force += compute_elastic_force(buf_in[idx + 1].pos, curr_pos);
    }

    // Up
    if(id.y < grid_dim.y - 1) {
        force += compute_elastic_force(buf_in[idx + grid_dim.x].pos, curr_pos);
    }

    // Down
    if(id.y > 0) {
        force += compute_elastic_force(buf_in[idx - grid_dim.x].pos, curr_pos);
    }
    
    // Damping
    force += -damping * curr_vel;
    
    // Integrate 
    float3 accel = force * (1.0 / mass);
    buf_out[idx].pos = curr_pos + delta * curr_vel;
    buf_out[idx].vel = curr_vel + delta * accel;

}