cbuffer StateCB : register(b0) {
    float3 sphere_pos;
    float delta;
    uint2 grid_dim;
    float sphere_radius;
    float mass;
    float stiffness;
    float damping;
    float initial_len;
    uint num_cloths;
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

// We could communicate this info with CPU
[numthreads(20,20,1)]
void CS(uint3 id : SV_DispatchThreadID) {
    for(uint k = 0; k < num_cloths; k++) {
        uint idx = k * grid_dim.x * grid_dim.y + id.y * grid_dim.x + id.x;
        if(idx > num_cloths * grid_dim.x * grid_dim.y) {
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

        if(buf_in[idx].is_fixed == 0) {
            buf_out[idx].vel = curr_vel + delta * accel;
            buf_out[idx].pos = curr_pos + delta * buf_out[idx].vel;
        } else {
            buf_out[idx].vel = curr_vel;
            buf_out[idx].pos = curr_pos;
        }

        curr_pos = buf_out[idx].pos;
        buf_out[idx].pos.x = clamp(curr_pos.x, -2, 2);
        buf_out[idx].pos.y = clamp(curr_pos.y, -0.999, 2);
        buf_out[idx].pos.z = clamp(curr_pos.z, -2, 2);

        buf_out[idx].is_fixed = buf_in[idx].is_fixed;
    }
  
}