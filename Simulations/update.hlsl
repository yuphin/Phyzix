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
    float cube_radius;
    float3 cube_pos;
    uint integrator;
    float cross_len;
};

struct MassPoint {
    float3 pos;
    float2 uv;
    float3 vel;
    float3 f;
    uint is_fixed;
};

StructuredBuffer<MassPoint> buf_in;
RWStructuredBuffer<MassPoint> buf_out;

float3 compute_elastic_force(float3 mp1, float3 mp2, bool cross) {
    float3 spring_vec = mp1 - mp2;
    return stiffness * (length(spring_vec) - (cross ? cross_len : initial_len)) * normalize(spring_vec);
}

float3 calculate_force(uint3 id, uint idx, float3 curr_pos, float3 curr_vel) {
    float3 force = external_force;
    // Instead of springs, we traverse in grid. I.e look for neighbors

    bool left = false, right = false, up = false, down = false;

    // Left
    if (id.x > 0) {
        force += compute_elastic_force(buf_in[idx - 1].pos, curr_pos, false);
        left = true;
    }
    // Right
    if (id.x < grid_dim.x - 1) {
        force += compute_elastic_force(buf_in[idx + 1].pos, curr_pos, false);
        right = true;
    }

    // Up
    if (id.y < grid_dim.y - 1) {
        force += compute_elastic_force(buf_in[idx + grid_dim.x].pos, curr_pos, false);
        up = true;
    }

    // Down
    if (id.y > 0) {
        force += compute_elastic_force(buf_in[idx - grid_dim.x].pos, curr_pos, false);
        down = true;
    }

    if (right && up) {
        force += compute_elastic_force(buf_in[idx + 1 + grid_dim.x].pos, curr_pos, true);
    }

    if (right && down) {
        force += compute_elastic_force(buf_in[idx + 1 - grid_dim.x].pos, curr_pos, true);
    }

    if (left && up) {
        force += compute_elastic_force(buf_in[idx - 1 + grid_dim.x].pos, curr_pos, true);
    }

    if (left && down) {
        force += compute_elastic_force(buf_in[idx - 1 - grid_dim.x].pos, curr_pos, true);
    }

    // Damping
    force += -damping * curr_vel;

    return force;
}

// We could communicate this info with CPU
[numthreads(20,20,1)]
void CS(uint3 id : SV_DispatchThreadID) {
    const float dn = 0.001;
    const float sphere_dn = 0.005;
    for(uint k = 0; k < num_cloths; k++) {
        uint idx = k * grid_dim.x * grid_dim.y + id.y * grid_dim.x + id.x;
        if(idx > num_cloths * grid_dim.x * grid_dim.y) {
            return;
        }

        float3 curr_pos = buf_in[idx].pos;
        float3 curr_vel = buf_in[idx].vel;

        float3 force = calculate_force(id, idx, curr_pos, curr_vel);
        // Integrate 
        float3 accel = force * (1.0 / mass);

        if (buf_in[idx].is_fixed == 0) {
            if (integrator == 0) {
                buf_out[idx].pos = curr_pos + delta * curr_vel;
                buf_out[idx].vel = curr_vel + delta * accel;
            }
            else if (integrator == 1) {
                buf_out[idx].vel = curr_vel + delta * accel;
                buf_out[idx].pos = curr_pos + delta * buf_out[idx].vel;
            }
            else {
                buf_out[idx].pos = curr_pos + delta / 2.0 * curr_vel;
                buf_out[idx].vel = curr_vel + delta / 2.0 * accel;

                force = calculate_force(id, idx, buf_out[idx].pos, buf_out[idx].vel);
                accel = force * (1.0 / mass);

                // curr_pos and curr_vel are actually old pos and vel in this context
                buf_out[idx].pos = curr_pos + delta * buf_out[idx].vel;
                buf_out[idx].vel = curr_vel + delta * accel;
            }
        }
        else {
            buf_out[idx].vel = curr_vel;
            buf_out[idx].pos = curr_pos;
        }

        // Sphere collision
        float3 dist_sphere = curr_pos - sphere_pos;
        if(length(dist_sphere) < sphere_radius + sphere_dn) {
            buf_out[idx].pos = sphere_pos + normalize(dist_sphere) * (sphere_radius + sphere_dn);
            buf_out[idx].vel = float3(0, 0, 0);
        }
        // Cube collision
        float3 dist_cube = curr_pos - cube_pos ;
        float cube_boundary = cube_radius + 20*dn;
        float3 clamped = clamp(dist_cube, -cube_boundary, cube_boundary);
        float3 axis = float3(0.0, 0.0, 0.0);
        if(abs(clamped.x) > cube_radius) {
            axis = axis + float3(sign(clamped.x) * 1.0, 0, 0);
        }
        if(abs(clamped.y) > cube_radius) {
            axis = axis + float3(0, sign(clamped.y) * 1.0, 0);
        }
        if(abs(clamped.z) > cube_radius) {
            axis = axis + float3(0, 0, sign(clamped.z) * 1.0);
        }
        if(length(dist_cube - clamped) < dn){
            buf_out[idx].pos = curr_pos + axis * dn;
            buf_out[idx].vel = float3(0, 0, 0);
        }
        // Floor collision
        if(curr_pos.y < -1.0) {
            buf_out[idx].pos.y = -1.0 + dn;
            buf_out[idx].vel = float3(0, 0, 0);
        }

        buf_out[idx].is_fixed = buf_in[idx].is_fixed;
        buf_out[idx].uv = buf_in[idx].uv;
    }
  
}