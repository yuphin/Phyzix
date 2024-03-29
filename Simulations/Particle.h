#pragma once
#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"
#include "util/util.h"
#include "RigidBody.h"
struct Particle {
	Particle(Real dm, Real dv, const Vec3& pos, 
		Real rho_0 = -1, int set_idx = 0, 
		bool visible = true) {

		this->mass = dm;
		this->dv = dv;
		this->pos = pos;
		this->visible = visible;
		if (rho_0 != -1) {
			// Boundary
			this->set_idx = set_idx;
			this->rho = rho_0;
		}
	}
	inline void add_force(const Vec3& force) {
		assert(owner);
		owner->apply_force(pos, force);
	}
	bool visible;
	int set_idx = 0;
	Real mass = 0;
	Real dv = 0;
	Vec3 accel = Vec3();
	Vec3 vel = Vec3();
	Vec3 pos = Vec3();
	Vec3 force = Vec3();
	Real rho = 0;
	Real pressure = 0;
	Real old_pressure = 0;
	Real aii;
	Real gamma1 = 1;
	Real gamma2 = 0;
	Vec3 dv_div_rhoSqr_grad;
	Vec3 dv_divRhoSqr_p_grad;
	Real rho_star;
	RigidBody* owner = nullptr;
};

