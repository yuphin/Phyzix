#include "SPHSimulator.h"
#include "util/util.h"
#define DIM 10
SPHSimulator::SPHSimulator() {
	m_iTestCase = 0;
	num_particles = DIM * DIM;
	gravity = Vec3(0, -9.81, 0);
	particle_radius = 0.3;
	rho_0 = 1000;
	// For kernel
	support_radius = 4 * particle_radius;
	kernel.init(support_radius);
	particles.reserve(num_particles);
	init_sim();
}

void SPHSimulator::init_sim() {
	// Init fluid particles
	int k = 2;
	const double two_r = particle_radius * 2.0f;
	const auto& pr = particle_radius;
	auto l = Vec3(-DIM * two_r, 0, 0);
	auto t = Vec3(0, DIM * two_r, 0);
	auto n = Vec3(-DIM * two_r, 0, 0);
	// Fluids
	for (int i = 0; i < DIM; i++) {
		for (int j = 0; j < DIM; j++) {
			// Note: The coefficient here is tweaked depending on the scene
			// TODO: Investigate
			dv = is_2d ? 1 * M_PI * pr * pr : 1 * (4.0 * M_PI / 3) * pr * pr * pr;
			dm = rho_0 * dv;
			const Vec3 pos = Vec3(i * two_r, j * two_r, 0);
			particles.push_back(Particle(dm, dv, l + t + pos));
		}
	}
	// Boundaries
	boundary_particles.push_back(Particle(dm, 0, Vec3(0, -0.75, 0), true));
	auto h_count = 20;
	auto v_count = (5 + 0.75) / particle_radius;
	l = Vec3(-h_count * particle_radius, 0, 0);
	auto r = Vec3(h_count * particle_radius, 0, 0);
	t = Vec3(0, -1 +v_count * particle_radius, 0);
	n = Vec3(-DIM * two_r, 0, 0);
	boundary_particles.back().rho = rho_0;
	for (int k = 0; k < 2; k++) {
		for (int i = 0; i < 2 * h_count; i++) {
			const Vec3 pos = Vec3(i * particle_radius, -0.75 - k * particle_radius, 0);
			boundary_particles.push_back(Particle{ dm, 0, pos + l});
		}
		for (int i = 0; i < v_count; i++) {
			Vec3 pos = Vec3(- k * particle_radius, - i * particle_radius, 0);
			boundary_particles.push_back(Particle{ dm, 0, pos + l + t });
			pos = Vec3( k * particle_radius, - i * particle_radius, 0);
			boundary_particles.push_back(Particle{ dm, 0, pos + r + t });
		}

	}
	compute_boundary_volumes();
}

void SPHSimulator::compute_boundary_volumes() {
	// Compute masses of boundaries ( [Akinci2012] eqn. 4 & 5)
	for (int i = 0; i < boundary_particles.size(); i++) {
		Real delta = kernel.W(0);
		const Vec3& xi = boundary_particles[i].pos;
		for (int j = 0; j < boundary_particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = boundary_particles[j].pos;
			delta += kernel.W(norm(xi - xj));
		}
		boundary_particles[i].dv = 1.0 / delta;
	}
}

void SPHSimulator::simulateTimestep(float time_step) {
	// Implements IISPH as described in [Koschier2019] pp. 14
	update_time_step(time_step);
	compute_density();
	compute_non_pressure_forces();
	enforce_continuity(time_step);
	solve_pressure(time_step);
	integrate(time_step);
}

void SPHSimulator::update_time_step(float& time_step) {
	Real max = 0.1;
	for (int i = 0; i < particles.size(); i++) {
		Real vpdv = norm(particles[i].vel + particles[i].accel * time_step);
		if (vpdv > max) {
			max = vpdv;
		}
	}
	for (int i = 0; i < boundary_particles.size(); i++) {
		Real vpdv = norm(boundary_particles[i].vel);
		if (vpdv > max) {
			max = vpdv;
		}
	}
	// CFL Condition. See [Koschier2019] Eqn. 33
	const Real two_r = particle_radius * 2;
	float ts = cfl_k * 0.4 * particle_radius / max;
	if (ts < 1e-5) {
		time_step = 1e-5;
	} else if (ts <= time_step) {
		time_step = ts;
	}
}

void SPHSimulator::compute_density() {
	// Compute density and clear accelerations/forces

	for (int i = 0; i < particles.size(); i++) {
		Real density = particles[i].dv *  kernel.W(0);
		const Vec3& xi = particles[i].pos;
		// Fluid
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = particles[j].pos;
			density += particles[j].dv * kernel.W(norm(xi - xj));
		}
		// Boundary
		for (int j = 0; j < boundary_particles.size(); j++) {

			const Vec3& xj = boundary_particles[j].pos;
			auto krnel = kernel.W(norm(xi - xj));
			density += boundary_particles[j].dv * krnel;
		}
		particles[i].rho = density * rho_0;
		particles[i].accel = gravity;
		particles[i].force = 0;
	}
}

void SPHSimulator::compute_non_pressure_forces() {
	// TODO
}

void SPHSimulator::enforce_continuity(float dt) {
	// [Koschier2019] Eq. 49 : calculate A_ii
	const float dt2 = dt * dt;
	for (int i = 0; i < particles.size(); i++) {
		Vec3& term1 = particles[i].dv_div_rhoSqr_grad;
		const Vec3& xi = particles[i].pos;
		term1 = Vec3();

		Real rho2 = pow(particles[i].rho / rho_0, 2);
		//  v* = v + dt * a_nonpressure
		particles[i].vel += dt * particles[i].accel;
		// Fluid
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = particles[j].pos;
			const Real rho = particles[j].rho / rho_0;
			rho2 = rho * rho;
			term1 += (particles[j].dv / rho2) * kernel.W_grad(xi - xj);
		}
		// Boundary
		for (int j = 0; j < boundary_particles.size(); j++) {
			const Vec3& xj = boundary_particles[j].pos;
			const Vec3 del_w = kernel.W_grad(xi - xj);
			term1 += (boundary_particles[j].dv / rho2) * del_w;
		}
	}
	for (int i = 0; i < particles.size(); i++) {
		const Vec3& xi = particles[i].pos;
		const Vec3& vi = particles[i].vel;
		const Real rho = particles[i].rho / rho_0;
		const Real rho2 = rho * rho;
		const Real dv_div_rho2 = particles[i].dv / rho2;
		particles[i].aii = 0;
		particles[i].rho_star = rho;
		// Fluid
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = particles[j].pos;
			const Vec3& vj = particles[j].pos;
			const Vec3 del_w = kernel.W_grad(xi - xj);
			const Vec3 term2 = dv_div_rho2 * del_w;
			if (del_w.x != 0 || del_w.y != 0 || del_w.z != 0) {
				int d = 4;
			}
			particles[i].aii -= particles[j].dv * dot((particles[i].dv_div_rhoSqr_grad + term2), del_w);
			// While we're at it, calculate predicted density according to the continuity 
			// equation i.e:
			// rho* = rho - dt * rho * div(v*)
			// where v* = v + dt * a_nonpressure
			// Note that div(v_i) ~ \sum_j (m_j / rho_j) * (vj - vi) * grad(W)
			// as opposed to rho_i * \sum_j m_j * (v_i / rho_i^2 + v_j / rho_j^2) * grad(W)
			// so it doesn't preserve linear and angular momentum, however its more accurate 
			// and is recommended by [Koschier2019]
			particles[i].rho_star += dt * dot(particles[j].dv * (vi - vj), kernel.W_grad(xi - xj));
		}
		// Boundary
		for (int j = 0; j < boundary_particles.size(); j++) {
			const Vec3& xj = boundary_particles[j].pos;
			const Vec3& vj = boundary_particles[j].pos;
			const Vec3 del_w = kernel.W_grad(xi - xj);
			const Vec3 term2 = dv_div_rho2 * del_w;
			if (del_w.x != 0 || del_w.y != 0 || del_w.z != 0) {
				int a = 4;
			}
			particles[i].aii -= boundary_particles[j].dv * dot((particles[i].dv_div_rhoSqr_grad + term2), del_w);
			particles[i].rho_star += dt * dot(boundary_particles[j].dv * (vi - vj), del_w);
		}
		particles[i].pressure = particles[i].old_pressure;
	}
}

void SPHSimulator::solve_pressure(float time_step) {
	// Apply relaxed Jacobi iteration with omega = 0.5
	const Real omega = 0.5;
	const int max_iterations = 100;
	const Real max_error = 0.05; // percent
	const Real eta = max_error * 0.01 * rho_0;
	int iterations = 0;
	Real avg_rho_err;
	bool terminate = false;
	while (!terminate && (iterations < max_iterations)) {
		avg_rho_err = 0;
		terminate = true;
		// Start iteration
		// [Koschier2019] Eq 40 && 41: Calculate acceleration from pressure gradient
		// As discussed above, here we are using symmetry preserving variant of 
		// SPH of the gradient
		// Calculate pressure gradient:
		for (int i = 0; i < particles.size(); i++) {
			const Vec3& xi = particles[i].pos;
			particles[i].dv_divRhoSqr_p_grad = 0;
			// Fluid
			for (int j = 0; j < particles.size(); j++) {
				if (i == j) continue;
				const Real rho = particles[j].rho / rho_0;
				const Real rho2 = rho * rho;
				const Vec3& xj = particles[j].pos;
				const Vec3 del_w = kernel.W_grad(xi - xj);
				particles[i].dv_divRhoSqr_p_grad -= 
					particles[j].dv / rho2 * particles[j].old_pressure * del_w;
			}
			// Pressure assumed 0 at boundaries
		}
		// Intuition
		// (pressure i +  i neighbor's pressure j ) - (pressure j +
		// j neighbor's pressure k's)
		// -> (i neighbors pressure j - pressure_j ) - 
		//    ( j neighbor's pressure k's - pressure i == all k except i)
		for (int i = 0; i < particles.size(); i++) {
			const Vec3& xi = particles[i].pos;
			const Real aii = particles[i].aii;
			const Real rho = particles[i].rho / rho_0;
			const Real rho2 = rho * rho;
			const Real dv_div_rho2 = particles[i].dv / rho2;
			particles[i].pressure = 0;
			Real sum = 0;
			// Fluid
			for (int j = 0; j < particles.size(); j++) {
				if (i == j) continue;
				const Vec3& xj = particles[j].pos;
				const Vec3 del_w = kernel.W_grad(xi - xj);
				const Vec3 term2 = dv_div_rho2 * del_w;
				sum += particles[j].dv * dot(
					particles[i].dv_divRhoSqr_p_grad -
					particles[j].dv_div_rhoSqr_grad * particles[j].old_pressure
					- (particles[j].dv_divRhoSqr_p_grad - term2), del_w);
			}
			// Boundary
			for (int j = 0; j < boundary_particles.size(); j++) {
				const Vec3& xj = boundary_particles[j].pos;
				// Only include fluid pressures
				const Vec3 del_w = kernel.W_grad(xi - xj);
				if (del_w.x != 0 || del_w.y != 0 || del_w.z != 0) {
					int a = 4;
				}
				sum += boundary_particles[j].dv * dot(
					particles[i].dv_divRhoSqr_p_grad
					, del_w);
			}
			// Solve
			// Ap = s
			// Where s = rho_0 - rho*
			const float dt2 = time_step * time_step;
			const Real s = 1 - particles[i].rho_star; // (rho0 - rho*) / rho0
			const Real denom = particles[i].aii * dt2;
			// [Koschier2019] Eq. 46 & 48
			if (fabs(denom) > 1.0e-9) {
				auto test = (s - dt2 * sum) / denom;
				particles[i].pressure = fmax(0, (1 - omega) * particles[i].old_pressure + 
					omega / denom * (s - dt2 * sum));
			} else {
				particles[i].pressure = 0;
			}
			if (particles[i].pressure != 0) {
				if (i == 1) {
					int j = 1;
				}
				const Real ap =  dt2 * (particles[i].aii + sum);
				const Real new_rho = rho_0 * (ap - s) + rho_0;
				avg_rho_err += fabs(new_rho - rho_0);
			}
		}
		for (int i = 0; i < particles.size(); i++) {
			particles[i].old_pressure = particles[i].pressure;
		}
		avg_rho_err /= particles.size();
		// End iteration
		iterations++;
		terminate = terminate && (avg_rho_err <= eta);
	}
}

void SPHSimulator::integrate(float dt) {
	if (!particles.size()) {
		return;
	}
	// Get acceleration & integrate
	for (int i = 0; i < particles.size(); i++) {
		const Vec3& xi = particles[i].pos;
		const Real rho = particles[i].rho / rho_0;
		const Real rho2 = rho * rho;
		const Real p_div_rho2 = particles[i].pressure / rho2;
		particles[i].accel = 0;
		// Acceleration from fluid pressure
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Real rho_j = particles[j].rho / rho_0;
			const Real rho2_j = rho_j * rho_j;
			const Real p_div_rho2_j = particles[j].pressure / rho2_j;
			const Vec3 del_w = kernel.W_grad(particles[i].pos - particles[j].pos);
			particles[i].accel -= particles[j].dv *
				(p_div_rho2 + p_div_rho2_j) * del_w;
		}
		// Mirror pressure, see [Akinci2012] and [Koschier2019]
		for (int j = 0; j < boundary_particles.size(); j++) {
			const Vec3& xj = boundary_particles[j].pos;
			auto del_w = kernel.W_grad(xi - xj);
			Vec3 a = boundary_particles[j].dv * p_div_rho2 * del_w;
			particles[i].accel -= a;
			particles[i].force += particles[i].mass * a;
		}
		particles[i].vel += particles[i].accel * dt;
		particles[i].pos += particles[i].vel * dt;
	}
}


const char* SPHSimulator::getTestCasesStr() {
	return "Box";
}

void SPHSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	this->context = DUC->g_pd3dImmediateContext;
	particles.clear();
	boundary_particles.clear();
	init_sim();
}

void SPHSimulator::reset() {
	mouse.x = mouse.y = 0;
	track_mouse.x = track_mouse.y = 0;
	old_track_mouse.x = old_track_mouse.y = 0;

}

void SPHSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	switch (m_iTestCase) {
	case 0:
	{
		DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.5));
		for (int i = 0; i < particles.size(); i++) {
			const float r = particle_radius;
			DUC->drawSphere(particles[i].pos, { r, r, r });
		}
		DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.1));
		for (int i = 0; i < boundary_particles.size(); i++) {
			const float r = particle_radius;
			DUC->drawSphere(boundary_particles[i].pos, { r, r, r });
		}
	}
	break;
	default:
		break;
	}
}

void SPHSimulator::notifyCaseChanged(int testCase) {
}

void SPHSimulator::externalForcesCalculations(float timeElapsed) {
}

void SPHSimulator::onClick(int x, int y) {
}

void SPHSimulator::onMouse(int x, int y) {
}

void SPHSimulator::pass_time_step_variable(float time_step) {
	this->time_step = &time_step;
}
