#include "SPHSimulator.h"
#include "util/util.h"
#define DIM 1
SPHSimulator::SPHSimulator() {
	m_iTestCase = 0;
	num_particles = DIM * DIM;
	gravity = Vec3(0, -9.81, 0);
	particle_radius = 0.5;
	rho_0 = 100;
	// For kernel
	support_radius = 4 * particle_radius;
	kernel.init(support_radius);
	particles.reserve(num_particles);
	init_sim();
}

void SPHSimulator::init_sim() {
	constexpr double MASS = 1;
	// Init fluid particles
	const float two_r = particle_radius * 2.0f;
	for (float i = -particle_radius * DIM; i <= particle_radius * DIM; i += two_r) {
		for (float j = -particle_radius * DIM; j <= particle_radius * DIM; j += two_r) {
			//for (float k = -particle_radius * DIM; k <= particle_radius * DIM; k += two_r)
			particles.push_back(Particle(MASS, Vec3(i, /*-0.75 + 2.1 * two_r*/ +j, 0)));
		}
	}
	 //Init boundary particles ( 2D for now)
	for (int k = 0; k < 2; k++) {
		// Bottom
		for (float i = -particle_radius * 4 * DIM; i <= particle_radius * 4 * DIM; i += two_r) {
			boundary_particles.push_back(Particle(MASS, Vec3(i, -0.5 - k * two_r, 0)));
			boundary_particles.back().density = rho_0;
		}
		// Left
		for (float i = -particle_radius * 4 * DIM; i <= particle_radius * 4 * DIM; i += two_r) {
			Real l_boundary = -6 * particle_radius * DIM;
			Real b_boundary = -0.5 + (3 * particle_radius) * DIM;
			boundary_particles.push_back(Particle(MASS, Vec3(l_boundary + (k - 1) * two_r, b_boundary + i, 0)));
			boundary_particles.back().density = rho_0;
		}
		// Right
		for (float i = -particle_radius * 4 * DIM; i <= particle_radius * 4 * DIM; i += two_r) {
			Real r_boundary = 6 * particle_radius * DIM;
			Real b_boundary = -0.5 + (3 * particle_radius) * DIM;
			boundary_particles.push_back(Particle(MASS, Vec3(r_boundary + (k - 1) * two_r, b_boundary + i, 0)));
			boundary_particles.back().density = rho_0;
		}

	}
	/*for (int k = 0; k < 1; k++) {
		for (float i = -particle_radius * 2 * DIM; i <= particle_radius * 2 * DIM; i += two_r) {
			Particle boundary(MASS, Vec3(i, -0.75 - k * two_r, 0));
			boundary.density = rho_0;
			boundary_particles.push_back(boundary);

		}
	}*/
	compute_boundary_masses();
}

void SPHSimulator::compute_boundary_masses() {
	// Compute masses of boundaries ( [Akinci2012] eqn. 4 & 5)
	for (int i = 0; i < boundary_particles.size(); i++) {
		Real delta = kernel.W(0);
		const Vec3& xi = boundary_particles[i].pos;
		for (int j = 0; j < boundary_particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = boundary_particles[j].pos;
			delta += kernel.W(norm(xi - xj));
		}
		boundary_particles[i].mass = rho_0 / delta;
	}
}

void SPHSimulator::simulateTimestep(float time_step) {
	// Implements IISPH as described in [Koschier2019] pp. 14
	compute_density();
	compute_non_pressure_forces();
	enforce_continuity(time_step);
	solve_pressure(time_step);
	integrate(time_step);
}

void SPHSimulator::compute_density() {
	// Compute density and clear accelerations/forces

	for (int i = 0; i < particles.size(); i++) {
		Real density = kernel.W(0);
		const Vec3& xi = particles[i].pos;
		// Fluid
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = particles[j].pos;
			density += particles[j].mass * kernel.W(norm(xi - xj));
		}
		// Boundary
		for (int j = 0; j < boundary_particles.size(); j++) {

			const Vec3& xj = boundary_particles[j].pos;
			auto krnel = kernel.W(norm(xi - xj));
			density += boundary_particles[j].mass * krnel;
		}
		particles[i].density = density;
		particles[i].accel = gravity;
		particles[i].force = 0;
	}
}

void SPHSimulator::compute_non_pressure_forces() {
	// TODO
}

void SPHSimulator::enforce_continuity(float time_step) {
	// [Koschier2019] Eq. 49 : calculate A_ii
	Vec3 term2;
	Real dp = 0;
	const float dt2 = time_step * time_step;
	for (int i = 0; i < particles.size(); i++) {
		Vec3& term1 = particles[i].m_divRhoSqr_grad;
		term1 = 0;
		const Vec3 xi = particles[i].pos;

		// Fluid
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = particles[j].pos;
			Real rho2 = particles[j].density * particles[j].density;
			term1 += (particles[j].mass / rho2) * kernel.W_grad(xi - xj);
		}
		// Boundary
		for (int j = 0; j < boundary_particles.size(); j++) {
			const Vec3& xj = boundary_particles[j].pos;
			Real rho2 = boundary_particles[j].density * boundary_particles[j].density;
			auto krnel = kernel.W_grad(xi - xj);
			term1 += (boundary_particles[j].mass / rho2) * krnel;
		}

		//  v* = v + dt * a_nonpressure
		particles[i].vel += time_step * particles[i].accel;
	}
	for (int i = 0; i < particles.size(); i++) {
		const Vec3 xi = particles[i].pos;
		const Vec3 vi = particles[i].vel;
		Real& aii = particles[i].aii;
		Real& rho_star = particles[i].rho_star;
		aii = 0;
		rho_star = particles[i].density;

		// Fluid
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Vec3& xj = particles[j].pos;
			const Vec3& vj = particles[j].pos;
			Real rho2 = particles[i].density * particles[i].density;
			dp = particles[i].mass / rho2;
			term2 = dp * kernel.W_grad(xi - xj);
			//aii -= dt2 * dot((particles[i].dij + term2), kernel.W_grad(xi - xj));
			auto krnel = kernel.W_grad(xi - xj);
			aii -= particles[j].mass * dot((particles[i].m_divRhoSqr_grad + term2), krnel);

			// While we're at it, calculate predicted density according to the continuity 
			// equation i.e:
			// rho* = rho - dt * rho * div(v*)
			// where v* = v + dt * a_nonpressure
			// Note that div(v_i) ~ \sum_j (m_j / rho_j) * (vj - vi) * grad(W)
			// as opposed to rho_i * \sum_j m_j * (v_i / rho_i^2 + v_j / rho_j^2) * grad(W)
			// so it doesn't preserve linear and angular momentum, however its more accurate 
			// and is recommended by [Koschier2019]

			rho_star += time_step * dot(particles[j].mass * (vi - vj), kernel.W_grad(xi - xj));
		}

		// Boundary
		for (int j = 0; j < boundary_particles.size(); j++) {
			const Vec3& xj = boundary_particles[j].pos;
			const Vec3& vj = boundary_particles[j].pos;
			auto krnel = kernel.W_grad(xi - xj);
			term2 = dp * krnel;
			aii -= boundary_particles[j].mass * dot((particles[i].m_divRhoSqr_grad + term2), krnel);
			rho_star += time_step * dot(boundary_particles[j].mass * (vi - vj), kernel.W_grad(xi - xj));
		}
		particles[i].pressure = particles[i].old_pressure;
	}
}

void SPHSimulator::solve_pressure(float time_step) {
	// Apply relaxed Jacobi iteration with omega = 0.5
	constexpr Real omega = 0.5;
	const int max_iterations = 100;
	const Real max_error = 0.05; // percent
	int iterations = 0;
	Real avg_rho_err;
	bool terminate = false;
	const Real eta = max_error * 0.01 * rho_0;
	while (!terminate && (iterations < max_iterations)) {
		avg_rho_err = 0;
		terminate = true;
		// Start iteration
		// [Koschier2019] Eq 40 && 41: Calculate acceleration from pressure gradient
		// As discussed above, here we are using symmetry preserving variant of 
		// SPH of the gradient
		// Calculate pressure gradient:

		for (int i = 0; i < particles.size(); i++) {
			Vec3& term1 = particles[i].m_divRhoSqr_p_grad;
			const Vec3& xi = particles[i].pos;
			term1 = Vec3();
			for (int j = 0; j < particles.size(); j++) {
				if (i == j) continue;
				Real rho2 = particles[j].density * particles[j].density;
				const Vec3& xj = particles[j].pos;
				term1 -= particles[j].mass / rho2 * particles[j].old_pressure *
					kernel.W_grad(xi - xj);
			}
		}

		// Pressure assumed 0 at the boundaries

		// Intuition
		// (pressure i +  i neighbor's pressure j ) - (pressure j +
		// j neighbor's pressure k's)
		// -> (i neighbors pressure j - pressure_j ) - 
		//    ( j neighbor's pressure k's - pressure i == all k except i)
		for (int i = 0; i < particles.size(); i++) {
			const Vec3& xi = particles[i].pos;
			Real sum = 0;
			// Fluid
			for (int j = 0; j < particles.size(); j++) {
				if (i == j) continue;
				const Vec3& xj = particles[j].pos;
				Real rho2 = particles[j].density * particles[j].density;
				const Vec3 term2 = particles[i].old_pressure *
					particles[i].mass / rho2 * kernel.W_grad(xi - xj);
				sum += particles[j].mass * dot(
					particles[i].m_divRhoSqr_p_grad -
					particles[j].m_divRhoSqr_grad * particles[j].old_pressure
					- (particles[j].m_divRhoSqr_p_grad - term2), kernel.W_grad(xi - xj));
			}
			// Boundary
			for (int j = 0; j < boundary_particles.size(); j++) {
				const Vec3& xj = boundary_particles[j].pos;
				// Only include fluid pressures
				sum += boundary_particles[j].mass * dot(
					particles[i].m_divRhoSqr_p_grad
					, kernel.W_grad(xi - xj));
			}

			// Solve
			// Ap = s
			// Where s = rho_0 - rho*
			const float dt2 = time_step * time_step;
			const Real s = rho_0 - particles[i].rho_star;
			const Real denom = particles[i].aii * dt2;
			// [Koschier2019] Eq. 46 & 48
			Real& p_new = particles[i].pressure;
			const Real& p_old = particles[i].old_pressure;
			if (fabs(denom) > 1.0e-9) {
				p_new = fmax(0, (1 - omega) * p_old + omega / denom * (s - dt2 * sum));
			} else {
				p_new = 0;
			}
			if (p_new != 0) {
				const Real ap = p_new * dt2 * (particles[i].aii + sum);
				const Real rho_err = (ap +
					particles[i].rho_star - rho_0) / rho_0;
				avg_rho_err += (rho_err);
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
		const Real dp = particles[i].pressure / (particles[i].density * particles[i].density);
		particles[i].accel = 0;
		// Acceleration from fluid pressure
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
			const Real rhoj2 = particles[j].density * particles[j].density;
			particles[i].accel -= particles[j].mass *
				(dp + particles[j].pressure / rhoj2) *
				kernel.W_grad(particles[i].pos - particles[j].pos);
		}
		// Mirror pressure, see [Akinci2012] and [Koschier2019]
		for (int j = 0; j < boundary_particles.size(); j++) {
			const Vec3& xj = boundary_particles[j].pos;
			auto krnel = kernel.W_grad(xi - xj);
			Vec3 a = boundary_particles[j].mass * dp * krnel;
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
	/*	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.1));
		for (int i = 0; i < boundary_particles.size(); i++) {
			const float r = particle_radius;
			DUC->drawSphere(boundary_particles[i].pos, { r, r, r });
		}*/
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
