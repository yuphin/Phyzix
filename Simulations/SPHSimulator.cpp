#include "SPHSimulator.h"
#include "util/util.h"
#define DIM 3
#define USE_NEIGHBORHOOD 1
// TODO : Better boundary sampling
// Fix sorting for neighbor search
SPHSimulator::SPHSimulator() {
	is_2d = false;

}

void SPHSimulator::init_sim(const Vec3& gravity, const Real rho_0,
	const int num_particles, const Real particle_radius) {
	this->gravity = gravity;
	this->rho_0 = rho_0;
	this->num_particles = num_particles;
	this->particle_radius = particle_radius;
	this->boundary_radius = particle_radius;
	const auto& pr = particle_radius;

	dv = is_2d ? 0.8 * M_PI * pr * pr : 0.8 * (4.0 * M_PI / 3) * pr * pr * pr;
	dm = rho_0 * dv;
	// For kernel
	support_radius = 4 * particle_radius;
	kernel.init(support_radius);
	particles.reserve(num_particles);
	srand(time(NULL));
	particles.clear();
	boundary_particles.clear();
	neighborhood_searcher.reset();
	neighborhood_searcher =
		std::make_unique<NeighborhoodSearcher>(support_radius);
}

void SPHSimulator::init_particles(const Vec3& x_offset, const Vec3& y_offset,
	const Vec3& z_offset, int DIM_X, int DIM_Y, int DIM_Z) {
	const Real two_r = particle_radius * 2.0f;
	for (int i = 0; i < DIM_X; i++) {
		for (int j = 0; j < DIM_Y; j++) {
			if (is_2d) {
				const Vec3 pos = Vec3(i * two_r, j * two_r, 0);

				particles.push_back(Particle(dm, dv,
					x_offset + y_offset + pos));
			} else {
				for (int k = 0; k < DIM_Z; k++) {
					Vec3 pos = Vec3(i * two_r, -1 + j * two_r, k * two_r);
					particles.push_back(Particle(dm, dv,
						x_offset + y_offset + (is_2d ? Vec3() : z_offset) + pos));
				}
			}
		}
	}
	neighborhood_searcher->register_set(particles);
}

void SPHSimulator::compute_boundary_volumes() {
	// Compute masses of boundaries ( [Akinci2012] eqn. 4 & 5)
	for (int i = 0; i < boundary_particles.size(); i++) {
		Real delta = kernel.W(0);
		const Vec3& xi = boundary_particles[i].pos;
#if USE_NEIGHBORHOOD
		const auto& boundary_neighbors = neighborhood_searcher->
			particle_sets[BND_ID].neighbor_idxs[BND_ID][i];
		if (neighborhood_searcher->particle_sets[BND_ID].neighbor_idxs.size() > MV_BND_ID) {
			auto& mov_boundary_neighbors = neighborhood_searcher->
				particle_sets[BND_ID].neighbor_idxs[MV_BND_ID][i];
			//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic
			for (const auto j : mov_boundary_neighbors) {
				const Vec3& xj = moving_boundary_particles[j].pos;
				delta += kernel.W(norm(xi - xj));
			}
		}

		for (const auto j : boundary_neighbors) {
#else
		for (int j = 0; j < boundary_particles.size(); j++) {
			if (i == j) continue;
#endif
			const Vec3& xj = boundary_particles[j].pos;
			delta += kernel.W(norm(xi - xj));
		}
		boundary_particles[i].dv = 1.0 / delta;
	}
#if USE_NEIGHBORHOOD
	for (int i = 0; i < moving_boundary_particles.size(); i++) {
		Real delta = kernel.W(0);
		const Vec3& xi = moving_boundary_particles[i].pos;
		const auto& boundary_neighbors = neighborhood_searcher->
			particle_sets[MV_BND_ID].neighbor_idxs[BND_ID][i];
		auto& mov_boundary_neighbors = neighborhood_searcher->
			particle_sets[MV_BND_ID].neighbor_idxs[MV_BND_ID][i];
		//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic
		for (const auto j : boundary_neighbors) {
			const Vec3& xj = boundary_particles[j].pos;
			delta += kernel.W(norm(xi - xj));
		}
		for (const auto j : mov_boundary_neighbors) {
			const Vec3& xj = moving_boundary_particles[j].pos;
			delta += kernel.W(norm(xi - xj));
		}
		moving_boundary_particles[i].dv = 1.0 / delta;
	}
#endif
}

void SPHSimulator::simulateTimestep(float ts) {
	// Implements IISPH as described in [Koschier2019] pp. 14
	update_time_step(time_step);
	neighborhood_searcher->find_neighborhoods();
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
	float ts = cfl_k * 0.4 * two_r / max;
	if (ts < 1e-5) {
		time_step = 1e-5;
	} else if (ts <= time_step) {
		time_step = ts;
	}
}

void SPHSimulator::compute_density() {
	// Compute density and clear accelerations/forces

	for (int i = 0; i < particles.size(); i++) {
		Real density = particles[i].dv * kernel.W(0);
		const Vec3& xi = particles[i].pos;
		// Fluid
#if USE_NEIGHBORHOOD
		auto& neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[FLD_ID][i];
		std::sort(neighbors.begin(), neighbors.end());
		for (const auto j : neighbors) {
#else
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
#endif
			const Vec3& xj = particles[j].pos;
			density += particles[j].dv * kernel.W(norm(xi - xj));
		}
		// Boundary
#if USE_NEIGHBORHOOD
		const auto& boundary_neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[BND_ID][i];
		if (neighborhood_searcher->particle_sets[FLD_ID].neighbor_idxs.size() > MV_BND_ID) {
			auto& mov_boundary_neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[MV_BND_ID][i];
			//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic
			for (const auto j : mov_boundary_neighbors) {
				const Vec3& xj = moving_boundary_particles[j].pos;
				auto krnel = kernel.W(norm(xi - xj));
				density += particles[i].gamma1 * moving_boundary_particles[j].dv * krnel;
			}
		}
		for (const auto j : boundary_neighbors) {
#else
		for (int j = 0; j < boundary_particles.size(); j++) {
#endif
			const Vec3& xj = boundary_particles[j].pos;
			auto krnel = kernel.W(norm(xi - xj));
			density += particles[i].gamma1 * boundary_particles[j].dv * krnel;
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

		const Real rho_i = particles[i].rho / rho_0;
		Real rho2_i = rho_i * rho_i;
		//  v* = v + dt * a_nonpressure
		particles[i].vel += dt * particles[i].accel;
		// Fluid
#if USE_NEIGHBORHOOD
		auto& neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[FLD_ID][i];
		std::sort(neighbors.begin(), neighbors.end());
		for (const auto j : neighbors) {
#else
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
#endif
			const Vec3& xj = particles[j].pos;
			term1 += (particles[j].dv / rho2_i) * kernel.W_grad(xi - xj);
		}
		// Boundary
#if USE_NEIGHBORHOOD

		const auto& boundary_neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[BND_ID][i];
		if (neighborhood_searcher->particle_sets[FLD_ID].neighbor_idxs.size() > MV_BND_ID) {
			auto& mov_boundary_neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[MV_BND_ID][i];
			//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic
			for (const auto j : mov_boundary_neighbors) {
				const Vec3& xj = moving_boundary_particles[j].pos;
				const Vec3 del_w = kernel.W_grad(xi - xj);
				term1 += (moving_boundary_particles[j].dv / rho2_i) * del_w;
			}
		}

		for (const auto j : boundary_neighbors) {
#else
		for (int j = 0; j < boundary_particles.size(); j++) {
#endif
			const Vec3& xj = boundary_particles[j].pos;
			const Vec3 del_w = kernel.W_grad(xi - xj);
			term1 += (boundary_particles[j].dv / rho2_i) * del_w;
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
		particles[i].old_pressure = particles[i].pressure;
		// Fluid
#if USE_NEIGHBORHOOD
		auto& neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[FLD_ID][i];
		std::sort(neighbors.begin(), neighbors.end());
		for (const auto j : neighbors) {
#else
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
#endif
			const Vec3& xj = particles[j].pos;
			const Vec3& vj = particles[j].vel;
			const Vec3 del_w = kernel.W_grad(xi - xj);
			const Vec3 term2 = dv_div_rho2 * del_w;
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
#if USE_NEIGHBORHOOD
		const auto& boundary_neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[BND_ID][i];
		if (neighborhood_searcher->particle_sets[FLD_ID].neighbor_idxs.size() > MV_BND_ID) {
			auto& mov_boundary_neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[MV_BND_ID][i];
			//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic
			for (const auto j : mov_boundary_neighbors) {
				const Vec3& xj = moving_boundary_particles[j].pos;
				const Vec3& vj = moving_boundary_particles[j].vel;
				const Vec3 del_w = kernel.W_grad(xi - xj);
				const Vec3 term2 = dv_div_rho2 * del_w;
				particles[i].aii -= moving_boundary_particles[j].dv *
					dot((particles[i].dv_div_rhoSqr_grad + term2), del_w);
				particles[i].rho_star += dt * dot(moving_boundary_particles[j].dv * (vi - vj), del_w);
			}
		}
		for (const auto j : boundary_neighbors) {
#else
		for (int j = 0; j < boundary_particles.size(); j++) {
#endif
			const Vec3& xj = boundary_particles[j].pos;
			const Vec3& vj = boundary_particles[j].vel;
			const Vec3 del_w = kernel.W_grad(xi - xj);
			const Vec3 term2 = dv_div_rho2 * del_w;
			particles[i].aii -= boundary_particles[j].dv * dot((particles[i].dv_div_rhoSqr_grad + term2), del_w);
			particles[i].rho_star += dt * dot(boundary_particles[j].dv * (vi - vj), del_w);
		}
		if (particles[i].aii > 0) {
			printf("Warning: Particle %d has A_ii >0\n", i);
		}
	}
}

void SPHSimulator::solve_pressure(float time_step) {
	// Apply relaxed Jacobi iteration with omega = 0.5
	const Real omega = 0.5;
	const int max_iterations = 100;
	const int min_iterations = 2;
	const Real max_error = 0.01; // percent
	const Real eta = max_error * 0.01 * rho_0;
	int iterations = 0;
	Real avg_rho_err = 0;
	bool terminate = false;
	// TODO: Adaptive iteration
	while ((!terminate || min_iterations > iterations) && (iterations < max_iterations)) {
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
#if USE_NEIGHBORHOOD
			auto& neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[FLD_ID][i];
			std::sort(neighbors.begin(), neighbors.end());
			for (const auto j : neighbors) {
#else
			for (int j = 0; j < particles.size(); j++) {
				if (i == j) continue;
#endif
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
#if USE_NEIGHBORHOOD
			auto& neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[FLD_ID][i];
			std::sort(neighbors.begin(), neighbors.end());
			for (const auto j : neighbors) {
#else
			for (int j = 0; j < particles.size(); j++) {
				if (i == j) continue;
#endif
				const Vec3& xj = particles[j].pos;
				const Vec3 del_w = kernel.W_grad(xi - xj);
				const Vec3 term2 = dv_div_rho2 * del_w;
				sum += particles[j].dv * dot(
					particles[i].dv_divRhoSqr_p_grad -
					particles[j].dv_div_rhoSqr_grad * particles[j].old_pressure
					- (particles[j].dv_divRhoSqr_p_grad - term2 * particles[i].old_pressure),
					del_w);
			}
			// Boundary
#if USE_NEIGHBORHOOD
			const auto& boundary_neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[BND_ID][i];
			if (neighborhood_searcher->particle_sets[FLD_ID].neighbor_idxs.size() > MV_BND_ID) {
				auto& mov_boundary_neighbors = neighborhood_searcher->
					particle_sets[FLD_ID].neighbor_idxs[MV_BND_ID][i];
				//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic

				for (const auto j : mov_boundary_neighbors) {
					const Vec3& xj = moving_boundary_particles[j].pos;
					// Only include fluid pressures
					const Vec3 del_w = kernel.W_grad(xi - xj);
					sum += moving_boundary_particles[j].dv * dot(
						particles[i].dv_divRhoSqr_p_grad, del_w);
				}
			}
			for (const auto j : boundary_neighbors) {
#else
			for (int j = 0; j < boundary_particles.size(); j++) {
#endif
			//for (int j = 0; j < boundary_particles.size(); j++) {
				const Vec3& xj = boundary_particles[j].pos;
				// Only include fluid pressures
				const Vec3 del_w = kernel.W_grad(xi - xj);
				sum += boundary_particles[j].dv * dot(
					particles[i].dv_divRhoSqr_p_grad, del_w);
			}
			// Solve
			// Ap = s
			// Where s = rho_0 - rho*
			const float dt2 = time_step * time_step;
			const Real s = 1.0 - particles[i].rho_star; // (rho0 - rho*) / rho0
			const Real denom = particles[i].aii * dt2;
			// [Koschier2019] Eq. 46 & 48
			Real test;
			if (fabs(denom) > 1.0e-9) {
				test = (s - dt2 * sum) / denom;
				particles[i].pressure = fmax(0.0, (1.0 - omega) * particles[i].old_pressure +
					omega / denom * (s - dt2 * sum));
			} else {
				particles[i].pressure = 0;
			}
			if (particles[i].pressure != 0) {
				const Real ap = dt2 * (particles[i].aii * particles[i].pressure + sum);
				const Real new_rho = rho_0 * (ap - s) + rho_0;
				avg_rho_err += abs(new_rho - rho_0);
			}
		}
		for (int i = 0; i < particles.size(); i++) {
			particles[i].old_pressure = particles[i].pressure;
		}
		avg_rho_err /= particles.size();
		// End iteration
		iterations++;
		if (iterations == max_iterations) {
			printf("Rip %lf\n", avg_rho_err);
		}
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
#if USE_NEIGHBORHOOD
		auto& neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[FLD_ID][i];
		std::sort(neighbors.begin(), neighbors.end());
		for (const auto j : neighbors) {
#else
		for (int j = 0; j < particles.size(); j++) {
			if (i == j) continue;
#endif
			const Real rho_j = particles[j].rho / rho_0;
			const Real rho2_j = rho_j * rho_j;
			const Real p_div_rho2_j = particles[j].pressure / rho2_j;
			const Vec3 del_w = kernel.W_grad(particles[i].pos - particles[j].pos);
			particles[i].accel -= particles[j].dv *
				(p_div_rho2 + p_div_rho2_j) * del_w;
		}
		// Mirror pressure, see [Akinci2012] and [Koschier2019]
#if USE_NEIGHBORHOOD
		auto& boundary_neighbors = neighborhood_searcher->
			particle_sets[FLD_ID].neighbor_idxs[BND_ID][i];
		if (neighborhood_searcher->particle_sets[FLD_ID].neighbor_idxs.size() > MV_BND_ID) {
			auto& mov_boundary_neighbors = neighborhood_searcher->
				particle_sets[FLD_ID].neighbor_idxs[MV_BND_ID][i];
			//std::sort(mov_boundary_neighbors.begin(), mov_boundary_neighbors.end()); //jic
			for (const auto j : mov_boundary_neighbors) {
				const Vec3& xj = moving_boundary_particles[j].pos;
				auto del_w = kernel.W_grad(xi - xj);
				Vec3 a = moving_boundary_particles[j].dv * p_div_rho2 * del_w;
				particles[i].accel -= a;
				moving_boundary_particles[j].add_force(particles[i].mass * a);
			}
		}
		for (const auto j : boundary_neighbors) {
#else
		for (int j = 0; j < boundary_particles.size(); j++) {
#endif
			const Vec3& xj = boundary_particles[j].pos;
			auto del_w = kernel.W_grad(xi - xj);
			Vec3 a = boundary_particles[j].dv * p_div_rho2 * del_w;
			particles[i].accel -= a;
		}
		particles[i].vel += particles[i].accel * dt;
		particles[i].pos += particles[i].vel * dt;
	}
}


const char* SPHSimulator::getTestCasesStr() {
	return "Dam, Box";
}

void SPHSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
	this->context = DUC->device_context;
}

void SPHSimulator::reset() {
	mouse.x = mouse.y = 0;
	track_mouse.x = track_mouse.y = 0;
	old_track_mouse.x = old_track_mouse.y = 0;

}

void SPHSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext) {
	const float br = boundary_radius;
	const float pr = particle_radius;
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.1));

	for (int i = 0; i < boundary_particles.size(); i++) {
		if (boundary_particles[i].visible) {
			DUC->drawSphere(boundary_particles[i].pos, { br, br, br });
		}
	}
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.5));
	for (int i = 0; i < particles.size(); i++) {
		DUC->drawSphere(particles[i].pos, { pr, pr, pr });
	}
	switch (m_iTestCase) {
	case 0:
	{
	}
	break;
	case 1:
	{
		DUC->drawRigidBody(box_t, box_r, box_s);
	}
	break;
	default:
		break;
	}
}

void SPHSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	auto num_particles = is_2d ? DIM * DIM : DIM * DIM * DIM;
	init_sim(Vec3(0, -9.81, 0), 1000, num_particles, 0.3);
	const Vec3 offset = Vec3{ 0, 0, -0.25 };
	const Real two_r = particle_radius * 2.0f;
	auto n = is_2d ? Vec3() : Vec3(0, 0, -DIM * two_r);
	int h_count = 6;
	int d_count = is_2d ? 0 : (4 / boundary_radius);
	auto l = Vec3(-h_count * boundary_radius, 0, 0);
	time_step = 0.01;
	const int bnd_set_idx = 1;
	switch (testCase) {
	case 0:
	{
		init_particles(Vec3(-DIM / 2 * two_r, 0, 0),
			Vec3(0, -0.25 + DIM * two_r, 0),
			Vec3(0, 0, -DIM * two_r) + 1.5 * offset, DIM, DIM, DIM);
		int v_count = (3 + 0.75) / boundary_radius;
		auto t = Vec3(0, -1 + v_count * boundary_radius, 0);
		auto r = Vec3(h_count * boundary_radius, 0, 0);
		for (int k = 0; k < 1; k++) {
			for (int i = 0; i < 2 * h_count; i++) {
				for (int j = 0; j <= 2 * d_count; j++) {
					Vec3 pos = Vec3(i * boundary_radius, -0.75 - k * boundary_radius, j * boundary_radius);
					boundary_particles.push_back(Particle{ dm, 0, pos + l + n, rho_0,
						bnd_set_idx });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(Particle{ dm, 0, pos + l + n, rho_0,
						bnd_set_idx });
				}
			}
			for (int i = 0; i < v_count; i++) {
				for (int j = 0; j <= 2 * d_count; j++) {
					Vec3 pos = Vec3(-k * boundary_radius, -i * boundary_radius, j * boundary_radius);
					boundary_particles.push_back(Particle{ dm, 0, pos + l + t + n , rho_0,
						bnd_set_idx });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(Particle{ dm, 0, pos + l + t + n , rho_0,
						bnd_set_idx });
					pos.x = k * boundary_radius;
					pos.z = j * boundary_radius;
					boundary_particles.push_back(Particle{ dm, 0, pos + r + t + n , rho_0,
						bnd_set_idx });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(Particle{ dm, 0, pos + r + t + n , rho_0,
						bnd_set_idx });
				}
			}
			if (!is_2d) {
				auto nf = {
						n + 2 * Vec3(0, 0, d_count * boundary_radius) ,
						n - 2 * Vec3(0, 0, d_count * boundary_radius)
				};
				for (const auto& p : nf) {
					for (int i = 0; i < 2 * h_count; i++) {
						for (int j = 0; j < v_count; j++) {
							Vec3 pos = Vec3(i * boundary_radius, -j * boundary_radius, 0);
							moving_boundary_particles.push_back(Particle{ dm, 0, pos + l + t + p , rho_0,
								bnd_set_idx });
							pos.x = k * boundary_radius;
							moving_boundary_particles.push_back(Particle{ dm, 0, pos + r + t + p , rho_0,
								bnd_set_idx });
						}
					}

				}
			}

		}
		neighborhood_searcher->register_set(boundary_particles);
		neighborhood_searcher->register_set(moving_boundary_particles);
		neighborhood_searcher->find_neighborhoods();
		compute_boundary_volumes();
	}
	break;
	case 1:
	{
		init_particles(Vec3(-DIM / 2 * two_r + 3, 0, 0),
			Vec3(0, -0.25 + DIM * two_r, 0),
			Vec3(0, 0, -DIM * two_r - 2) + 1.5 * offset, DIM, DIM, DIM);
		h_count = 20;
		for (int k = 0; k < 3; k++) {

			for (int i = 0; i < 2 * h_count; i++) {
				for (int j = 0; j < 2 * d_count; j++) {
					Vec3 pos = Vec3(i * boundary_radius,
						-0.75 - k * boundary_radius,
						j * boundary_radius);
					boundary_particles.push_back(
						Particle{ dm, 0, pos + l + n, rho_0 , 2, /* visible */ k > 0 });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(
						Particle{ dm, 0, pos + l + n, rho_0, 2, /* visible */ k > 0 });
				}
			}
		}
		DUC->box->samples.clear();
		DUC->box->sample_mesh();
		samples = DUC->box->samples;
		for (int i = 0; i < samples.size(); i++) {
			samples[i] += box_t;
			samples[i] *= box_s;
			boundary_particles.push_back(Particle{ dm, 0, samples[i], rho_0 });
		}
		neighborhood_searcher->register_set(boundary_particles);
		neighborhood_searcher->find_neighborhoods();
		compute_boundary_volumes();

	}
	break;

	}
}

void SPHSimulator::externalForcesCalculations(float timeElapsed) {
}

void SPHSimulator::onClick(int x, int y) {
}

void SPHSimulator::onMouse(int x, int y) {
}

void SPHSimulator::pass_time_step_variable(float time_step) {
	this->time_step = time_step;
}
