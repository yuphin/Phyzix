#include "Sandbox.h"
#define DIM 5
Sandbox::Sandbox() {
	m_iTestCase = 0;

	collision_map[2] = collision_sphere_sphere;
	collision_map[3] = collision_box_sphere;
	collision_map[5] = collision_box_plane;
	collision_map[6] = collision_sphere_plane;

}
static int num_run = 0;
const char* Sandbox::getTestCasesStr()
{
	return "Many Bodies,2,3,4,5,6,Dam,Box,8,9,10,11";
}

void Sandbox::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	if (m_iTestCase >= 10) {
		sf->initUI(DUC);
	}
	else {
		TwAddVarRW(DUC->g_pTweakBar, "Render Planes", TW_TYPE_BOOLCPP, &render_planes, "");
		TwAddVarCB(DUC->g_pTweakBar, "Gravity", TW_TYPE_DIR3F, set_gravity, get_gravity, &gravity, "");
		TwAddButton(DUC->g_pTweakBar, "Create Random Box", addRandomBox, this, "");
		TwAddButton(DUC->g_pTweakBar, "Create Random Sphere", addRandomSphere, this, "");
	}
}

void TW_CALL Sandbox::addRandomBox(void* value) {
	static std::mt19937 eng;
	static std::uniform_real_distribution<float> randomer(-1.5, 2);
	((Sandbox*)value)->add_box({ randomer(eng), 1.0, randomer(eng) }, { 0.3, 0.3, 0.3 }, 2);
}

void TW_CALL Sandbox::addRandomSphere(void* value) {
	static std::mt19937 eng;
	static std::uniform_real_distribution<float> randomer(-1.5, 2);
	((Sandbox*)value)->add_sphere({ randomer(eng), 1.0, randomer(eng) }, 0.3, 2);
}


void TW_CALL Sandbox::get_gravity(void* value, void* client_data) {
	static_cast<float*> (value)[0] = static_cast<const Vec3*>(client_data)->x;
	static_cast<float*> (value)[1] = static_cast<const Vec3*>(client_data)->y;
	static_cast<float*> (value)[2] = static_cast<const Vec3*>(client_data)->z;
}

void TW_CALL Sandbox::set_gravity(const void* value, void* client_data) {
	static_cast<Vec3*> (client_data)->x = static_cast<const float*> (value)[0];
	static_cast<Vec3*> (client_data)->y = static_cast<const float*> (value)[1];
	static_cast<Vec3*> (client_data)->z = static_cast<const float*> (value)[2];
}

void Sandbox::reset() {}

void Sandbox::drawFrame(ID3D11DeviceContext* context) {
	if (m_iTestCase >= 10) {
		sf->drawFrame(context);
		return;
	}
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	for (int i = 0; i < rigid_bodies.size(); i++) {
		switch (rigid_bodies[i].type) {
		case RigidBodyType::CUBOID:
		{
			DUC->drawRigidBody(rigid_bodies[i].obj_to_world().toDirectXMatrix());
		}
		break;
		case RigidBodyType::SPHERE:
		{
			const auto& rad = rigid_bodies[i].offset;
			DUC->drawSphere(rigid_bodies[i].position, { rad,rad,rad });

		}
		break;
		case RigidBodyType::PLANE:
		{
			if (render_planes) {
				DUC->drawRigidBody(rigid_bodies[i].obj_to_world_plane_rendering().toDirectXMatrix());
			}
		}
		break;
		default:
			break;
		}
	}
	if (m_iTestCase >= 6) {
		const float br = sph->boundary_radius;
		const float pr = sph->particle_radius;
		DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.1));

		for (int i = 0; i < sph->boundary_particles.size(); i++) {
			if (sph->boundary_particles[i].visible) {
				DUC->drawSphere(sph->boundary_particles[i].pos, { br, br, br });
			}
		}
		for (int i = 0; i < sph->moving_boundary_particles.size(); i++) {
			if (sph->moving_boundary_particles[i].visible) {
				DUC->drawSphere(sph->moving_boundary_particles[i].pos, { br, br, br });
			}
		}
		DUC->setUpLighting(Vec3(0, 0, 0), Vec3(), 1, Vec3(0.1, 0.1, 0.5));
		for (int i = 0; i < sph->particles.size(); i++) {
			DUC->drawSphere(sph->particles[i].pos, { pr, pr, pr });
		}
	}

}

void Sandbox::notifyCaseChanged(int testCase) {
	//testCase = 7;
	m_iTestCase = testCase;
	rigid_bodies.clear();
	gravity = 0;
	mouse_force = 0;
	render_planes = false;
	const int bnd_set_idx = 1;
	if (testCase >= 10) {
		sf = std::make_unique<StableFluid>();
		sf->pass_time_step_variable(*timestep);
		sf->notifyCaseChanged(testCase - 10);
	}
	else if (testCase >= 6) {
		sph = std::make_unique<SPHSimulator>();
		sph_timestep = 0.01f;
		sph->pass_time_step_variable(sph_timestep);
	}
	switch (testCase) {
	case 0:
		*timestep = 0.001f;
		add_sphere({ 0.3,0,0 }, 0.5, 10);
		add_sphere({ 0,2,0 }, 0.5, 10);
		gravity = Vec3(0, -9.81f, 0);
		add_box({ -0.5,2,0 }, { 0.3,0.3,0.3 }, 10);
		add_box({ 1.0,3,1 }, { 0.3,0.3,0.3 }, 10);
		add_box({ -1.0,4,0.5 }, { 0.3,0.3,0.3 }, 10);
		add_box({ 0.5,1.5,-0.5 }, { 1,1,1 }, 10);
		add_sphere({ 0.3,2,0 }, 0.5, 10);
		add_sphere({ 0.45,3,0 }, 0.5, 10);
		add_sphere({ -0.45,4,-.5 }, 0.5, 10);
		break;
	case 1:
		*timestep = 0.005f;
		add_box({ 0.25, -0.5, 0 }, { 1, 0.5, 0.5 }, 2);
		add_box({ -0.25, 1, 0 }, { 1, 0.5, 0.5 }, 2);
		applyForceOnBody(0, { 0.25, -0.5, 0 }, { 0, 10,0 });
		applyForceOnBody(1, { -0.25, 1, 0 }, { 0, -35,0 });
		break;
	case 2:
		gravity = Vec3(0, -9.81f, 0);
		*timestep = 0.001f;
		/*	addRigidBody({ 0, 1, 0 }, { 1, 1, 1 }, 10);
		addRigidBody({ 2, 1, 0 }, { 1, 1, 1 }, 10);
		applyForceOnBody(0, { 0, 1.0, 0.0 }, { 100000,0,0 });
		applyForceOnBody(1, { 2, 1, 0.0 }, { -100000,0,0 });*/
		add_box({ 0.5, 0, 0 }, { 1, 1, 1 }, 10);
		add_box({ 0, 2, 0 }, { 1, 1, 1 }, 10);
		applyForceOnBody(1, { 0.0, 0.0, 0.0 }, { 0,-500,0 });
		break;
	case 3:
		*timestep = 0.001f;
		gravity = Vec3(0, -9.81f, 0);
		add_box({ 0, 0, 0 }, { 1, 1, 1 }, 10);
		//applyForceOnBody(0, { 0.0, 0.5, 0.5 }, { 1,1,0 });
		setOrientationOf(0, Quat(Vec3(0.5f, 0.8f, 1.0f), (float)(M_PI) * 0.5f));
		add_torque(0, { 5000, 5000, 5000 });
		break;
	case 4:
		*timestep = 0.001f;
		gravity = Vec3(0, -9.81f, 0);
		add_sphere({ 0,0,0 }, 0.5, 10);
		break;
	case 5:
		*timestep = 0.001f;
		gravity = Vec3(0, -9.81f, 0);

		add_box({ 0.5, 0, 0 }, { 1, 1, 1 }, 10);
		break;
	case 6:
		// SPH Dam
	{
		add_box({ 3, -0.5, -1 }, { 1, 1, 1 }, 10);
		*timestep = 0.001f;
		bounciness = 0.3;
		gravity = Vec3(0, -9.81f, 0);
		int num_particles = DIM * DIM * DIM;
		Real rho_0 = 1;
		Real particle_radius = 0.3;
		sph->init_sim(gravity, rho_0, num_particles, particle_radius);
		const Vec3 offset = Vec3{ 0, 0, -0.25 };
		const Real two_r = particle_radius * 2.0f;
		auto n = Vec3(0, 0, -DIM * two_r);
		int h_count = 6;
		int d_count = (4 / sph->boundary_radius);
		auto l = Vec3(-h_count * sph->boundary_radius, 0, 0);
		sph->init_particles(
			Vec3(-DIM / 2 * two_r, 0, 0),
			Vec3(0, -0.25 + DIM * two_r, 0),
			Vec3(0, 0, -DIM * two_r) + 1.5 * offset, DIM, DIM, DIM
		);
		const Real boundary_radius = sph->boundary_radius;
		auto& boundary_particles = sph->boundary_particles;
		int v_count = (3 + 0.75) / boundary_radius;
		auto t = Vec3(0, -1 + v_count * boundary_radius, 0);
		auto r = Vec3(h_count * boundary_radius, 0, 0);
		for (int k = 0; k < 1; k++) {
			for (int i = 0; i < 2 * h_count; i++) {
				for (int j = 0; j <= 2 * d_count; j++) {
					Vec3 pos = Vec3(i * boundary_radius, -0.75 - k * boundary_radius, j * boundary_radius);
					sph->boundary_particles.push_back(Particle{ sph->dm, 0, pos + l + n, rho_0,
						bnd_set_idx });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(Particle{ sph->dm, 0, pos + l + n, rho_0,
						bnd_set_idx });
				}
			}
			for (int i = 0; i < v_count; i++) {
				for (int j = 0; j <= 2 * d_count; j++) {
					Vec3 pos = Vec3(-k * boundary_radius, -i * boundary_radius, j * boundary_radius);
					boundary_particles.push_back(Particle{ sph->dm, 0, pos + l + t + n , rho_0,
						bnd_set_idx });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(Particle{ sph->dm, 0, pos + l + t + n , rho_0,
						bnd_set_idx });
					pos.x = k * boundary_radius;
					pos.z = j * boundary_radius;
					boundary_particles.push_back(Particle{ sph->dm, 0, pos + r + t + n , rho_0,
						bnd_set_idx });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(Particle{ sph->dm, 0, pos + r + t + n , rho_0,
						bnd_set_idx });
				}
			}
			if (!sph->is_2d) {
				auto nf = {
						n + 2 * Vec3(0, 0, d_count * boundary_radius) ,
						n - 2 * Vec3(0, 0, d_count * boundary_radius)
				};
				for (const auto& p : nf) {
					for (int i = 0; i < 2 * h_count; i++) {
						for (int j = 0; j < v_count; j++) {
							Vec3 pos = Vec3(i * boundary_radius, -j * boundary_radius, 0);
							boundary_particles.push_back(Particle{ sph->dm, 0, pos + l + t + p , rho_0,
								bnd_set_idx });
							pos.x = k * boundary_radius;
							boundary_particles.push_back(Particle{ sph->dm, 0, pos + r + t + p , rho_0,
								bnd_set_idx });
						}
					}

				}
			}

		}
		sph->neighborhood_searcher->register_set(boundary_particles);
		sph->neighborhood_searcher->find_neighborhoods();
		sph->compute_boundary_volumes();
	}

	break;
	case 7:
		// SPH Box
	{
		add_box({ 0, -0.5, -1 }, { 1, 1, 1 }, 100);
		*timestep = 0.001f;
		bounciness = 0.3;
		gravity = Vec3(0, -9.81f, 0);
		int num_particles = DIM * DIM * DIM;
		Real rho_0 = 10;
		Real particle_radius = 0.3;
		sph->init_sim(gravity, rho_0, num_particles, particle_radius);
		const Vec3 offset = Vec3{ 0, 0, -0.25 };
		const Real two_r = particle_radius * 2.0f;
		auto n = Vec3(0, 0, -DIM * two_r);
		int h_count = 20;
		int d_count = (4 / sph->boundary_radius);
		auto l = Vec3(-h_count * sph->boundary_radius, 0, 0);
		sph->init_particles(Vec3(-DIM / 2 * two_r + 1, 0, 0),
			Vec3(0, -0.5 + DIM * two_r, 0),
			Vec3(0, 0, -DIM * two_r - 2) + 1.5 * offset, DIM, DIM, DIM);
		const Real boundary_radius = sph->boundary_radius;
		auto& boundary_particles = sph->boundary_particles;
		int v_count = (3 + 0.75) / boundary_radius;
		auto t = Vec3(0, -1 + v_count * boundary_radius, 0);
		auto r = Vec3(h_count * boundary_radius, 0, 0);
		for (int k = 0; k < 3; k++) {

			for (int i = 0; i < 2 * h_count; i++) {
				for (int j = 0; j < 2 * d_count; j++) {
					Vec3 pos = Vec3(i * boundary_radius,
						-0.75 - k * boundary_radius,
						j * boundary_radius);
					boundary_particles.push_back(
						Particle{ sph->dm, 0, pos + l + n, rho_0 ,
						bnd_set_idx, /* visible */ k > 0 });
					pos.z = -j * boundary_radius;
					boundary_particles.push_back(
						Particle{ sph->dm, 0, pos + l + n, rho_0,
						bnd_set_idx, /* visible */ k > 0 });
				}
			}
		}
		DUC->box->samples.clear();
		DUC->box->sample_mesh();
		create_rb_boundaries(true /* create boundary particles */);
		sph->neighborhood_searcher->register_set(boundary_particles);
		sph->neighborhood_searcher->register_set(sph->moving_boundary_particles);
		sph->neighborhood_searcher->find_neighborhoods();
		sph->compute_boundary_volumes();
	}

	default:
		break;
	}
	add_plane(-1, { 0,1,0 });
}

void Sandbox::externalForcesCalculations(float timeElapsed) {
	Point2D mouse_diff;
	mouse_diff.x = trackmouse.x - old_trackmouse.x;
	mouse_diff.y = trackmouse.y - old_trackmouse.y;
	if (mouse_diff.x != 0 || mouse_diff.y != 0) {
		Mat4 world_view_inv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		world_view_inv = world_view_inv.inverse();
		Vec3 input_view = Vec3((float)mouse_diff.x, (float)-mouse_diff.y, 0);
		Vec3 input_world = world_view_inv.transformVectorNormal(input_view);
		// find a proper scale!
		float inputScale = 0.0001f * (0.01 / *timestep);
		input_world = input_world * inputScale;
		mouse_force = input_world;
	} else {
		mouse_force = {};
	}
	for (auto& rb : rigid_bodies) {
		rb.force += mouse_force + gravity * rb.mass;
	}
}

void Sandbox::handle_collisions() {
	if (rigid_bodies.size() < 2) {
		return;
	}
	bool resolve = false;
	Contact* ci = nullptr;
	CollisionData data;
	for (int i = 0; i < rigid_bodies.size() - 1; i++) {

		RigidBody& b1 = rigid_bodies[i];

		for (int j = i + 1; j < rigid_bodies.size(); j++) {

			RigidBody& b2 = rigid_bodies[j];
			// TODO: Vector is overkill here, fix
			std::vector<RigidBody*> pairs = { &b1,&b2 };
			std::sort(pairs.begin(), pairs.end(), [](RigidBody* a, RigidBody* b) {
				return a->type < b->type;
			});
			if (pairs[0]->type == RigidBodyType::CUBOID && pairs[1]->type == RigidBodyType::CUBOID) {

				ci = checkCollisionSAT(pairs[0]->obj_to_world(), pairs[1]->obj_to_world(), &data);
				resolve = ci && ci->is_valid;
				if (resolve) {
					ci->bodies[0] = &b1;
					ci->bodies[1] = &b2;
					ci->cp_rel[0] = ci->collision_point - b1.position;
					ci->cp_rel[1] = ci->collision_point - b2.position;
				}
			} else {
				int k = static_cast<int>(pairs[0]->type) | static_cast<int>(pairs[1]->type);
				auto collision_func = collision_map[k];
				Mat4 b1_world = pairs[0]->obj_to_world();
				ci = collision_func(pairs[0], pairs[1], b1_world, data);
				resolve = ci && ci->is_valid;
			}

			if (resolve) {
				// Apply position change
				resolve_positions(data);
				// Apply velocity change
				resolve_velocities(data, ci, pairs);
			}
			data.reset();
		}
	}
}

void Sandbox::simulateTimestep(float time_step) {
	if (m_iTestCase >= 10) {
		sf->simulateTimestep(time_step);
		return;
	}
	num_run++;
	if (m_iTestCase == 3) {
		static std::mt19937 eng;
		static std::uniform_real_distribution<float> randomer(-1.5, 2);
		static int counter = 0;
		counter++;
		if (counter > 500) {
			add_box({ randomer(eng), 1.0, randomer(eng) }, { 0.3, 0.3, 0.3 }, 2);
			counter = 0;
		}
	}
	if (sph) {
		sph->simulateTimestep(time_step);
	}
	for (auto& rb : rigid_bodies) {
		if (!rb.movable)
			continue;
		Vec3 delta_x = time_step * rb.linear_velocity;
		rb.position += delta_x;
	
		rb.linear_velocity += time_step * rb.force * rb.inv_mass;
		auto ang_vel = Quat(rb.angular_vel.x, rb.angular_vel.y, rb.angular_vel.z, 0);
		rb.orientation += time_step * 0.5f * rb.orientation * ang_vel;
		rb.orientation = rb.orientation.unit();
		rb.angular_momentum += time_step * rb.torque;
		auto inv_inertia = rb.get_transformed_inertia(rb.inv_inertia_0);
		rb.angular_vel = inv_inertia * rb.angular_momentum;
		if (sph) {
			auto rot_mat = rb.orientation.getRotMat();
			for (auto& p : sph->moving_boundary_particles) {
				p.pos += delta_x;
				p.pos = rb.position + rot_mat * (p.pos - rb.position);
			}
		}
	}
	// Clear forces & torques
	for (auto& rb : rigid_bodies) {
		rb.force = 0;
		rb.torque = 0;
	}

	handle_collisions();
}

void Sandbox::onClick(int x, int y) {
	trackmouse.x = x;
	trackmouse.y = y;
}

void Sandbox::onMouse(int x, int y) {
	old_trackmouse.x = x;
	old_trackmouse.y = y;
	trackmouse.x = x;
	trackmouse.y = y;
}

int Sandbox::getNumberOfRigidBodies() {
	return rigid_bodies.size();
}

Vec3 Sandbox::getPositionOfRigidBody(int i) {
	return rigid_bodies[i].position;
}

Vec3 Sandbox::getLinearVelocityOfRigidBody(int i) {
	return rigid_bodies[i].linear_velocity;
}

Vec3 Sandbox::getAngularVelocityOfRigidBody(int i) {
	return rigid_bodies[i].angular_vel;
}

void Sandbox::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	rigid_bodies[i].apply_force(loc, force);
}

void Sandbox::add_box(Vec3 position, Vec3 size, int mass) {
	RigidBody box;
	box.make_box(position, size, mass);
	rigid_bodies.push_back(box);
}

void Sandbox::add_sphere(const Vec3& pos, float radius, int mass) {
	RigidBody sphere;
	sphere.make_sphere(radius, pos, mass);
	rigid_bodies.push_back(sphere);
}

void Sandbox::add_plane(float offset, const Vec3& normal) {
	RigidBody plane;
	plane.make_plane(offset, normal);
	rigid_bodies.push_back(plane);

}

void Sandbox::setOrientationOf(int i, Quat orientation) {
	rigid_bodies[i].orientation = orientation;
}

void Sandbox::setVelocityOf(int i, Vec3 velocity) {
	rigid_bodies[i].linear_velocity = velocity;
}

void Sandbox::add_torque(int i, Vec3 ang_accelaration) {

	auto inertia = rigid_bodies[i].get_transformed_inertia(
		rigid_bodies[i].inv_inertia_0.inverse()
	);
	rigid_bodies[i].torque += inertia * ang_accelaration;
}

void Sandbox::resolve_positions(CollisionData& data) {
	// Apply positional change
	auto iter_cnt = data.num_contacts == 1 ? 1 : 4 * data.num_contacts;
	Contact* collision_info;
	Vec3 angular_delta[2] = { Vec3(), Vec3() };
	Vec3 linear_delta[2] = { Vec3(), Vec3() };

	constexpr float angular_limit = 0.2f;
	for (int iter = 0; iter < iter_cnt; iter++) {
		auto max = 0.001f;
		int index = data.num_contacts;

		for (int i = 0; i < data.num_contacts; i++) {
			if (data.contacts[i].penetration > max) {
				max = data.contacts[i].penetration;
				index = i;
				collision_info = &data.contacts[i];
			}
		}
		if (index == data.num_contacts) {
			break;
		}
		float angular_mov[2];
		float linear_mov[2];
		float total_inertia = 0;
		float linear_inertia[2];
		float angular_inertia[2];

		for (int i = 0; i < 2; i++) {
			if (collision_info->bodies[i]) {
				Mat4 inv_inertia = collision_info->bodies[i]->get_transformed_inertia(
					collision_info->bodies[i]->inv_inertia_0
				);
				//Mat4 inv_inertia = collision_info->bodies[i]->inv_inertia_0;
				// -Linear inertia is proportional to inv_mass
				// -Angular inertia is proportional to linear projection of velocity along 
				// the normal induced by delta-angular velocity(impulsive torque)
				Vec3 ang_comp = cross(collision_info->cp_rel[i], collision_info->normal);
				ang_comp = inv_inertia * ang_comp;
				// Get induced linear velocity then project
				ang_comp = cross(ang_comp, collision_info->cp_rel[i]);
				angular_inertia[i] = dot(ang_comp, collision_info->normal);

				linear_inertia[i] = collision_info->bodies[i]->inv_mass;
				total_inertia += angular_inertia[i] + linear_inertia[i];
			}
		}

		for (int i = 0; i < 2; i++) {
			if (collision_info->bodies[i]) {
				auto sign = i == 0 ? 1 : -1;
				angular_mov[i] = sign * collision_info->penetration * (angular_inertia[i] / total_inertia);
				linear_mov[i] = sign * collision_info->penetration * (linear_inertia[i] / total_inertia);
				// Bigger bodies should be able to rotate more
				Vec3 proj = collision_info->cp_rel[i] - collision_info->normal * dot(collision_info->normal, collision_info->cp_rel[i]);
				auto max_angle = angular_limit * norm(proj);
				// Distribute the clamp
				if (angular_mov[i] < -max_angle) {
					auto tot = angular_mov[i] + linear_mov[i];
					angular_mov[i] = -max_angle;
					linear_mov[i] = tot - angular_mov[i];
				} else if (angular_mov[i] > max_angle) {
					auto tot = angular_mov[i] + linear_mov[i];
					angular_mov[i] = max_angle;
					linear_mov[i] = tot - angular_mov[i];
				}
				if (angular_mov[i]) {
					Mat4 inv_inertia = collision_info->bodies[i]->get_transformed_inertia(
						collision_info->bodies[i]->inv_inertia_0
					);
					Vec3 tmp = cross(collision_info->cp_rel[i], collision_info->normal);
					angular_delta[i] = inv_inertia * tmp * (angular_mov[i] / angular_inertia[i]);
				} else {
					angular_delta[i] = 0;
			}
				linear_delta[i] = collision_info->normal * linear_mov[i];
				// Apply deltas
				collision_info->bodies[i]->position += collision_info->normal * linear_mov[i];
				// Note : Rotating contact points doesn't work properly with high bounciness
#define ORIENT_BODY 0
#if ORIENT_BODY == 1
				Quat q(angular_delta[i].x, angular_delta[i].y, angular_delta[i].z, 0);
				collision_info->bodies[i]->orientation += 0.5 * collision_info->bodies[i]->orientation * q;
#elif ORIENT_BODY == 2
				collision_info->bodies[i]->orientation.x += angular_delta[i].x * 0.5;
				collision_info->bodies[i]->orientation.y += angular_delta[i].y * 0.5;
				collision_info->bodies[i]->orientation.z += angular_delta[i].z * 0.5;
#endif
#undef ORIENT_BODY
				collision_info->bodies[i]->orientation.unit();
		}
	}
	// Propagate
		for (int i = 0; i < data.num_contacts; i++) {
			for (int j = 0; j < 2; j++) {
				if (data.contacts[i].bodies[j]) {
					for (int k = 0; k < 2; k++) {
						if (data.contacts[i].bodies[j] == data.contacts[index].bodies[k]) {
							auto sgn = j ? 1 : -1;
							Vec3 delta_pos = linear_delta[k] + cross(angular_delta[k], data.contacts[i].cp_rel[j]);
							data.contacts[i].penetration += dot(delta_pos, data.contacts[i].normal) * sgn;
						}
					}
				}

			}
		}
}
}

void Sandbox::resolve_velocities(CollisionData& data, Contact* best_col,
	const std::vector<RigidBody*>& pairs) {
	auto iter_cnt = data.num_contacts == 1 ? 1 : 4 * data.num_contacts;
	Vec3 angular_mom_delta[2] = { Vec3(), Vec3() };
	Vec3 linear_vel_delta[2] = { Vec3(), Vec3() };
	Contact* collision_info;
	// Set initial expected velocity after collision (Relative, Va - Vb)
	for (int i = 0; i < data.num_contacts; i++) {
		auto b1_collision_pos = data.contacts[i].collision_point - pairs[0]->position;
		auto b2_collision_pos = data.contacts[i].collision_point - pairs[1]->position;
		auto b1_collision_vel = pairs[0]->linear_velocity + cross(pairs[0]->angular_vel, b1_collision_pos);
		auto b2_collision_vel = pairs[1]->linear_velocity + cross(pairs[1]->angular_vel, b2_collision_pos);
		auto normal_dot_relative_vel = dot(data.contacts[i].normal, b1_collision_vel - b2_collision_vel);
		data.contacts[i].relative_vel = normal_dot_relative_vel;
		data.contacts[i].expected_vel = -(1 + bounciness) * normal_dot_relative_vel;
	}
	for (int iter = 0; iter < iter_cnt; iter++) {
		int index = data.num_contacts;
		auto max = 0.001;
		for (int i = 0; i < data.num_contacts; i++) {
			if (data.contacts[i].expected_vel > max) {
				max = data.contacts[i].expected_vel;
				index = i;
				collision_info = &data.contacts[i];
			}
		}
		if (index == data.num_contacts) {
			break;
		}

		//collision_info = best_col;
		auto b1_inv_inertia = pairs[0]->get_transformed_inertia(pairs[0]->inv_inertia_0);
		auto b2_inv_inertia = pairs[1]->get_transformed_inertia(pairs[1]->inv_inertia_0);
		auto b1_collision_pos = collision_info->collision_point - pairs[0]->position;
		auto b2_collision_pos = collision_info->collision_point - pairs[1]->position;
		auto b1_collision_vel = pairs[0]->linear_velocity + cross(pairs[0]->angular_vel, b1_collision_pos);
		auto b2_collision_vel = pairs[1]->linear_velocity + cross(pairs[1]->angular_vel, b2_collision_pos);
		auto normal_dot_relative_vel = dot(collision_info->normal, b1_collision_vel - b2_collision_vel);

		if (collision_info->relative_vel < 0) {
			auto numerator = collision_info->expected_vel;
			auto denominator = pairs[0]->inv_mass + pairs[1]->inv_mass
				+ dot(
					cross(b1_inv_inertia * cross(b1_collision_pos, collision_info->normal), b1_collision_pos) +
					cross(b2_inv_inertia * cross(b2_collision_pos, collision_info->normal), b2_collision_pos),
					collision_info->normal);

			auto impulse = numerator / denominator;
			linear_vel_delta[0] = impulse * collision_info->normal * pairs[0]->inv_mass;
			linear_vel_delta[1] = impulse * collision_info->normal * pairs[1]->inv_mass;
			angular_mom_delta[0] = cross(b1_collision_pos, impulse * collision_info->normal);
			angular_mom_delta[1] = cross(b2_collision_pos, impulse * collision_info->normal);
			pairs[0]->linear_velocity += linear_vel_delta[0];
			pairs[1]->linear_velocity -= linear_vel_delta[1];

			pairs[0]->angular_momentum += angular_mom_delta[0];
			pairs[1]->angular_momentum -= angular_mom_delta[1];
			Vec3 delta_vel[2];
			delta_vel[0] = linear_vel_delta[0] + cross(b1_inv_inertia * angular_mom_delta[0], b1_collision_pos);
			delta_vel[1] = linear_vel_delta[1] + cross(b2_inv_inertia * angular_mom_delta[1], b2_collision_pos);
			auto del = dot(delta_vel[0] - delta_vel[1], collision_info->normal);
			calc_after_col_vel(collision_info, del + collision_info->relative_vel, pairs);
		}
	}
}

void Sandbox::calc_after_col_vel(Contact* contact, float delta_vel,
	const std::vector<RigidBody*>& pairs) {
	contact->relative_vel = delta_vel;
	contact->expected_vel = -(1 + bounciness) * delta_vel;
}

void Sandbox::create_rb_boundaries(bool create) {
	for (auto& rb : rigid_bodies) {
		if (create) {
			if (rb.type == RigidBodyType::CUBOID) {
				rb.samples = DUC->box->samples;
			}
		}
			for (int i = 0; i < rb.samples.size(); i++) {
				rb.samples[i] += rb.position;
				rb.samples[i] = rb.orientation.getRotMat() * rb.samples[i];
				rb.samples[i].x *= rb.size.x;
				rb.samples[i].y *= rb.size.y;
				rb.samples[i].z *= rb.size.z;
				if (create) {
					Particle particle(sph->dm, 0, rb.samples[i], sph->rho_0, 2);
					particle.owner = &rigid_bodies.back();
					sph->moving_boundary_particles.push_back(particle);
				}
			}
		}
}

void Sandbox::pass_time_step_variable(float& time_step) {
	timestep = &time_step;
}


