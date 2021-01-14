#include "DiffusionSimulator.h"
using namespace std;


Grid::Grid(int dim_x, bool is_3d) : dim_x(dim_x), is_3d(is_3d) {
	num_points = is_3d ? pow(dim_x, 3) : pow(dim_x, 2);
	values.resize(num_points);
	positions.resize(num_points);
	pos_internal.resize(num_points);
	// Some documentation of grid for reference
	// Flattened(default) version (indices). For EX in 2D:
	/*
		0, grid_dim, 1, 1+grid_dim, ... grid_dim-1, 2*grid_dim -1,
		2*grid_dim 2*grid_dim + grid_dim...
	*/
	// For 3D:
	/*
		0, grid_dim, 1, 1 + grid_dim, ... grid_dim - 1, 2 * grid_dim - 1
		grid_dim*grid_dim, grid_dim*grid_dim +1, grid_dim*grid_dim + grid_dim...
	*/

	// Example 4x4 block indices
	/*
		0 2 4 6
		1 3 5 7
		8 10 12 14
		9 11 13 15
	*/

	// Example 5x5 block indices
	/*
		0 2 4 6 8
		1 3 5 7 9
		10 12 14 16 18
		11 13 15 17 19
		20 21 22 23 24
	*/

	// Order is j->k-i

	// Unflattened version: Regular grid i.e
	/*
		0,1,2,3,4.......grid_sz -1
	*/

	// Set up initial values and boundary values
	// We are using Drichlet conditions so we have
	/*
		0 0 0 0 ... 0
		0 x x x ... 0
		0 x x x ... 0
		0 x x x ... 0
		..
		0 0 0 0 ... 0
	*/
	for (int i = 0; i < num_points; i++) {
		if (i == 3) {
			values[i] = 1000000;
		}
	}
	// R = 0.1
	// For easier specification of positions, we naively assign position values
	Vec3 L = Vec3(-0.1 * dim_x, 0, 0);
	Vec3 T = Vec3(0, 0.1 * dim_x, 0);
	int k_iter = is_3d ? dim_x : 1;
	int dim_sqr = dim_x * dim_x;
	for (int k = 0; k < k_iter; k++) {
		int k_offset = dim_sqr * k;
		for (int i = 0; i < dim_x - 1; i += 2) {
			auto offsety = Vec3(0, i * 0.1, 0);
			for (int j = 0; j < dim_x; j++) {
				auto offsetx = Vec3(j * 0.1, 0, 0);
				positions[k_offset + dim_x * i + j] = L + offsetx - Vec3(0, 0, k * 0.1);
				positions[k_offset + dim_x * i + j] += T - offsety;
				positions[k_offset + dim_x * (i + 1) + j] = 
					positions[k_offset + dim_x * i + j] - Vec3(0, 0.1, 0);
			}
		}
		if (dim_x % 2) {
			Vec3 B = Vec3(0, 0.1, 0);
			for (int i = 0; i < dim_x; i++) {
				positions[k_offset + (dim_x - 1) * dim_x + i] =
					B + L + Vec3(i * 0.1, 0, -k * 0.1);
			}
		}
	}
	auto idxs = get_internal_repr_idxs();
	// We map previous position values to an internal representation.
	// NOTE: This assumes fixed positions. For a dynamic grid we'd need to calculate 
	// internal positions on the fly or set them according to the mapping in the first place.
	for (int i = 0; i < num_points; i++) {
		pos_internal[i] = positions[idxs[i]];
	}
}

std::vector<int> Grid::get_internal_repr_idxs() {
	
	std::vector<int> result;
	result.resize(num_points);
	auto idx = 0;
	int dim_sqr = dim_x * dim_x;
	int k_iter = is_3d ? dim_x : 1;
	for (int k = 0; k < k_iter; k++) {
		idx = 0;
		int k_offset = dim_sqr * k;
		for (int i = 0; i < dim_x / 2; i++) {
			for (int j = 0; j < dim_x; j++) {
				result[k_offset + idx] = k_offset + i * 2 * dim_x + j;
				result[k_offset + idx + 1] = k_offset + i * 2 * dim_x + j + dim_x;
				idx += 2;
			}
		}
		if (dim_x % 2) {
			for (int i = 0; i < dim_x; i++) {
				result[dim_sqr * k + idx + i] = dim_sqr * k + idx + i;
			}
		}
	}
	return result;
}

DiffusionSimulator::DiffusionSimulator(bool adaptive_step) {
	m_iTestCase = 0;
	movable_obj_pos = Vec3();
	movable_obj_final_pos = Vec3();
	rotate = Vec3();
	this->adaptive_step = adaptive_step;
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit solver, Implicit solver, Implicit 3D";
}

void DiffusionSimulator::reset() {
	mouse.x = mouse.y = 0;
	track_mouse.x = track_mouse.y = 0;
	old_track_mouse.x = old_track_mouse.y = 0;

}

void TW_CALL DiffusionSimulator::setDimSize(const void* value, void* clientData) {
	client_data* client_p = static_cast<client_data*>(clientData);
	*(client_p->dim_size) = *(static_cast<const int*>(value));
	client_p->self->init_grid();
	if (client_p->self->m_iTestCase == 1) {
		client_p->self->setup_for_implicit();
	}
}

void TW_CALL DiffusionSimulator::getDimSize(void* value, void* clientData) {
	int* val_p = static_cast<int*>(value);
	*val_p = *((static_cast<client_data*>(clientData))->dim_size);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;

	data = new client_data();
	data->dim_size = &dim_size;
	data->self = this;

	TwAddVarCB(DUC->g_pTweakBar, "Grid Size", TW_TYPE_INT32, setDimSize, getDimSize, reinterpret_cast<void*>(data), "step=1 min=1");
}

static bool is_any_boundary(int i, int j, int dim_size, bool is_3d,
							int dim_sqr, int dim_2, int rem, bool is_odd) {
	int res = i / dim_sqr;
	bool top = (i % dim_sqr) < dim_2 && (i % 2 == (is_odd ? res % 2 : 0));
	bool bottom = (!is_odd && ((i % dim_sqr) > dim_sqr - dim_2 && i % 2 == 1)
		|| (is_odd && ((i % dim_sqr) >= dim_sqr - dim_size)));
	int odd_offset = is_odd ? rem * (res % 2) : 0;
	bool right = i % dim_2 == (dim_2 - 2 + odd_offset) % dim_2
		|| i % dim_2 == (dim_2 - 1 + odd_offset) % dim_2
		|| (is_odd && (i % dim_sqr == (dim_sqr - 1)));
	bool left = (!is_odd && ((i % dim_2 == 0) || i % dim_2 == 1))
		|| (is_odd && ((i % dim_sqr < (dim_sqr - dim_size) &&
			(i% dim_2 == odd_offset || i % dim_2 == (1 + odd_offset) % dim_2))))
		|| (is_odd && (i % dim_sqr == dim_sqr - dim_size));
	bool f = res == dim_size - 1;
	bool tr = top && right;
	bool br = bottom && right;
	bool tl = top && left;
	bool bl = bottom && left;
	bool fl = f && left;
	bool fr = f && right;
	bool bf = f && bottom;
	bool tf = f && top;
	if ((i % dim_2 == dim_2 - 2 || i % dim_2 == dim_2 - 1)
		|| bottom || top 
		|| (i % dim_2 == 0 || i % dim_2 == 1)
		|| (is_3d && res >= dim_size - 1)) {
		if ((is_3d && (res <= dim_size - 1 && res != 0)) &&
			(((((((!tr && !br) && !tl) && !bl) && !fl) && !fr) && !bf) && !tf)) {
			return false;
		}
		return true;
	}
	return false;
}

static void setup_A(SparseMatrix<Real>& A, double alpha, Real grid_size, int dim_size, float dt,
	bool is_3d = false) {
	// Order : j->i->(possibly k)
	const Real dx = 0.1; //this used to be grid_size / dim_size but when dim_size increased, grid size increases as well so it always keeps a ratio of 0.1
	const Real dx2 = dx * dx;
	bool is_dim_odd = dim_size % 2;
	const Real F = alpha * dt / (2 * dx2);
	const int N = is_3d ? dim_size * dim_size * dim_size :
		dim_size * dim_size;
	const int dim_2 = dim_size * 2;
	const int dim_sqr = dim_size * dim_size;
	const int rem = dim_sqr % dim_2;
	// Note: In 3D case, instead of individually formulating each boundary 
	// condition, we can just replicate each N^2 mask across the stack(i.e cube) as a shorter and faster solution. 
	// Since this function is run once, it doesnt matter for now.
	for (int i = 0; i < N; i++) {
		bool right = false;
		bool bottom = false;
		bool top = false;
		bool f = false;
		bool left = false;
		bool any_boundary = false;
		int res = i / dim_sqr;
		int odd_offset = is_dim_odd ? rem * (res % 2) : 0;
		if ((i  % dim_2 == (dim_2 - 2  + odd_offset) % dim_2
			|| i % dim_2 == (dim_2 - 1 + odd_offset) % dim_2)
			|| (is_dim_odd && (i % dim_sqr == (dim_sqr - 1)))) {
			right = true;
			any_boundary = true;
		}

		if ((!is_dim_odd && ((i % dim_sqr) > dim_sqr - dim_2 && i % 2 == 1))
			|| (is_dim_odd && (i % dim_sqr) >= dim_sqr - dim_size)) {
			bottom = true;
			any_boundary = true;
		}
		if ((is_3d && res >= dim_size - 1)) {
			if (res == dim_size - 1) {
				f = true;
			}
			any_boundary = true;
		}
		if ((i % dim_sqr) < dim_2 && i % 2 == (is_dim_odd ? res % 2 : 0)) {
			any_boundary = true;
			top = true;
			A.set_element(i, i, 1);
		}
		// TODO: There is probably a huge simplification here
		if ((!is_dim_odd && ((i % dim_2 == 0) || i % dim_2 == 1))
			|| (is_dim_odd && ((i % dim_sqr < (dim_sqr - dim_size) && 
				 (i % dim_2 == odd_offset ||i % dim_2 == (1 + odd_offset) % dim_2))))
			|| (is_dim_odd && (i % dim_sqr == dim_sqr - dim_size))) {
			left = true;
			any_boundary = true;
			A.set_element(i, i, 1);
		}
		bool tr = top && right;
		bool br = bottom && right;
		bool tl = top && left;
		bool bl = bottom && left;
		bool fr = f && right;
		bool fl = f && left;
		bool bf = bottom && f;
		bool tf = top && f;
		if ((is_3d && (res != 0 && res <= dim_size - 1)) &&
			(((((((!tr && !br) && !tl) && !bl) && !fl) && !fr) && !bf) && !tf)) {
			any_boundary = false;
		}
		if (any_boundary) {
			A.set_element(i, i, 1);
		}
		else {
			A.set_element(i, i, is_3d ? 1 + 6 * F : 1 + 4 * F);
		}
		if (is_3d && !f) {
			int offset_z = dim_size * dim_size;
			if (!any_boundary) {
				A.set_element(i, i + offset_z, -F);
			}
			if (!is_any_boundary(i + offset_z, i, dim_size, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
				A.set_element(i + offset_z, i, -F);
			}
		}
		int row = ((i % dim_sqr) / (2 * dim_size)) * 2 + 1;
		if (!bottom) {
			if (i % 2 == ((odd_offset % 2) ^ 1)) {
				int offset = (is_dim_odd && row == dim_size - 2) ? dim_2 - 1 - (((i % dim_sqr) % (2 * dim_size)) / 2) :
					dim_2 - 1;
				if (!any_boundary) {
					A.set_element(i, i + offset, -F);
				}
				if (!is_any_boundary(i + offset, i, dim_size, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
					A.set_element(i + offset, i, -F);
				}
			}
			else {
				if (!any_boundary) {
					A.set_element(i, i + 1, -F);
				}
				if (!is_any_boundary(i + 1, i, dim_size, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
					A.set_element(i + 1, i, -F);
				}
			}
		}
		if (!right) {
			bool last_row = row == dim_size - 1;
			int row_offset = is_dim_odd && last_row  ? 1 : 2;
			if (!any_boundary) {
				A.set_element(i, i + row_offset, -F);
			}
			if (!is_any_boundary(i + row_offset, i, dim_size, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
				A.set_element(i + row_offset, i, -F);
			}
		}
	}
}
void DiffusionSimulator::notifyCaseChanged(int test_case) {
	m_iTestCase = test_case;
	movable_obj_pos = Vec3(0, 0, 0);
	rotate = Vec3(0, 0, 0);
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		is_3d = false;
		init_grid();
		break;

	case 1:
	{
		cout << "Implicit solver!\n";
		is_3d = false;
	init:
		init_grid();
		if (!adaptive_step) {
			setup_for_implicit();
		}
	}
	break;
	case 2:
	{
		cout << "3D Implicit Solver! ";
		is_3d = true;
		goto init;
	}
	break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::solve_explicit(float time_step) {
	auto new_grid = std::make_unique<Grid>(dim_size, is_3d);
	const int N = is_3d ? dim_size * dim_size * dim_size : dim_size * dim_size;
	const int size2d = dim_size * dim_size;
	
	const Real dx = 0.1f;
	const Real dx2 = dx * dx;
	bool is_dim_odd = dim_size % 2;
	float highest_value = 0;

	/*
		0 2 4 6 8
		1 3 5 7 9
		10 12 14 16 18
		11 13 15 17 19
		20 21 22 23 24
	*/
	/*
		0   2  4  6  8 10 12
		1   3  5  7  9 11 13
		14 16 18 20 22 24 26
		15 17 19 21 23 25 27
		28 30 32 34 36 38 40
		29 31 33 35 37 39 41
		42 43 44 45 46 47 48

		0 2 4
		1 3 5
		6 7 8

		0 2 4 6 8 10 12 14 16 18 20 22 24 26 28 30
		1 3 5 7 9 11 13 15 17 19 21 23 25 27 29 31
	*/

	std::cout << "starting\n";
	for (int i = 0; i < N; i++) {
		float it = grid->values[i];
		auto neutralized_i = i % size2d;

		float x_diff = 0;
		float y_diff = 0;
		float z_diff = 0;

		int y = (neutralized_i / (2 * dim_size)) * 2;
		if ((is_dim_odd && y != (dim_size - 1)) || !is_dim_odd) {
			y += neutralized_i % 2 == 0 ? 0 : 1;
		}

		int x = 0;
		x = (neutralized_i % (2 * dim_size) - (neutralized_i % 2 == 0 ? 0 : 1)) / 2;
		if (is_dim_odd && y == (dim_size - 1)) {
			x = neutralized_i - (dim_size * (dim_size - 1));
		}		

		int z = i / size2d;

		if (x == 0 || x == dim_size - 1 || y == 0 || y == dim_size - 1 || (is_3d && z == 0) || (is_3d && z == dim_size - 1)) {
			new_grid->values[i] = 0;
			continue;
		}

		bool last_row = y == (dim_size - 1);

		//x
		int x_offset = (last_row && is_dim_odd) ? 1 : 2;
		x_diff += (neutralized_i - x_offset) < 0 ? 0 : grid->values[i - x_offset];
		x_diff += (neutralized_i + x_offset) >= size2d ? 0 : grid->values[i + x_offset];
		x_diff -= 2 * it;

		//y
		if (last_row && is_dim_odd) {
			int y_offset = (dim_size * 2 - 1) - (neutralized_i - dim_size * (dim_size - 1));
			y_diff += (neutralized_i - y_offset) < 0 ? 0 : grid->values[i - y_offset];
		}
		if (y == dim_size - 2 && is_dim_odd) {
			int y_offset = (dim_size * 2 - 1) - (neutralized_i - (dim_size * (dim_size - 3) + 1)) / 2;
			y_diff += grid->values[i - 1];
			y_diff += (neutralized_i + y_offset) >= size2d ? 0 : grid->values[i + y_offset];
		}
		else if (neutralized_i % 2 == 0) {
			y_diff += (neutralized_i - dim_size * 2 + 1) < 0 ? 0 : grid->values[i - dim_size * 2 + 1];
			y_diff += grid->values[i + 1];
		}
		else {
			y_diff += grid->values[i - 1];
			y_diff += (neutralized_i + dim_size * 2 - 1) >= size2d ? 0 : grid->values[i + dim_size * 2 - 1];
		}
		y_diff -= 2 * it;

		//z
		if (is_3d) {
			z_diff += (i - dim_size * dim_size) < 0 ? 0 : grid->values[i - dim_size * dim_size];
			z_diff += (i + dim_size * dim_size) >= N ? 0 : grid->values[i + dim_size * dim_size];
			z_diff -= 2 * it;
		}

		new_grid->values[i] = (x_diff/dx2 + y_diff/dx2 + z_diff/dx2) * alpha * time_step + it;
		if (new_grid->values[i] > highest_value) {
			highest_value = new_grid->values[i];
		}
	}

	std::cout << highest_value << "\n";

	grid->values = std::move(new_grid->values);
	return nullptr;
}

void DiffusionSimulator::solve_implicit(float time_step) {
	// solve A T = b
	std::unique_ptr<SparseMatrix<Real>> A_local = nullptr;
	constexpr Real pcg_target_residual = 1e-05;
	constexpr Real pcg_max_iterations = 1000;
	const int N = is_3d ? dim_size * dim_size * dim_size : dim_size * dim_size;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;
	if (adaptive_step) {
		A_local = std::make_unique<SparseMatrix<Real>>(N);
		std::vector<Real> b(N, 0);
		setup_A(*A_local, alpha, grid_size, dim_size, time_step);
	}
	// perform solve
	static SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);
	std::vector<Real> x(N, 0);
	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(adaptive_step ? *A_local : *A, grid->values, x, ret_pcg_residual, ret_pcg_iterations, 2);
	// x contains the new temperature values
	grid->values = std::move(x);
}

void DiffusionSimulator::pass_time_step_variable(float time_step) {
	this->time_step = time_step;
}

void DiffusionSimulator::init_grid() {
	grid = std::make_unique<Grid>(dim_size, is_3d);
}

void DiffusionSimulator::setup_for_implicit() {
	const int N = is_3d ? dim_size * dim_size * dim_size :
		dim_size * dim_size;
	A = std::make_unique<SparseMatrix<Real>>(N);
	setup_A(*A, alpha, grid_size, dim_size, time_step, is_3d);

}

void DiffusionSimulator::simulateTimestep(float time_step) {
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		solve_explicit(time_step);
		break;
	case 2:
	case 1:
	{
		solve_implicit(time_step);
		break;
	}
	}
}

void DiffusionSimulator::externalForcesCalculations(float time_elapsed) {
}

void getHeatMapColor(float value, float* red, float* green, float* blue)
{
	const int NUM_COLORS = 7;
	static float color[NUM_COLORS][3] = { {0, 0, 0},  {0,0,1}, {0, 1, 1}, {0,1,0}, {1,1,0}, {1,0,0}, {1, 1, 1} };

	int idx1;
	int idx2;
	float fractBetween = 0;

	if (value <= 0) { idx1 = idx2 = 0; }
	else if (value >= 1) { idx1 = idx2 = NUM_COLORS - 1; }
	else
	{
		value = value * (NUM_COLORS - 1);
		idx1 = floor(value);
		idx2 = idx1 + 1;
		fractBetween = value - float(idx1);
	}

	*red = (color[idx2][0] - color[idx1][0]) * fractBetween + color[idx1][0];
	*green = (color[idx2][1] - color[idx1][1]) * fractBetween + color[idx1][1];
	*blue = (color[idx2][2] - color[idx1][2]) * fractBetween + color[idx1][2];
}

void DiffusionSimulator::draw_objects() {
	// to be implemented
	//visualization
	switch (m_iTestCase) {
	case 0:
	case 2:
	case 1:
	{
		const auto rad = 0.1f;
		for (int i = 0; i < grid->num_points; i++) {
			float r, g, b;
			getHeatMapColor(grid->values[i] / 100, &r, &g, &b);
			DUC->setUpLighting(Vec3(r, g, b), Vec3(), 1, Vec3(0.1, 0.1, 0.1));
			DUC->drawSphere(grid->pos_internal[i], { rad,rad,rad });
		}
	}
	break;
	default:
		break;
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* context) {
	draw_objects();
}

void DiffusionSimulator::onClick(int x, int y) {
	track_mouse.x = x;
	track_mouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y) {
	old_track_mouse.x = x;
	old_track_mouse.y = y;
	track_mouse.x = x;
	track_mouse.y = y;
}
