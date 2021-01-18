#include "DiffusionSimulator.h"
#include "util/util.h"
using namespace std;

Grid::Grid(int dim_x, int dim_y, int dim_z, bool is_3d) : dim_x(dim_x), is_3d(is_3d), 
	dim_y(dim_y), dim_z(dim_z) {
	num_points = is_3d ? dim_x * dim_y * dim_z : dim_x * dim_y;
	values.resize(num_points);
	vals_gpu.resize(num_points);
	positions.resize(num_points);
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
	values[3] = 1e10;

	// R = 0.1
	// For easier specification of positions, we naively assign position values
	Vec3 L = Vec3(-0.1 * dim_x, 0, 0);
	Vec3 T = Vec3(0, 0.1 * dim_y, 0);
	int k_iter = is_3d ? dim_z : 1;
	int dim_sqr = dim_x * dim_y;
	for (int k = 0; k < k_iter; k++) {
		int k_offset = dim_sqr * k;
		for (int i = 0; i < dim_y - 1; i += 2) {
			auto offsety = Vec3(0, i * 0.1, 0);
			for (int j = 0; j < dim_x; j++) {
				auto offsetx = Vec3(j * 0.1, 0, 0);
				positions[k_offset + dim_x * i + j] = L + offsetx - Vec3(0, 0, k * 0.1);
				positions[k_offset + dim_x * i + j] += T - offsety;
				positions[k_offset + dim_x * (i + 1) + j] =
					positions[k_offset + dim_x * i + j] - Vec3(0, 0.1, 0);
			}
		}
		if (dim_y % 2) {
			Vec3 B = Vec3(0, 0.1, 0);
			for (int i = 0; i < dim_x; i++) {
				positions[k_offset + (dim_y - 1) * dim_x + i] =
					B + L + Vec3(i * 0.1, 0, -k * 0.1);
			}
		}
	}
	pos_internal.resize(num_points);
	//int dim_z = is_3d ? dim_z : 1;
	boundary_indices.resize( 2 * (dim_y + dim_x -2) * (dim_z > 1 ? 2 : 1) + (dim_z > 1 ? 4 * (dim_z -2 ) : 0 ));
	auto idxs = get_internal_repr_idxs();
	// We map previous position values to an internal representation.
	// NOTE: This assumes fixed positions. For a dynamic grid we'd need to calculate 
	// internal positions on the fly or set them according to the mapping in the first place.
	for (int i = 0; i < num_points; i++) {
		pos_internal[i] = positions[idxs[i]];
	}
	set_boundary_indices(dim_sqr, is_3d ? dim_z : 1);
}

std::vector<Real> Grid::create_new()
{
	std::vector<Real> result;
	result.resize(num_points);
	result[3] = 1e10;
	return result;
}

std::vector<int> Grid::get_internal_repr_idxs() {

	std::vector<int> result;
	result.resize(num_points);
	auto idx = 0;
	int dim_sqr = dim_x * dim_y;
	int k_iter = is_3d ? dim_z : 1;
	for (int k = 0; k < k_iter; k++) {
		idx = 0;
		int k_offset = dim_sqr * k;
		for (int i = 0; i < dim_y / 2; i++) {
			for (int j = 0; j < dim_x; j++) {
				result[k_offset + idx] = k_offset + i * 2 * dim_x + j;
				result[k_offset + idx + 1] = k_offset + i * 2 * dim_x + j + dim_x;
				idx += 2;
			}
		}
		if (dim_y % 2) {
			for (int i = 0; i < dim_x; i++) {
				result[dim_sqr * k + idx + i] = dim_sqr * k + idx + i;
			}
		}
	}
	return result;
}

void Grid::set_boundary_indices(int dim_sqr, int dim_z) {
	
	int idx = 0;
	for (int i = 0; i < 2 * dim_x; i += 2) {
	
		boundary_indices[idx++] = i;
		if (i != 0 && i != 2 * (dim_x - 1)) {
			// Far top
			if (dim_z > 1) {
				boundary_indices[idx++] = i + (dim_z - 1) * dim_sqr;
			}
		}
	}
	for (int i = dim_sqr; i > dim_sqr - 2 * dim_x; i--) {
	
		if (dim_y % 2 && i >= (dim_sqr - dim_x)) {
			if (i == dim_sqr) {
				continue;
			}
			boundary_indices[idx++] = i;
			if (dim_z > 1 && (i != dim_sqr - 1 && i != (dim_sqr - dim_x))) {
				// Far bottom
				boundary_indices[idx++] = i + (dim_z - 1) * dim_sqr;
			}

		} else if (dim_y % 2 == 0 && i % 2) {
			boundary_indices[idx++] = i;
			if (dim_z > 1 && (i != dim_sqr - 1 && i != (dim_sqr - 1 - 2 * (dim_x - 1)))) {
				// Far bottom
				boundary_indices[idx++] = i + (dim_z - 1) * dim_sqr;
			}
		}
	}
	int r = 0;
	for (int i = 1; i <= dim_y - 2; i++) {
	
		if (i % 2) {
			boundary_indices[idx++] = 1 + r * dim_x * 2;
			boundary_indices[idx] = boundary_indices[idx - 1] + (dim_x * 2 - 2);
			idx++;
			// Far left
			if (dim_z > 1) {
				boundary_indices[idx++] = 1 + r * dim_x * 2 + (dim_z - 1) * dim_sqr;
				boundary_indices[idx] = boundary_indices[idx - 3] + (dim_x * 2 - 2) + (dim_z - 1) * dim_sqr;
				idx++;
			}

			r++;
		} else {
			boundary_indices[idx++] = r * dim_x * 2;
			boundary_indices[idx] = boundary_indices[idx - 1] + (dim_x * 2 - 2);
			idx++;
			// Far right
			if (dim_z > 1) {
				boundary_indices[idx++] = r * dim_x * 2 + (dim_z - 1) * dim_sqr;
				boundary_indices[idx] = boundary_indices[idx - 3] + (dim_x * 2 - 2) + (dim_z - 1) * dim_sqr;
				idx++;
			}
		}
	}
	for (int k = 1; k < dim_z; k++) {
	
		int k_offset = dim_sqr * k;
		boundary_indices[idx++] = k_offset;
		boundary_indices[idx++] = k_offset + dim_x * 2 - 2;
		boundary_indices[idx++] = k_offset + dim_sqr - 1;
		boundary_indices[idx++] = k_offset + (dim_y % 2 ? dim_sqr - dim_x : dim_sqr - (dim_x * 2 - 1));
	}
}

DiffusionSimulator::DiffusionSimulator(bool adaptive_step) {
	m_iTestCase = 0;
	movable_obj_pos = Vec3();
	movable_obj_final_pos = Vec3();
	rotate = Vec3();
	this->adaptive_step = adaptive_step;
	this->DUC = nullptr;
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit solver, Implicit solver, Implicit 3D, 3D GPU(Jacobi + PCG)";
}

void DiffusionSimulator::reset() {
	mouse.x = mouse.y = 0;
	track_mouse.x = track_mouse.y = 0;
	old_track_mouse.x = old_track_mouse.y = 0;

}

void TW_CALL DiffusionSimulator::set_dim_size(const void* value, void* client_data) {
	ClientData* client_p = static_cast<ClientData*>(client_data);
	*(client_p->dim_size) = *(static_cast<const int*>(value));
	*(client_p->dim_x) = *(static_cast<const int*>(value));
	*(client_p->dim_y) = *(static_cast<const int*>(value));
	*(client_p->dim_z) = *(static_cast<const int*>(value));
	client_p->self->init_grid();
	if (client_p->self->m_iTestCase >= 1) {
		client_p->self->setup_for_implicit();
	}
}

void TW_CALL DiffusionSimulator::set_dim_x(const void* value, void* client_data) {
	ClientData* client_p = static_cast<ClientData*>(client_data);
	*(client_p->dim_x) = *(static_cast<const int*>(value));
	client_p->self->init_grid();
	if (client_p->self->m_iTestCase >= 1) {
		client_p->self->setup_for_implicit();
	}
}

void TW_CALL DiffusionSimulator::set_dim_y(const void* value, void* client_data) {
	ClientData* client_p = static_cast<ClientData*>(client_data);
	*(client_p->dim_y) = *(static_cast<const int*>(value));
	client_p->self->init_grid();
	if (client_p->self->m_iTestCase >= 1) {
		client_p->self->setup_for_implicit();
	}
}

void TW_CALL DiffusionSimulator::set_dim_z(const void* value, void* client_data) {
	ClientData* client_p = static_cast<ClientData*>(client_data);
	*(client_p->dim_z) = *(static_cast<const int*>(value));
	client_p->self->init_grid();
	if (client_p->self->m_iTestCase >= 1) {
		client_p->self->setup_for_implicit();
	}
}

void TW_CALL DiffusionSimulator::get_dim_size(void* value, void* client_data) {
	int* val_p = static_cast<int*>(value);
	*val_p = *((static_cast<ClientData*>(client_data))->dim_size);
}

void TW_CALL DiffusionSimulator::get_dim_x(void* value, void* client_data) {
	int* val_p = static_cast<int*>(value);
	*val_p = *((static_cast<ClientData*>(client_data))->dim_x);
}

void TW_CALL DiffusionSimulator::get_dim_y(void* value, void* client_data) {
	int* val_p = static_cast<int*>(value);
	*val_p = *((static_cast<ClientData*>(client_data))->dim_y);
}

void TW_CALL DiffusionSimulator::get_dim_z(void* value, void* client_data) {
	int* val_p = static_cast<int*>(value);
	*val_p = *((static_cast<ClientData*>(client_data))->dim_z);
}

void TW_CALL DiffusionSimulator::set_cg_iters(const void* value, void* client_data) {
	ClientData* client = static_cast<ClientData*>(client_data);
	client->self->num_cg_iters = *static_cast<const int*>(value);
}

void TW_CALL DiffusionSimulator::get_cg_iters(void* value, void* client_data) {
	int* val = static_cast<int*>(value);
	*val = *((static_cast<ClientData*>(client_data))->cg_iters);
}

void TW_CALL DiffusionSimulator::set_jacobi_iters(const void* value, void* client_data) {
	ClientData* client = static_cast<ClientData*>(client_data);
	client->self->num_jacobi_iters = *static_cast<const int*>(value);
}

void TW_CALL DiffusionSimulator::get_jacobi_iters(void* value, void* client_data) {
	int* val = static_cast<int*>(value);
	*val = *((static_cast<ClientData*>(client_data))->jacobi_iters);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	this->context = DUC->g_pd3dImmediateContext;
	data = std::make_unique<ClientData>();
	data->dim_size = &dim_size;
	data->dim_x = &dim_x;
	data->dim_y = &dim_y;
	data->dim_z = &dim_z;
	data->jacobi_iters = &num_jacobi_iters;
	data->cg_iters = &num_cg_iters;
	data->self = this;
	TwAddVarCB(DUC->g_pTweakBar, "Grid Size", TW_TYPE_INT32, set_dim_size, 
		get_dim_size, reinterpret_cast<void*>(data.get()), "step=1 min=1");
	TwAddVarCB(DUC->g_pTweakBar, "X", TW_TYPE_INT32, set_dim_x,
		get_dim_x, reinterpret_cast<void*>(data.get()), "step=1 min=1");
	TwAddVarCB(DUC->g_pTweakBar, "Y", TW_TYPE_INT32, set_dim_y,
		get_dim_y, reinterpret_cast<void*>(data.get()), "step=1 min=1");

	if (m_iTestCase > 1) {
		TwAddVarCB(DUC->g_pTweakBar, "Z", TW_TYPE_INT32, set_dim_z,
			get_dim_z, reinterpret_cast<void*>(data.get()), "step=1 min=1");
	} else {
		TwRemoveVar(DUC->g_pTweakBar, "Z");
	}
	if (m_iTestCase >= 1) {
		TwAddVarCB(DUC->g_pTweakBar, "Num CG Iterations", TW_TYPE_INT32,
			set_cg_iters, get_cg_iters, reinterpret_cast<void*>(data.get()),
			"step=1 min=0");
	} else {
		TwRemoveVar(DUC->g_pTweakBar, "Num CG Iterations");
	}

	if (m_iTestCase == 3) {
		TwAddVarCB(DUC->g_pTweakBar, "Num Jacobi Iterations", TW_TYPE_INT32,
			set_jacobi_iters, get_jacobi_iters, reinterpret_cast<void*>(data.get()),
			"step=2 min=0");
	} else {
		TwRemoveVar(DUC->g_pTweakBar, "Num Jacobi Iterations");
	}
}

static bool is_any_boundary(int i, int j, int dim_x, int dim_y, int dim_z, bool is_3d,
	int dim_sqr, int dim_2, int rem, bool is_odd) {

	int res = i / dim_sqr;
	bool top = (i % dim_sqr) < dim_2 && (i % 2 == ((dim_sqr % 2) ? res % 2 : 0));
	bool bottom = (!is_odd && ((i % dim_sqr) > dim_sqr - dim_2 && i % 2 == 1)
		|| (is_odd && ((i % dim_sqr) >= dim_sqr - dim_x)));
	int odd_offset = is_odd ? rem * (res % 2) : 0;
	bool right = i % dim_2 == (dim_2 - 2 + odd_offset) % dim_2
		|| i % dim_2 == (dim_2 - 1 + odd_offset) % dim_2
		|| (is_odd && (i % dim_sqr == (dim_sqr - 1)));
	bool left = (!is_odd && ((i % dim_2 == 0) || i % dim_2 == 1))
		|| (is_odd && ((i % dim_sqr < (dim_sqr - dim_x) &&
			(i% dim_2 == odd_offset || i % dim_2 == (1 + odd_offset) % dim_2))))
		|| (is_odd && (i % dim_sqr == dim_sqr - dim_x));
	bool f = res == dim_z - 1;
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
		|| (is_3d && res >= dim_z - 1)) {
		if ((is_3d && (res <= dim_z - 1 && res != 0)) &&
			(((((((!tr && !br) && !tl) && !bl) && !fl) && !fr) && !bf) && !tf)) {
			return false;
		}
		return true;
	}
	return false;
}

static void setup_A(SparseMatrix<Real>& A, double alpha, Real grid_size, 
	int dim_x, int dim_y, int dim_z, float dt, bool is_3d = false) {
	// Order : j->i->(possibly k)
	const Real dx = grid_size / dim_x;
	const Real dy = grid_size / dim_x;
	const Real dz = grid_size / dim_x;
	const Real dx2 = dx * dx;
	const Real dy2 = dx * dx;
	const Real dz2 = dx * dx;
	bool is_dim_odd = dim_y % 2;
	const Real Fx = alpha * dt / (2 * dx2);
	const Real Fy = alpha * dt / (2 * dy2);
	const Real Fz = alpha * dt / (2 * dz2);
	const int N = is_3d ? dim_x * dim_y * dim_z :
		dim_x * dim_y;
	const int dim_2 = dim_x * 2;
	const int dim_sqr = dim_x * dim_y;
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
		if ((i % dim_2 == (dim_2 - 2 + odd_offset) % dim_2
			|| i % dim_2 == (dim_2 - 1 + odd_offset) % dim_2)
			|| (is_dim_odd && (i % dim_sqr == (dim_sqr - 1)))) {
			right = true;
			any_boundary = true;
		}

		if ((!is_dim_odd && ((i % dim_sqr) > dim_sqr - dim_2 && i % 2 == 1))
			|| (is_dim_odd && (i % dim_sqr) >= dim_sqr - dim_x)) {
			bottom = true;
			any_boundary = true;
		}
		if ((is_3d && res >= dim_z - 1)) {
			if (res == dim_z - 1) {
				f = true;
			}
			any_boundary = true;
		}
		if ((i % dim_sqr) < dim_2 && i % 2 == ((dim_sqr % 2) ? res % 2 : 0)) {
			any_boundary = true;
			top = true;
			A.set_element(i, i, 1);
		}
		// TODO: There is probably a huge simplification here
		if ((!is_dim_odd && ((i % dim_2 == 0) || i % dim_2 == 1))
			|| (is_dim_odd && ((i % dim_sqr < (dim_sqr - dim_x) &&
			(i % dim_2 == odd_offset || i % dim_2 == (1 + odd_offset) % dim_2))))
			|| (is_dim_odd && (i % dim_sqr == dim_sqr - dim_x))) {
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
		if ((is_3d && (res != 0 && res <= dim_z - 1)) &&
			(((((((!tr && !br) && !tl) && !bl) && !fl) && !fr) && !bf) && !tf)) {
			any_boundary = false;
		}
		if (any_boundary) {
			A.set_element(i, i, 1);
		} else {
			A.set_element(i, i, is_3d ? 1 + 2 * (Fx + Fy + Fz) : 1 + 2 * (Fx + Fy));
		}
		if (is_3d && !f) {
			int offset_z = dim_x * dim_y;
			if (!any_boundary) {
				A.set_element(i, i + offset_z, -Fz);
			}
			if (!is_any_boundary(i + offset_z, i, dim_x, dim_y, dim_z, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
				A.set_element(i + offset_z, i, -Fz);
			}
		}
	
		int row  = ((i % dim_sqr) / (2 * dim_x)) * 2 + (i % 2 ? 1 : 0);
		if (is_dim_odd && (i % dim_sqr) >= dim_sqr - dim_x) {
			row = dim_y - 1;
		}
		if (!bottom) {
		
			if (i % 2 == ((odd_offset % 2) ^ 1)) {
				int offset = (is_dim_odd && row == dim_y - 2) ? dim_2 - 1 - (((i % dim_sqr) % (2 * dim_x)) / 2) :
					dim_2 - 1;
				if (!any_boundary) {
					A.set_element(i, i + offset, -Fy);
				}
				if (!is_any_boundary(i + offset, i, dim_x, dim_y, dim_z, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
					A.set_element(i + offset, i, -Fy);
				}
			} else {
				if (!any_boundary) {
					A.set_element(i, i + 1, -Fy);
				}
				if (!is_any_boundary(i + 1, i, dim_x, dim_y, dim_z, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
					A.set_element(i + 1, i, -Fy);
				}
			}
		}
		if (!right) {
			bool last_row = row == dim_y - 1;
			int row_offset = is_dim_odd && last_row ? 1 : 2;
			if (!any_boundary) {
				A.set_element(i, i + row_offset, -Fx);
			}
			if (!is_any_boundary(i + row_offset, i, dim_x, dim_y, dim_z, is_3d, dim_sqr, dim_2, rem, is_dim_odd)) {
				A.set_element(i + row_offset, i, -Fx);
			}
		}
	}
}
void DiffusionSimulator::notifyCaseChanged(int test_case) {
	m_iTestCase = test_case;
	movable_obj_pos = Vec3(0, 0, 0);
	rotate = Vec3(0, 0, 0);

	is_3d = false;
	use_gpu = false;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		init_grid();
		break;

	case 1:
	{
		cout << "Implicit solver!\n";
init:
		
		init_grid();
		if (!adaptive_step) {
			setup_for_implicit();
		}
	}
	break;
	case 2:
	{
		cout << "3D Implicit Solver!\n";
		is_3d = true;
		goto init;
	}
	break;
	case 3:
	{
		cout << "3D GPU(Jacobi + PCG)\n";
		use_gpu = true;
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

	std::vector<Real> new_grid_values = grid->create_new();
	const int N = is_3d ? dim_x * dim_y * dim_z : dim_x * dim_y;
	const int size2d = dim_x * dim_y;

	const Real dx = grid_size / dim_x;
	const Real dy = grid_size / dim_y;
	const Real dz = grid_size / dim_z;
	const Real dx2 = dx * dx;
	const Real dy2 = dy * dy;
	const Real dz2 = dz * dz;
	bool is_dim_odd = dim_size % 2;
	// Stability conditions:
	// In 1D:
	// u_i^{t+1} = u_i^{t} + F * (u_i^{t} - 2 * u_i^{t} + u_{i-1}^{t} )
	// In nD
	// u_i^{t+1} = u_i^{t} + F * (u_ix^{t} - 2 * u_ix^{t} + u_ix{i-1}^{t} ) 
	//				+ F * (u_iy^ { t } - 2 * u_iy^ { t } + u_{ i - 1 }y^ {t})
					// ...
	// Where F = alpha * time_step / dx2;
	// Reorganizing we have
	//  u_i^{t+1} = (1 - 2 * n * F) u_i^{t} + F (u_ix^{t} + u_{i-1}x^{t} ...)
	// The stability condition is when 1 - 2 * n * F <= 0 => F <= 1/(2 * n)
	// Note that larger F values cause current cell to have negative impact hence the solution blows up

	// The limiting time_step value is
	//  alpha * time_step / dx2 <= 1/(2*n)
	// => time_step <= dx2 / (2 * n * alpha)
	constexpr double EPS = 0.01;
	Real val = dx2 / ((is_3d ? 6 : 4) * (1 + EPS) * alpha);
	if (time_step >= val) {
		time_step = val;
	}

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

	// std::cout << "starting\n";
	for (int i = 0; i < N; i++) {
		float it = grid->values[i];
		auto neutralized_i = i % size2d;

		float x_diff = 0;
		float y_diff = 0;
		float z_diff = 0;

		int y = (neutralized_i / (2 * dim_x)) * 2;
		if ((is_dim_odd && y != (dim_y - 1)) || !is_dim_odd) {
			y += neutralized_i % 2 == 0 ? 0 : 1;
		}

		int x = 0;
		x = (neutralized_i % (2 * dim_x) - (neutralized_i % 2 == 0 ? 0 : 1)) / 2;
		if (is_dim_odd && y == (dim_y - 1)) {
			// ?
			x = neutralized_i - (dim_x * (dim_y - 1));
		}

		int z = i / size2d;

		if (x == 0 || x == dim_x - 1 || y == 0 || y == dim_y - 1 || (is_3d && z == 0) || (is_3d && z == dim_z - 1)) {
			new_grid_values[i] = 0;
			continue;
		}

		bool last_row = y == (dim_y - 1);

		//x
		int x_offset = (last_row && is_dim_odd) ? 1 : 2;
		x_diff += (neutralized_i - x_offset) < 0 ? 0 : grid->values[i - x_offset];
		x_diff += (neutralized_i + x_offset) >= size2d ? 0 : grid->values[i + x_offset];
		x_diff -= 2 * it;

		//y
		if (last_row && is_dim_odd) {
			int y_offset = (dim_x * 2 - 1) - (neutralized_i - dim_x * (dim_y - 1));
			y_diff += (neutralized_i - y_offset) < 0 ? 0 : grid->values[i - y_offset];
		}
		if (y == dim_y - 2 && is_dim_odd) {
			int y_offset = (dim_x * 2 - 1) - (neutralized_i - (dim_x * (dim_y - 3) + 1)) / 2;
			y_diff += grid->values[i - 1];
			y_diff += (neutralized_i + y_offset) >= size2d ? 0 : grid->values[i + y_offset];
		} else if (neutralized_i % 2 == 0) {
			y_diff += (neutralized_i - dim_x * 2 + 1) < 0 ? 0 : grid->values[i - dim_x * 2 + 1];
			y_diff += grid->values[i + 1];
		} else {
			y_diff += grid->values[i - 1];
			y_diff += (neutralized_i + dim_x * 2 - 1) >= size2d ? 0 : grid->values[i + dim_x * 2 - 1];
		}
		y_diff -= 2 * it;

		//z
		if (is_3d) {
			z_diff += (i - dim_x * dim_y) < 0 ? 0 : grid->values[i - dim_x * dim_y];
			z_diff += (i + dim_x * dim_y) >= N ? 0 : grid->values[i + dim_x * dim_y];
			z_diff -= 2 * it;
		}
		//const Real F = alpha * time_step / (dx2);
		new_grid_values[i] = (x_diff / dx2 + y_diff / dy2 + z_diff / dz2) * alpha * time_step + it;
	}

	grid->values = std::move(new_grid_values);

	return nullptr;
}

void DiffusionSimulator::solve_implicit(float time_step) {
	// solve A T = b
	std::unique_ptr<SparseMatrix<Real>> A_local = nullptr;
	constexpr Real pcg_target_residual = 1e-3;
	int pcg_max_iterations = num_cg_iters;
	const int N = is_3d ? dim_x * dim_y * dim_z : dim_x * dim_y;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;
	if (adaptive_step) {
		A_local = std::make_unique<SparseMatrix<Real>>(N);
		std::vector<Real> b(N, 0);
		setup_A(*A_local, alpha, grid_size, dim_x, dim_y, dim_z, time_step);
	}
	// perform solve
	static SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);
	std::vector<Real> x(N, 0);
	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(adaptive_step ? *A_local : *A, grid->values, x, ret_pcg_residual, ret_pcg_iterations, 2);
	if (use_gpu) {
		// ---Begin Jacobi---
		if (!fsm) {
			setup_for_jacobi(solver.fixed_matrix);
			fill_static_resources();
		}
		fill_dynamic_resources(x);

		ID3D11ShaderResourceView* srvs[2] = { x_in_srv, x_out_srv };
		ID3D11UnorderedAccessView* uavs[2] = { x_out_uav, x_in_uav };
		ID3D11ShaderResourceView* null_srv[1] = { nullptr };
		ID3D11UnorderedAccessView* null_uav[1] = { nullptr };
		bool srv_idx = 1;
		bool uav_idx = 1;
		context->CSSetShader(compute_shader.Get(), nullptr, 0);
		constexpr int NUM_THREADS_X = 32;
		const int TG_X = ceil(((float)fsm->n / NUM_THREADS_X));
		context->CSSetShaderResources(1, 1, &rowstart_srv);
		context->CSSetShaderResources(2, 1, &colindex_srv);
		context->CSSetShaderResources(3, 1, &mat_values_srv);
		context->CSSetShaderResources(4, 1, &rhs_srv);
		cb.N = N;
		context->UpdateSubresource(
			diffusion_cb.Get(),
			0,
			nullptr,
			&cb,
			0,
			0
		);
		context->CSSetConstantBuffers(0, 1, diffusion_cb.GetAddressOf());
		for (int i = 0; i < num_jacobi_iters; i++) {
			srv_idx ^= 1;
			uav_idx ^= 1;
			context->CSSetShaderResources(0, 1, &srvs[srv_idx]);
			context->CSSetUnorderedAccessViews(0, 1, &uavs[uav_idx], nullptr);
			context->Dispatch(TG_X, 1, 1);
			context->CSSetShaderResources(0, 1, null_srv);
			context->CSSetUnorderedAccessViews(0, 1, null_uav, nullptr);
		}
		// Copy the output to original vector
		D3D11_MAPPED_SUBRESOURCE mapped_resource;
		ID3D11Buffer* staging_buf = CreateAndCopyToDebugBuf(device,
			context, x_in_buf.Get());
		float* out;
		context->Map(staging_buf, 0, D3D11_MAP_READ, 0, &mapped_resource);
		out = (float*)mapped_resource.pData;
		//std::vector<float> debug;
		//debug.assign(out, out + grid->num_points);
		for (int i = 0; i < grid->values.size(); i++) {
			x[i] = out[i];
		}
		staging_buf->Release();
		// ---End Jacobi---
	}

	// Enforce boundary conditions (because of inaccuracies)
	for (const auto& idx : grid->boundary_indices) {
		x[idx] = 0;
	}
	grid->values = std::move(x);
}

void DiffusionSimulator::pass_time_step_variable(float time_step) {
	this->time_step = time_step;
}

void DiffusionSimulator::init_resources(ID3D11Device* device) {
	this->device = device;
	HRESULT hr = S_OK;
	ID3DBlob* cs_blob = nullptr;
	ID3DBlob* err_blob = nullptr;
	// Create Compute Shader
	hr = D3DCompileFromFile(L"jacobi_heat.hlsl", NULL, NULL, "CS", "cs_5_0", 0, 0, &cs_blob, &err_blob);
	if (FAILED(hr)) {
		if (err_blob) {
			printf("Error %s\n", err_blob->GetBufferPointer());
		}
	}
	device->CreateComputeShader(cs_blob->GetBufferPointer(), cs_blob->GetBufferSize(), NULL, &compute_shader);
	CD3D11_BUFFER_DESC cb_desc(
		sizeof(DiffusionCB),
		D3D11_BIND_CONSTANT_BUFFER
	);
	hr = device->CreateBuffer(&cb_desc, nullptr, diffusion_cb.GetAddressOf());
}

void DiffusionSimulator::fill_static_resources() {
	// Rowstart
	create_structured_buffer(device, sizeof(int), fsm->rowstart.size(),
		fsm->rowstart.data(), rowstart_buf.GetAddressOf());
	create_srv(device, rowstart_buf.Get(), &rowstart_srv);
	// Col index
	create_structured_buffer(device, sizeof(int), fsm->colindex.size(),
		fsm->colindex.data(), colindex_buf.GetAddressOf());
	create_srv(device, colindex_buf.Get(), &colindex_srv);
	// Mat values
	create_structured_buffer(device, sizeof(int), fsm->value.size(),
		fsm->value.data(), mat_values_buf.GetAddressOf());
	create_srv(device, mat_values_buf.Get(), &mat_values_srv);
	// X_out
	create_structured_buffer(device, sizeof(float), grid->num_points,
		0, x_out_buf.GetAddressOf());
	create_srv(device, x_out_buf.Get(), &x_out_srv);
	create_uav(device, x_out_buf.Get(), &x_out_uav);
}

void DiffusionSimulator::fill_dynamic_resources(const std::vector<Real>& x) {
	for (int i = 0; i < grid->values.size(); i++) {
		grid->vals_gpu[i] = static_cast<float>(grid->values[i]);
		x_gpu[i] = static_cast<float>(x[i]);
	}

	if (!rhs_buf) {
		create_structured_buffer(device, sizeof(float), grid->num_points,
			grid->vals_gpu.data(), rhs_buf.GetAddressOf());
		create_srv(device, rhs_buf.Get(), &rhs_srv);
	} else {
		context->UpdateSubresource(rhs_buf.Get(), 0, nullptr,
			grid->vals_gpu.data(), 0, 0);
	}

	if (!x_in_buf) {
		create_structured_buffer(device, sizeof(float), grid->num_points,
			x_gpu.data(), x_in_buf.GetAddressOf());
		create_srv(device, x_in_buf.Get(), &x_in_srv);
		create_uav(device, x_in_buf.Get(), &x_in_uav);
	} else {
		context->UpdateSubresource(x_in_buf.Get(), 0, nullptr,
			x_gpu.data(), 0, 0);
	}
}

void DiffusionSimulator::init_grid() {
	grid = std::make_unique<Grid>(dim_x, dim_y, dim_z, is_3d);
}

void DiffusionSimulator::setup_for_implicit() {
	const int N = is_3d ? dim_x * dim_y * dim_z :
		dim_x * dim_y;
	fsm.reset();
	free_resources();
	A = std::make_unique<SparseMatrix<Real>>(N);
	setup_A(*A, alpha, grid_size, dim_x, dim_y, dim_z, time_step, is_3d);
	x_gpu.resize(grid->num_points);
}

void DiffusionSimulator::setup_for_jacobi(const FixedSparseMatrix<Real>& mat) {
	fsm = std::make_unique<FixedSparseMatrix<float>>();
	fsm->n = mat.n;
	fsm->colindex = mat.colindex;
	fsm->rowstart = mat.rowstart;
	fsm->value.resize(mat.value.size());
	for (int i = 0; i < mat.value.size(); i++) {
		fsm->value[i] = static_cast<float>(mat.value[i]);
	}
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
	case 3:
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

	if (value <= 0) { idx1 = idx2 = 0; } else if (value >= 1) { idx1 = idx2 = NUM_COLORS - 1; } else
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
	case 3:
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

void DiffusionSimulator::free_resources() {
	if (rowstart_srv) rowstart_srv->Release();
	if (colindex_srv) colindex_srv->Release();
	if (mat_values_srv) mat_values_srv->Release();
	if (rhs_srv) rhs_srv->Release();
	if (x_in_srv) x_in_srv->Release();
	if (x_in_uav) x_in_uav->Release();
	if (x_out_srv) x_out_srv->Release();
	if (x_out_uav) x_out_uav->Release();
	rowstart_srv = nullptr;
	colindex_srv = nullptr;
	mat_values_srv = nullptr;
	rhs_srv = nullptr;
	x_in_srv = nullptr;
	x_in_uav = nullptr;
	x_out_srv = nullptr;
	x_out_uav = nullptr;
	rowstart_buf.Reset();
	colindex_buf.Reset();
	mat_values_buf.Reset();
	rhs_buf.Reset();
	x_in_buf.Reset();
	x_out_buf.Reset();
}

DiffusionSimulator::~DiffusionSimulator() {
	free_resources();
}
