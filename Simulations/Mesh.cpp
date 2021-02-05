#include "Mesh.h"
#include "util/util.h"
static Real area(const XMFLOAT3& v1, const XMFLOAT3& v2, const XMFLOAT3& v3) {
	return 0.5 * norm(cross(
		Vec3(v2.x, v2.y, v2.z) - Vec3(v1.x, v1.y, v1.z),
		Vec3(v3.x, v3.y, v3.z) - Vec3(v1.x, v1.y, v1.z)
	));
}
static int search(const std::vector<Real>& vec, Real sample) {
	int l = 0;
	int r = vec.size();
	while (l <= r) {
		int m = (l + r) / 2;
		if (vec[m] < sample && m < vec.size()) {
			l = m + 1;
		} else {
			if (m == 0) return 0;
			if (vec[m - 1] <= sample) {
				return m;
			}
			r = m - 1;
		}
	}
	return -1;
}
void Mesh::calculate_triangles() {
	for (int i = 0; i < indices.size(); i += 3) {
		triangles.push_back(
			{ indices[i], indices[i + 1], indices[i + 2] }
		);
	}
}
void Mesh::sample_mesh() {
	const int num_samples = 512;
	const int num_tris = indices.size() / 3;
	double a;
	std::vector<Real> area_cumulative(num_tris, 0);
	const auto& t0v1 = vertices[indices[0]].position;
	const auto& t0v2 = vertices[indices[1]].position;
	const auto& t0v3 = vertices[indices[2]].position;
	area_cumulative[0] = area(t0v1, t0v2, t0v3);
	int j = 1;
	for (int i = 3; i < indices.size(); i += 3) {
		const auto& v1 = vertices[indices[i]].position;
		const auto& v2 = vertices[indices[i + 1]].position;
		const auto& v3 = vertices[indices[i + 2]].position;
		area_cumulative[j] = area_cumulative[j - 1] +
			area(v1, v2, v3);
		j++;
	}
	for (int i = 0; i < num_samples; i++) {
		int face_idx;
		Real sample = area_cumulative.back() * ((Real)rand() / (Real)RAND_MAX);
		face_idx = search(area_cumulative, sample);
		//for (face_idx = 0; area_cumulative[face_idx] < sample && face_idx < area_cumulative.size(); face_idx++);
		XMFLOAT3 v1, v2, v3, p;
		v1 = vertices[indices[face_idx * 3]].position;
		v2 = vertices[indices[face_idx * 3 + 1]].position;
		v3 = vertices[indices[face_idx * 3 + 2]].position;
		Real alpha = ((Real)rand() / (Real)RAND_MAX);
		Real beta = ((Real)rand() / (Real)RAND_MAX);
		Real a, b, c;
		// Sampling a triangle
		a = 1 - sqrt(beta);
		b = sqrt(beta) * (1 - alpha);
		c = sqrt(beta) * alpha;
		p = v1 * a + v2 * b + v3 * c;
		samples.push_back({ p.x, p.y, p.z });
	}
}
