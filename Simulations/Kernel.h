#pragma once
#include "DrawingUtilitiesClass.h"
#include <math.h>

// https://pysph.readthedocs.io/en/latest/reference/kernels.html
class CubicSplineKernel {
public:

	void init(Real support_radius) {
		half_radius = support_radius / 2;
		Real hr2 = half_radius * half_radius;
		Real hr3 = hr2 * half_radius;
		k = 3 / (2 * hr3 * M_PI);
		k_grad = 9 / (4 * M_PI * (hr3 * hr2));

	}
	Real W(const Real dist) {
		Real q = dist / half_radius;
		Real q2 = q * q;
		Real q3 = q2 * q;
		Real res = 0;
		if (q < 2.0) {
			if (q < 1) {
				res = 2.0 / 3.0 - q2 + q3 / 2.0;
			} else {
				res = 1.0 / 6.0 * pow(2 - q, 3.0);
			}
			res *= k;
		}
		return res;
	}
	Vec3 W_grad(Vec3 dist) {
		Real dist_norm = normalize(dist);
		Real q = dist_norm / half_radius;

		Vec3 res = Vec3();
		if (q < 2.0) {
			res = k_grad * dist;
			if (q < 1) {
				res *= (q - 4.0 / 3.0) * q * half_radius;
			} else {
				auto test = -pow((2.0 - q), 2.0);
				res *= -pow((2.0 - q), 2.0) * half_radius / 3.0;
			}
		}
		return res;
	}
	void set_support_radius(const Real support_radius) {
		this->support_radius = support_radius;
	}
private:
	Real support_radius;
	Real half_radius;
	Real k;
	Real k_grad;
};

class CubicSplineKernel2 {
public:

	void init(Real support_radius) {
		this->support_radius = support_radius;
		Real hr2 = support_radius * support_radius;
		Real hr3 = hr2 * support_radius;
		k = 8 / (M_PI * hr3);
		k_grad = 48 / (M_PI * hr3);

	}
	Real W(const Real dist) {
		const Real q = dist / support_radius;
		Real res = 0;
		if (q <= 1.0) {
			if (q <= 0.5) {
				const Real q2 = q * q;
				const Real q3 = q2 * q;
				res = k * (6.0 * q3 - 6.0 * q2 + 1.0);
			} else {
				res = k * (2.0 * pow(1.0 - q, 3.0));
			}
		}
		return res;
	}
	Vec3 W_grad(const Vec3& dist) {
		Vec3 res;
		const Real rl = norm(dist);
		const Real q = rl / support_radius;
		if ((rl > 1.0e-5) && (q <= 1.0))
		{
			const Vec3 delq = dist * (static_cast<Real>(1.0) / (rl * support_radius));
			if (q <= 0.5) {
				res = k_grad * q * (3.0 * q - 2.0) * delq;
			} else {
				const Real factor = 1.0 - q;
				res = k_grad * (-factor * factor) * delq;
			}
		}
		return res;
	}
	void set_support_radius(const Real support_radius) {
		this->support_radius = support_radius;
	}
private:
	Real support_radius;
	Real k;
	Real k_grad;
};
