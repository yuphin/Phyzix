#pragma once
#include "DrawingUtilitiesClass.h"
class Camera {
public:
	Camera(Vec3f& eye, Vec3f& target, float fov);
	Vec3f pos;
	Vec3f up;
	Vec3f right;
	Vec3f forward;
	float fov;
	bool moving;
private:
	void look_at(const Vec3f& pos, const Vec3f& target, const Vec3f& up);

};
