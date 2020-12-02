#pragma once
#include "DrawingUtilitiesClass.h"
class Camera {
public:
	Camera(Vec3& eye, Vec3& target, float fov);
	Vec3 pos;
	Vec3 up;
	Vec3 right;
	Vec3 forward;
	float fov;
	bool moving;
private:
	void look_at(const Vec3& pos, const Vec3& target, const Vec3& up);

};
