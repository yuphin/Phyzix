#include "Camera.h"

Camera::Camera(Vec3& eye, Vec3& look_at, float fov) : fov(fov){
	this->look_at(eye, look_at, Vec3(0,1,0));
}

void Camera::look_at(const Vec3& pos, const Vec3& target, const Vec3& up) {
	this->pos = pos;
	this->forward = target - pos;
	normalize(this->forward);
	this->right = cross(up, this->forward);
	normalize(this->right);
	this->up = cross(this->forward, this->right);
	normalize(this->up);
	this->forward /= tan(0.5 * fov);
}
