#include "Camera.h"

Camera::Camera(Vec3f& eye, Vec3f& look_at, float fov) : fov(fov){
	this->look_at(eye, look_at, Vec3f(0,1,0));
}

void Camera::look_at(const Vec3f& pos, const Vec3f& target, const Vec3f& up) {
	this->pos = pos;
	this->forward = target - pos;
	normalize(this->forward);
	this->right = cross(up, this->forward);
	normalize(this->right);
	this->up = cross(this->forward, this->right);
	normalize(this->up);
	this->forward /= tan(0.5 * fov);
}
