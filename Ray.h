#pragma once
#include <Eigen>
//定义光线类,用于定义光线
class ray {
public:
	Eigen::Vector3f A;
	Eigen::Vector3f B;
	ray() {} //默认的构造函数
	ray( const Eigen::Vector3f& orign, const Eigen::Vector3f& direction) { A = orign; B = direction; }
	Eigen::Vector3f origen() { return A; }
	Eigen::Vector3f direction() { return B; }
	Eigen::Vector3f point_at_paramenter(float t) { return A + t * B; } //返回t时间的光线坐标点

};