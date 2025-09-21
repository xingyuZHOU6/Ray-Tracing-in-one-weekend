#pragma once
#include <Eigen>
#include "Ray.h"
#include "Material.h"
struct hit_record { //获取碰撞信息
	float t; //t值用于获得正确的遮挡关系
	Eigen::Vector3f hit_point;
	Eigen::Vector3f hit_normal;
	material* mat_ptr;
};

class hittable { //虚基类，被子类继承后重写碰撞函数
public:
	virtual bool hit( ray& r,float t_min, float t_max, hit_record& rec)const = 0;

};