#pragma once
#include <Eigen>
#include "Ray.h"
#include "Material.h"
struct hit_record { //��ȡ��ײ��Ϣ
	float t; //tֵ���ڻ����ȷ���ڵ���ϵ
	Eigen::Vector3f hit_point;
	Eigen::Vector3f hit_normal;
	material* mat_ptr;
};

class hittable { //����࣬������̳к���д��ײ����
public:
	virtual bool hit( ray& r,float t_min, float t_max, hit_record& rec)const = 0;

};