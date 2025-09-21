#pragma once
#include <Eigen>
#include "Hit.h"

class sphere:public hittable { //定义球类
public:
	Eigen::Vector3f center;
	float radius;
	material* mat_ptr;
	sphere() {}
	sphere(Eigen::Vector3f m_center, float m_radius, material* m) { center = m_center, radius = m_radius; mat_ptr = m; }
	virtual bool hit(ray& r, float t_min, float t_max, hit_record& rec) const { //重写hit函数
		Eigen::Vector3f oc = r.origen() - center;
		float a = r.direction().dot(r.direction());
		float b = 2*oc.dot(r.direction());
		float c = oc.dot(oc) - radius * radius;
		float deleta = b * b -   4*a * c;
		if (deleta > 0)
		{
			float temp = (-b - sqrt(deleta)) / (2.0 * a); //小根
			if (temp > t_min && temp < t_max) {
				//输入碰撞信息
				rec.t = temp;
				rec.hit_point = r.point_at_paramenter(temp);
				rec.hit_normal = (rec.hit_point - center)/radius; //法线向量
				rec.mat_ptr = mat_ptr;
				return true;
			}
			temp = (-b + sqrt(deleta)) / (2.0 * a); //大根
			if (temp > t_min && temp < t_max)
			{
				//输入碰撞信息
				rec.t = temp;
				rec.hit_point = r.point_at_paramenter(temp);
				rec.hit_normal = (rec.hit_point - center)/radius; //法线向量
				rec.mat_ptr = mat_ptr;
				return true;
			}
			

		}
		return false;




	}

};