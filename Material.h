#pragma once
#include "Ray.h"
#include "Hit.h"

struct hit_record; //Ç°ÏòÉùÃ÷

class material {
public:
	virtual bool scatter( ray& ray_in,hit_record& rec, Eigen::Vector3f& attenuation, ray& scattered) = 0;
};

