#pragma once
#include <Eigen>
//���������,���ڶ������
class ray {
public:
	Eigen::Vector3f A;
	Eigen::Vector3f B;
	ray() {} //Ĭ�ϵĹ��캯��
	ray( const Eigen::Vector3f& orign, const Eigen::Vector3f& direction) { A = orign; B = direction; }
	Eigen::Vector3f origen() { return A; }
	Eigen::Vector3f direction() { return B; }
	Eigen::Vector3f point_at_paramenter(float t) { return A + t * B; } //����tʱ��Ĺ��������

};