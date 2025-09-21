#include <iostream>
#include <Eigen>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "Ray.h"
#include "Sphere.h"
#include "Hittable_list.h"
#include <random>

using namespace std;



thread_local std::random_device rd;
thread_local std::mt19937 gen(rd());
thread_local std::uniform_real_distribution<float>dis(-1.0f, 1.0f);


Eigen::Vector3f random_in_unit_sphere()
{
	


	Eigen::Vector3f s;
	do {
		s = Eigen::Vector3f(dis(gen), dis(gen), dis(gen));
		
	
	} while (s.norm() >= 1.0f);
	return s;
}

class lambertian :public material { //漫反射材质的处理方法
public:
	Eigen::Vector3f albedo; //漫反射率，
	lambertian() {}
	lambertian(const Eigen::Vector3f& a) { albedo = a; }
	 virtual bool scatter( ray& ray_in,  hit_record& rec, Eigen::Vector3f& attenuation, ray& scattered)
	{
		Eigen::Vector3f s = rec.hit_point + rec.hit_normal + random_in_unit_sphere();
		scattered = ray(rec.hit_point, (s - rec.hit_point).normalized());
		attenuation = albedo;
		return true;
	}
};


Eigen::Vector3f reflect(const Eigen::Vector3f& v, const Eigen::Vector3f& n)
{
	return v - 2 * v.dot(n) * n;
}

bool refract(const Eigen::Vector3f& v, const Eigen::Vector3f& n, float ni_over_nt,Eigen::Vector3f& refracted)
{
	Eigen::Vector3f v_norm = v.normalized();
	Eigen::Vector3f n_norm = n.normalized();
	float c1 = n_norm.dot(v_norm);
	float c2 = 1.0f - ni_over_nt * ni_over_nt * (1 - c1 * c1);
	if (c2 > 0) //是否发生折射的条件
	{
		refracted = ni_over_nt * (v_norm - c1 * n_norm) - n_norm * sqrt(c2);
		return true;
	}
	return false;
}
float schlick(float cosine, float n1_over_nt)
{
	float r0 = (1 - n1_over_nt) / (1 + n1_over_nt);
	r0 *= r0;
	return r0 + (1 - r0) * pow((1 - cosine), 5);
}


class metal :public material {
public:
	Eigen::Vector3f albedo;
	float fuzz;
	metal() {}
	metal(const Eigen::Vector3f &a,float f) {
		albedo = a;
		fuzz = f;
		if (f > 1)fuzz = 1;
		
		
	}
	virtual bool scatter(ray& ray_in, hit_record& rec, Eigen::Vector3f& attenuation, ray& scattered)
	{
		Eigen::Vector3f v = ray_in.direction().normalized();
		Eigen::Vector3f normal = rec.hit_normal;
		Eigen::Vector3f p = rec.hit_point;
		Eigen::Vector3f r = reflect(v,normal);
		Eigen::Vector3f offset = fuzz * random_in_unit_sphere(); //实现金属的哑金效果,不让反射那么的完美，给一个偏移量
		scattered = ray(p, (r + offset).normalized());
		attenuation = albedo;
		return (r.dot(normal) > 0);

	}

};

class dielectric :public material {
public:
	float ref_idx;  //n2/n1 从水中射入空气
	dielectric(float ri) { ref_idx = ri; }
	virtual bool scatter(ray& ray_in, hit_record& rec, Eigen::Vector3f& attenuation, ray& scattered) {
		Eigen::Vector3f outward_normal;
		Eigen::Vector3f reflected = reflect(ray_in.direction(), rec.hit_normal);
		float ni_over_nt;
		attenuation = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
		Eigen::Vector3f refracted;

		float reflect_prob;
		float cosine;

		if (ray_in.direction().dot(rec.hit_normal) > 0) //从里到外
		{
			outward_normal = -rec.hit_normal.normalized();
			ni_over_nt = ref_idx;
			cosine = -ref_idx * ray_in.direction().normalized().dot(outward_normal);
		}
		else {
			outward_normal = rec.hit_normal.normalized();
			ni_over_nt = 1.0f / ref_idx;
			cosine = -ray_in.direction().normalized().dot(outward_normal);
		
		}
		if (refract(ray_in.direction(), outward_normal, ni_over_nt, refracted))
		{
			reflect_prob = schlick(cosine,ref_idx);
			
		}
		else {
			reflect_prob = 1.0f;
		}
		if (0.5 * (dis(gen) + 1.0f) < reflect_prob)
		{
			scattered = ray(rec.hit_point, reflected);
		}
		else {
			scattered = ray(rec.hit_point, refracted);
		
		}
		return true;

	
	
	
	}


};



Eigen::Vector3f getcolor( ray& r,hittable* world,int max_height)
{
	hit_record rec;
	if (world->hit(r, 0.001, std::numeric_limits<float>::max(), rec)) { //命中物体
		ray scattered; //散射光线
		Eigen::Vector3f attenuation; //漫反射率 

		if (max_height < 50&& rec.mat_ptr->scatter(r, rec, attenuation, scattered)) //递归次数的限定
		{
			return attenuation.cwiseProduct(getcolor(scattered, world, max_height + 1)); //递归,每次反射减少一半能量
		}
		else {
			return Eigen::Vector3f(0, 0, 0); //超过递归深度，直接返回
		}
	}
	float t;
	Eigen::Vector3f r_normalized = r.B.normalized(); //归一化
	t = 0.5 * (r_normalized.y() + 1.0f); //限制到0-1区间
	//进行线性插值
	return (1 - t) * Eigen::Vector3f(1.0,1.0,1.0) + t * Eigen::Vector3f(0.5, 0.7, 1.0); //表示t越大，显示的值越蓝
}

//封装相机类
class camera {
public:
	Eigen::Vector3f lower_left_corner; // 表示屏幕的左下角坐标
	Eigen::Vector3f horizontal; //屏幕的水平宽度
	Eigen::Vector3f vertical; //屏幕竖直高度
	Eigen::Vector3f origin; //相机原点
	float len_radius; 

	camera(Eigen::Vector3f eye, Eigen::Vector3f target,Eigen::Vector3f up,float fov,float aspect,float aperture,float focus)
	{
		len_radius = aperture / 2;
		float half_height = std::tan(fov * 3.1415926535 / 180.0f / 2);
		float half_width = half_height * aspect;

		Eigen::Vector3f g = (target - eye).normalized();
		Eigen::Vector3f t = up.normalized();
		Eigen::Vector3f right = g.cross(t).normalized();
		Eigen::Vector3f real_up = right.cross(g).normalized();

		origin = eye;
		lower_left_corner = origin - half_height * real_up*focus - half_width * right*focus + g*focus;
		horizontal = 2 * half_width * right*focus;
		vertical = 2 * half_height * real_up*focus;


	}
	ray get_ray(float u,float v)
	{
		//不使用镜头偏移
		ray r = ray(origin, lower_left_corner + u * horizontal + v * vertical-origin);
		return r;
	}


};

//随机场景函数.
hittable* random_scene()
{
	// 为场景生成创建独立的随机数生成器
	std::random_device rd_scene;
	std::mt19937 gen_scene(rd_scene());
	std::uniform_real_distribution<float> dis_scene(-1.0f, 1.0f);

	int n = 500;
	hittable** list = new hittable * [n + 1];
	list[0] = new sphere(Eigen::Vector3f(0, -1000, 0), 1000, new lambertian(Eigen::Vector3f(0.5, 0.5, 0.5)));

	int i = 1;
	for (int a = -11; a < 11; a++)
	{
		for (int b = -11; b < 11; b++)
		{
			float choose_mat = 0.5f * (dis_scene(gen_scene) + 1.0f);
			Eigen::Vector3f center(a + 0.9 * (0.5f * (dis_scene(gen_scene) + 1.0f)), 0.2f, b + 0.9 * 0.5f * (dis_scene(gen_scene) + 1.0f));

			if ((center - Eigen::Vector3f(4, 0.2, 0)).norm() > 0.9)
			{
				if (choose_mat < 0.8)
				{
					list[i++] = new sphere(center, 0.2f,
						new lambertian(Eigen::Vector3f(0.5f * (dis_scene(gen_scene) + 1.0f),
							0.5f * (dis_scene(gen_scene) + 1.0f),
							0.5f * (dis_scene(gen_scene) + 1.0f))));
				}
				else if (choose_mat < 0.9)
				{
					list[i++] = new sphere(center, 0.2f,
						new metal(Eigen::Vector3f(0.5f * (dis_scene(gen_scene) + 1.0f),
							0.5f * (dis_scene(gen_scene) + 1.0f),
							0.5f * (dis_scene(gen_scene) + 1.0f)), 0));
				}
				else
				{
					list[i++] = new sphere(center, 0.2f, new dielectric(1.5f));
				}
			}
		}
	}

	list[i++] = new sphere(Eigen::Vector3f(0, 1, 0), 1.0, new dielectric(1.5));
	list[i++] = new sphere(Eigen::Vector3f(-4, 1, 0), 1.0, new lambertian(Eigen::Vector3f(0.4, 0.2, 0.1)));
	list[i++] = new sphere(Eigen::Vector3f(4, 1, 0), 1.0, new metal(Eigen::Vector3f(0.7, 0.6, 0.5), 0.0));

	return new hittable_list(list, i);
}

//转换为全局变量
hittable* world = random_scene();
int ssaa_factor = 2;
int nx = 200;
int ny = 100;
camera* my_camera;
vector<vector<Eigen::Vector3f>> pixel_cache;  // 像素缓存 [行][列]


void render_thread(int thread_id,int num_threads)
{
	for (int dy = ny - 1-thread_id; dy >= 0; dy-=num_threads)
	{
		for (int dx = 0; dx < nx; dx++)
		{
			Eigen::Vector3f avg_color(0, 0, 0);
			for (int ssy = 0; ssy < ssaa_factor; ssy++)
			{
				for (int ssx = 0; ssx < ssaa_factor; ssx++)
				{
					int s_x = dx * ssaa_factor + ssx;
					int s_y = dy * ssaa_factor + ssy;

					float u = float(s_x) / float(nx * ssaa_factor);
					float v = float(s_y) / float(ny * ssaa_factor);
					ray r = my_camera->get_ray(u, v);
					Eigen::Vector3f color = getcolor(r, world, 0);
					avg_color += color;
				}
			}
			avg_color /= ssaa_factor * ssaa_factor;
			avg_color = Eigen::Vector3f(sqrt(avg_color[0]), sqrt(avg_color[1]), sqrt(avg_color[2])); //gamma矫正
			

			pixel_cache[ny - 1 - dy][dx] = avg_color;  // 注意行号转换


		}
	}
}


void raytracing() //开启多线程
{
	std::vector<thread> threads;
	pixel_cache.resize(ny, vector<Eigen::Vector3f>(nx)); //初始化
	int num_threads = thread::hardware_concurrency();
	if (num_threads == 0)num_threads = 4;

	for (int i = 0; i < num_threads; i++)
	{
		threads.push_back(thread(render_thread,i, num_threads));

	}
	for (auto& t : threads)
	{
		t.join();
	}
	// 单线程按顺序写入文件（保证像素顺序正确）
	fstream file("output.ppm", ios::out);
	if (!file.is_open())
	{
		cerr << "文件打开失败" << endl;
		return;
	}
	file << "P3\n" << nx << " " << ny << "\n255\n";
	for (int y = 0; y < ny; y++)
	{
		for (int x = 0; x < nx; x++)
		{
			Eigen::Vector3f& color = pixel_cache[y][x];
			file << int(color[0] * 255.99) << " "
				<< int(color[1] * 255.99) << " "
				<< int(color[2] * 255.99) << "\n";
		}
	}
}



int main()
{
	
	
	my_camera = new camera(Eigen::Vector3f(13, 2, 3), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0), 30, float(nx) / float(ny),0,1);
	raytracing(); //多线程渲染
	
}