// SimpleRayTracer.cpp
//
// This program renders a simple scene with a ray tracing algorithm
// Followed a tutorial on https://www.scratchapixel.com/
//

#include <iostream>
#include <vector>
#include <fstream>

#if defined __linux__ || defined __APPLE__
// LINUX
#else
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif

template<typename T>
class Vec3
{
public:
	T x, y, z;
	Vec3() : x(T(0)), y(T(0)), z(T(0)){}
	Vec3(T xx) : x(xx), y(xx), z(xx) {}
	Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}

	Vec3& normalize()
	{
		T nor2 = length2();
		if(nor2 > 0)
		{
			T invNor = 1 / sqrt(nor2);
			x *= invNor, y *= invNor, z *= invNor;
		}
		return *this;
	}

	Vec3<T> operator * (const T& f) const { return Vec3<T>(x * f, y * f, z * f); }
	Vec3<T> operator * (const Vec3<T> &f) const { return Vec3<T>(x * f.x, y * f.y, z * f.z); }
	T dot(const Vec3<T>& v) const { return x * v.x + y * v.y + z * v.z; }
	Vec3<T> operator - (const Vec3<T>& v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); }
	Vec3<T> operator + (const Vec3<T>& v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); }
	Vec3<T>& operator += (const Vec3<T>& v) { x += v.x, y += v.y, z += v.z; return *this; }
	Vec3<T>& operator *= (const Vec3<T>& v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); }
	T length2() const { return x * x + y * y + z * z; }
	T length() const { return sqrt(length2()); }
	friend std::ostream& operator << (std::ostream& os, const Vec3<T>& v)
	{
		os << "[" << v.x << " " << v.y << " " << v.z << "]";
		return os;
	}
};

typedef Vec3<float> Vec3f;

class Sphere
{
public:
	Vec3f center;
	float radius, radius_squared;
	Vec3f surfaceColor, emissionColor;
	float transparency, reflection;

	Sphere(
		const Vec3f &c,
		const float &r,
		const Vec3f& sc,
		const float& refl = 0,
		const float& t = 0,
		const Vec3f &ec = 0)
	: center(c), radius(r), radius_squared(r* r), surfaceColor(sc), emissionColor(ec), transparency(t), reflection(refl)
	{
		
	}

	bool intersect(const Vec3f &rayorigin, const Vec3f &raydirection, float &t0, float &t1) const 
	{
		Vec3f l = center - rayorigin;
		float tca = l.dot(raydirection);
		if (tca < 0) return false;
		float d2 = l.dot(l) - tca * tca;
		if (d2 > radius_squared) return false;
		float thc = sqrt(radius_squared - d2);
		t0 = tca - thc;
		t1 = tca + thc;
		return true;
	}
	
};

#define MAX_RAY_DEPTH 6

float mix(const float &a, const float &b, const float &mix)
{
	return b * mix + a * (1 - mix);
}

Vec3f trace(const Vec3f &rayorigin, const Vec3f &raydirection, const std::vector<Sphere> &spheres, const int &depth)
{
	float tnear = INFINITY;
	const Sphere* sphere = NULL;

	for(size_t i = 0; i < spheres.size(); i++)
	{
		float t0 = INFINITY, t1 = INFINITY;
		if(spheres[i].intersect(rayorigin, raydirection, t0, t1))
		{
			if (t0 < 0) t0 = t1;
			if(t0 < tnear)
			{
				tnear = t0;
				sphere = &spheres[i];
			}
		}
	}

	if (!sphere) return Vec3f(2);
	Vec3f surfaceColor = 0;
	Vec3f pHit = rayorigin + raydirection * tnear;
	Vec3f nHit = pHit - sphere->center;
	nHit.normalize();

	float bias = 1e-4;
	bool inside = false;
	if (raydirection.dot(nHit) > 0) nHit = -nHit, inside = true;
	if((sphere->transparency > 0 || sphere->reflection > 0) && depth < MAX_RAY_DEPTH)
	{
		float facingRatio = -raydirection.dot(nHit);
		float fresneleffect = mix(pow(1 - facingRatio, 3), 1, 0.1);
		Vec3f refldir = raydirection - nHit * 2 * raydirection.dot(nHit);
		refldir.normalize();
		Vec3f reflection = trace(pHit + nHit * bias, refldir, spheres, depth + 1);
		Vec3f refraction = 0;

		if(sphere->transparency)
		{
			float ior = 1.1, eta = (inside) ? ior : 1 / ior; 
			float cosi = -nHit.dot(raydirection);
			float k = 1 - eta * eta * (1 - cosi * cosi);
			Vec3f refrdir = raydirection * eta + nHit * (eta * cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(pHit - nHit * bias, refrdir, spheres, depth + 1);
		}
		surfaceColor = (reflection * fresneleffect + refraction * (1 - fresneleffect) * sphere->transparency) * sphere->surfaceColor;
	}
	else
	{
		for(size_t i = 0; i < spheres.size(); i++)
		{
			if(spheres[i].emissionColor.x > 0)
			{
				Vec3f transmission = 1;
				Vec3f lightDirection = spheres[i].center - pHit;
				lightDirection.normalize();
				for(size_t j = 0; j < spheres.size(); j++)
				{
					if(i != j)
					{
						float t0, t1;
						if(spheres[j].intersect(pHit + nHit * bias, lightDirection, t0,t1))
						{
							transmission = 0;
							break;
						}
					}
				}
				surfaceColor += sphere->surfaceColor * transmission * std::max(float(0), nHit.dot(lightDirection)) * spheres[i].emissionColor;
			}
		}
	}

	return surfaceColor + sphere->emissionColor;
}

void Render(const std::vector<Sphere>& spheres)
{
	unsigned width = 640, height = 480;
	Vec3f* image = new Vec3f[width * height], * pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 30, aspectratio = width / float(height);
	float angle = tan(M_PI * 0.5 * fov / 180.);
	// Trace rays
	for (unsigned y = 0; y < height; ++y) {
		for (unsigned x = 0; x < width; ++x, ++pixel) {
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
			Vec3f raydir(xx, yy, -1);
			raydir.normalize();
			*pixel = trace(Vec3f(0), raydir, spheres, 0);
		}
	}
	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::ofstream ofs("./untitled.ppm", std::ios::out | std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	for (unsigned i = 0; i < width * height; ++i) {
		ofs << (unsigned char)(std::min(float(1), image[i].x) * 255) <<
			(unsigned char)(std::min(float(1), image[i].y) * 255) <<
			(unsigned char)(std::min(float(1), image[i].z) * 255);
	}
	ofs.close();
	delete[] image;
}

int main()
{
	//srand48(13);
	std::vector<Sphere> spheres;
	// position, radius, surface color, reflectivity, transparency, emission color
	spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
	spheres.push_back(Sphere(Vec3f(0.0, 0, -20), 4, Vec3f(1.00, 0.32, 0.36), 1, 0.5));
	spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
	spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));
	spheres.push_back(Sphere(Vec3f(-5.5, 0, -15), 3, Vec3f(0.90, 0.90, 0.90), 1, 0.0));
	// light
	spheres.push_back(Sphere(Vec3f(0.0, 20, -30), 3, Vec3f(0.00, 0.00, 0.00), 0, 0.0, Vec3f(3)));
	Render(spheres);

	return 0;
}

