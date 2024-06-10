// (c) www.scratchapixel.com - 2024.
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -Wall -Wextra -std=c++23 -o light.exe light.cc -O3

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <cstdint>
#include <memory>
#include <limits>
#include <xmmintrin.h>
#include <iostream>
#include <fstream>

template<typename T>
class Vec3 {
public:
	Vec3() : x(0), y(0), z(0) {}
	Vec3(T xx) : x(xx), y(xx), z(xx) {}
	Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
	Vec3 operator+(const Vec3& v) const noexcept {
		return {x + v.x, y + v.y, z + v.z};
	}
	Vec3 operator-(const Vec3& v) const noexcept {
		return {x - v.x, y - v.y, z - v.z};
	}
	Vec3 operator*(const T& real) const noexcept {
		return {x * real, y * real, z * real};
	}
	friend Vec3 operator*(const T& real, const Vec3& v) noexcept {
		return {real * v.x, real * v.y, real * v.z};
	}
	Vec3 operator/(const T& real) const noexcept {
		return {x / real, y / real, z / real};
	}
	Vec3 operator-() const noexcept {
		return {-x, -y, -z};
	}
	T Length() const {
		return std::sqrtf(x * x + y * y + z * z);
	}
	Vec3& Normalize() noexcept {
		T len = Length();
		if (len != 0) [[likely]] {
			x /= len;
			y /= len;
			z /= len;
		}
		return *this;
	}
	T Dot(const Vec3& v) const noexcept {
		return x * v.x + y * v.y + z * v.z;
	}
	Vec3 Cross(const Vec3& v) const noexcept {
		return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
	}
	friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
		return os << v.x << " " << v.y << " " << v.z;
	}
	T x, y, z;
};

struct Hit {
	float u;
	float v;
	int id0{-1};
	int id1{-1};
	float t{std::numeric_limits<float>::max()};
	operator bool() const { return id0 != -1; }
};

struct DifferentialGeometry : Hit {
	Vec3<float> P;
	Vec3<float> Ng;
	Vec3<float> Ns;
};

struct Ray {
	Vec3<float> orig;
	Vec3<float> dir;
	float near{0.1};
	float far{std::numeric_limits<float>::max()};
};

class TriangleMesh {
public:
	struct Triangle {
		uint32_t v0, v1, v2;
	};

	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) {
		const Triangle& tri = triangles_[dg.id1];
		Vec3<float> p0 = position_[tri.v0];
		Vec3<float> p1 = position_[tri.v1];
		Vec3<float> p2 = position_[tri.v2];

		float u = dg.u, v = dg.v, w = 1.f - u - v, t = dg.t;

		const Vec3<float> dPdu = p1 - p0, dPdv = p2 - p0;
		dg.P = ray.orig + t * ray.dir;
		dg.Ng = dPdv.Cross(dPdu).Normalize();
	
		if (normals_.size()) {
			const Vec3<float> n0 = normals_[tri.v0], n1 = normals_[tri.v1], n2 = normals_[tri.v2];
			Vec3<float> Ns = w * n0 + u * n1 + v * n2;
			float len2 = Ns.Dot(Ns);
			Ns = len2 > 0 ? Ns / std::sqrt(len2) : dg.Ng;
			if (Ns.Dot(dg.Ng) < 0) Ns = -Ns;
			dg.Ns = Ns;
		}
		else 
			dg.Ns = dg.Ng;
	}

	std::vector<Vec3<float>> position_;
	std::vector<Vec3<float>> normals_;
	std::vector<Triangle> triangles_;
};

class Sphere : public TriangleMesh {
public:
	Sphere() : center_(Vec3<float>(0,-2,-22)), radius_(2) {
		Triangulate();
	}
private:
	Vec3<float> SphericalToCartesian(float theta, float phi) {
		return Vec3<float>(
			std::sin(theta) * std::cos(phi),
			std::cos(theta), 
			std::sin(theta) * std::sin(phi));
	}
	void Triangulate() {
		for (uint32_t theta = 0; theta <= num_theta_; ++theta) {
			for (uint32_t phi = 0; phi < num_phi_; ++phi) {
				Vec3<float> p = SphericalToCartesian(theta * M_PI / num_theta_, phi * 2 * M_PI / num_phi_);
				normals_.push_back(p);
				position_.push_back(center_ + p * radius_);
			}
			if (theta == 0) continue;
			for (uint32_t phi = 1; phi <= num_phi_; ++phi) {
				uint32_t p00 = (theta - 1) * num_phi_ + phi - 1;
				uint32_t p01 = (theta - 1) * num_phi_ + phi % num_phi_;
				uint32_t p10 = theta * num_phi_ + phi - 1;
				uint32_t p11 = theta * num_phi_ + phi % num_phi_;
				if (theta > 1) triangles_.push_back({p10, p01, p00});
				if (theta < num_theta_) triangles_.push_back({p11, p01, p10});
			}
		}
	}
public:
	uint32_t num_theta_{16};
	uint32_t num_phi_{16};
	Vec3<float> center_;
	float radius_;
};

class Light {
public:
	Vec3<float> Sample(const DifferentialGeometry& dg, Vec3<float>& wi, float& pdf, float &t_max) const {
		Vec3<float> d = pos_ - dg.P;
		float distance = d.Length();
		wi = d / distance;
		pdf = distance * distance;
		t_max = distance;
		return color_;
	}
	Vec3<float> pos_{0,8,-22};
	Vec3<float> color_{1,1,1};	
};

/**
 * Extracts the sign bit of a float, returning -0.0 for negative and 0.0 for
 * positive or zero. Uses SIMD operations for efficiency. _mm_set_ss sets
 * float x in a 128-bit vector, while _mm_set1_epi32(0x80000000) creates a
 * mask to isolate the sign bit. _mm_and_ps applies the mask, and _mm_cvtss_f32
 * converts the result back to a float.
 */
__forceinline float signmsk(const float x) {
	return _mm_cvtss_f32(_mm_and_ps(_mm_set_ss(x),_mm_castsi128_ps(_mm_set1_epi32(0x80000000))));
}


/**
 * xorf performs a bitwise XOR on float x and y, returning a float.
 * - If x and y are both positive or both negative: No sign change in x.
 * - If x and y have different signs: The sign of x is "inverted".
 * This operation can flip the sign of x or leave it unchanged,
 * depending on y's sign.
 */
__forceinline float xorf(const float x, const float y) { 
	return _mm_cvtss_f32(_mm_xor_ps(_mm_set_ss(x),_mm_set_ss(y)));
}

void Intersect(const Ray& ray, 
			   Hit& hit, 
			   int obj_id, 
			   int tri_id, 
			   const TriangleMesh::Triangle& tri, 
			   const Vec3<float>* verts) {
	const Vec3<float> p0 = verts[tri.v0];
	const Vec3<float> p1 = verts[tri.v1];
	const Vec3<float> p2 = verts[tri.v2];
	const Vec3<float> e1 = p0 - p1;
	const Vec3<float> e2 = p2 - p0;
	const Vec3<float> Ng = e1.Cross(e2);

	const Vec3<float> C = p0 - ray.orig;
	const Vec3<float> R = ray.dir.Cross(C);
	const float det = Ng.Dot(ray.dir);
	const float abs_det = std::abs(det);
	const float sng_det = signmsk(det);
	if (det == 0) [[unlikely]] return;
	
	const float U = xorf(R.Dot(e2), sng_det);
	if (U < 0) [[likely]] return;
	
	const float V = xorf(R.Dot(e1), sng_det);
	if (V < 0) [[likely]] return;
	
	const float W = abs_det - U - V;
	if (W < 0) [[likely]] return;
	
	const float T = xorf(Ng.Dot(C), sng_det);
	if (T < abs_det * ray.near || abs_det * hit.t < T) [[unlikely]] return;

	hit.u = U / abs_det;
	hit.v = V / abs_det;
	hit.t = T / abs_det;
	hit.id0 = obj_id;
	hit.id1 = tri_id;
}

bool Occluded(const Ray& ray, 
			  const TriangleMesh::Triangle& tri, 
			  const Vec3<float>* verts) {
	const Vec3<float> p0 = verts[tri.v0];
	const Vec3<float> p1 = verts[tri.v1];
	const Vec3<float> p2 = verts[tri.v2];
	const Vec3<float> e1 = p0 - p1;
	const Vec3<float> e2 = p2 - p0;
	const Vec3<float> Ng = e1.Cross(e2);

	const Vec3<float> C = p0 - ray.orig;
	const Vec3<float> R = ray.dir.Cross(C);
	const float det = Ng.Dot(ray.dir);
	const float abs_det = abs(det);
	const float sgn_det = signmsk(det);
	if (det == 0.f) [[unlikely]] return false;

	const float U = xorf(R.Dot(e2),sgn_det);
	if (U < 0.f) [[likely]] return false;

	const float V = xorf(R.Dot(e1), sgn_det);
	if (V < 0.f) [[likely]] return false;

	const float W = abs_det - U - V;
	if (W < 0.f) [[likely]] return false;

	const float T = xorf(Ng.Dot(C), sgn_det);
	if (T < abs_det * ray.near || abs_det * ray.far < T) [[unlikely]] return false;

	return true;
}
			  
bool Occluded(const Ray& ray, const std::vector<std::unique_ptr<TriangleMesh>>& prims) {
	for (int i = 0; i < (int)prims.size(); ++i) {
		const std::vector<Vec3<float>>& pos = prims[i]->position_;
		const std::vector<TriangleMesh::Triangle>& tris = prims[i]->triangles_;
		for (size_t j = 0; j < tris.size(); ++j) {
			if (Occluded(ray, tris[j], pos.data()))
				return true;
		}			
	}
	return false;
}

constexpr uint32_t width = 960;
constexpr uint32_t height = 540;
constexpr float angle_of_view = 60.f;

int main() {
	std::vector<std::unique_ptr<TriangleMesh>> prims;
	prims.push_back(std::make_unique<Sphere>());
	Light light;
	
	std::vector<Vec3<float>> verts = {
		{-5,-4,-17}, {5,-4,-17}, {5,-4,-27}, {-5,-4,-27},
		{5,-4,-27}, {5,6,-27}, {-5,6,-27}, {-5,-4,-27}};
	std::vector<Vec3<float>> nors = {{0,1,0},{0,0,1}};
	
	for (uint32_t i = 0; i < 2; ++i) {
		TriangleMesh* mesh = new TriangleMesh;
		for (uint32_t j = 0; j < 4; ++j) {
			mesh->position_.push_back(verts[i * 4 + j]);
			mesh->normals_.push_back(nors[i]);
		}
		mesh->triangles_.push_back({2,1,0});
		mesh->triangles_.push_back({3,2,0});
 
		prims.push_back(std::unique_ptr<TriangleMesh>(mesh));
	}

	float scale = std::tan(angle_of_view * 0.5 * M_PI / 180.f);
	float aspect_ratio = width / static_cast<float>(height);
	std::unique_ptr<uint8_t[]> buf = std::make_unique<uint8_t[]>(width * height * 3);
	uint8_t* pbuf = buf.get();
	std::memset(pbuf, 0x0, width * height * 3);

	for (uint32_t y = 0; y < height; ++y) {
		for (uint32_t x = 0; x < width; ++x, pbuf += 3) {
			float px = (2.f * (x + 0.5) / static_cast<float>(width) - 1.f) * scale;
			float py = (1.f - 2.0f * (y + 0.5) / static_cast<float>(height)) * scale / aspect_ratio;
			Vec3<float> dir(px, py, -1);
			dir.Normalize();
			Ray ray = {Vec3<float>(0), dir};
			DifferentialGeometry dg;
			for (int i = 0; i < (int)prims.size(); ++i) {
				const std::vector<Vec3<float>>& pos = prims[i]->position_;
				const std::vector<TriangleMesh::Triangle>& tris = prims[i]->triangles_;
				for (size_t j = 0; j < tris.size(); ++j) {
					Intersect(ray, dg, i, j, tris[j], pos.data());  
				}			
			}
			if (dg) {
				prims[dg.id0]->PostIntersect(ray, dg);
				Vec3<float> wi;
				float t_max, pdf;
				Vec3<float> light_L = light.Sample(dg, wi, pdf, t_max);
				bool in_shadow = Occluded({dg.P, wi, 0.01f, t_max - 0.01f}, prims);
				if (in_shadow)
					continue;
				Vec3<float> L = std::pow(2.f, 7) * light_L * std::max(0.f, dg.Ns.Dot(wi)) / (M_PI * pdf);
				pbuf[0] = static_cast<uint8_t>(std::min(1.f, L.x) * 255);
				pbuf[1] = static_cast<uint8_t>(std::min(1.f, L.y) * 255);
				pbuf[2] = static_cast<uint8_t>(std::min(1.f, L.z) * 255);
			}
		}
	}

	std::ofstream ofs("./test.ppm", std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	ofs.write(reinterpret_cast<char*>(buf.get()), width * height * 3);
	ofs.close();

	return 0;
};