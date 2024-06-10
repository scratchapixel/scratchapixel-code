// (c) www.scratchapixel.com - 2024.
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
//  clang++ -std=c++23 -O3 -o area-light.exe area-light.cc

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <cstdint>
#include <memory>
#include <limits>
#include <xmmintrin.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <random>

template<typename T>
class Vec2 {
public:
	Vec2() : x(0), y(0) {}
	Vec2(T xx) : x(xx), y(xx) {}
	Vec2(T xx, T yy) : x(xx), y(yy) {}
	T x, y;
};

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
	Vec3& operator+=(const Vec3& v) noexcept {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	Vec3& operator/=(T a) noexcept {
		x /= a;
		y /= a;
		z /= a;
		return *this;
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
	Vec3 Normalized() const noexcept {
		T len = Length();

		if (len == 0) [[unlikely]] return Vec3(0);

		return Vec3(x / len, y / len, z / len);
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

template<typename T>
class Matrix44 {
public:
	Matrix44(T a, T b, T c, T d,
			 T e, T f, T g, T h,
			 T i, T j, T k, T l,
			 T m, T n, T o, T p) {
		x[0][0] = a;
		x[0][1] = b;
		x[0][2] = c;
		x[0][3] = d;

		x[1][0] = e;
		x[1][1] = f;
		x[1][2] = g;
		x[1][3] = h;

		x[2][0] = i;
		x[2][1] = j;
		x[2][2] = k;
		x[2][3] = l;

		x[3][0] = m;
		x[3][1] = n;
		x[3][2] = o;
		x[3][3] = p;	   
	}
	Matrix44() {
		std::memset(x[0], 0x0, sizeof(T) * 16);
		x[0][0] = x[1][1] = x[2][2] = x[3][3] = T(1);
	}
	template<typename S>
	const Matrix44<T>& SetAxisAngle (const Vec3<S>& axis, S angle) noexcept {
		Vec3<S> unit(axis.Normalized());
		S sine = std::sin (angle);
		S cosine = std::cos (angle);

		x[0][0] = unit.x * unit.x * (1 - cosine) + cosine;
		x[0][1] = unit.x * unit.y * (1 - cosine) + unit.z * sine;
		x[0][2] = unit.x * unit.z * (1 - cosine) - unit.y * sine;
		x[0][3] = 0;

		x[1][0] = unit.x * unit.y * (1 - cosine) - unit.z * sine;
		x[1][1] = unit.y * unit.y * (1 - cosine) + cosine;
		x[1][2] = unit.y * unit.z * (1 - cosine) + unit.x * sine;
		x[1][3] = 0;

		x[2][0] = unit.x * unit.z * (1 - cosine) + unit.y * sine;
		x[2][1] = unit.y * unit.z * (1 - cosine) - unit.x * sine;
		x[2][2] = unit.z * unit.z * (1 - cosine) + cosine;
		x[2][3] = 0;

		x[3][0] = 0;
		x[3][1] = 0;
		x[3][2] = 0;
		x[3][3] = 1;

		return *this;
	}
	template <class S>
	void MultDirMatrix(const Vec3<S>& src, Vec3<S>& dst) const noexcept {
		S a, b, c;

		a = src.x * x[0][0] + src.y * x[1][0] + src.z * x[2][0];
		b = src.x * x[0][1] + src.y * x[1][1] + src.z * x[2][1];
		c = src.x * x[0][2] + src.y * x[1][2] + src.z * x[2][2];

		dst.x = a;
		dst.y = b;
		dst.z = c;
	}
	template <class S>
	void MultVecMatrix(const Vec3<S>& src, Vec3<S>& dst) const noexcept {
		S a, b, c, w;

		a = src.x * x[0][0] + src.y * x[1][0] + src.z * x[2][0] + x[3][0];
		b = src.x * x[0][1] + src.y * x[1][1] + src.z * x[2][1] + x[3][1];
		c = src.x * x[0][2] + src.y * x[1][2] + src.z * x[2][2] + x[3][2];
		w = src.x * x[0][3] + src.y * x[1][3] + src.z * x[2][3] + x[3][3];

		dst.x = a / w;
		dst.y = b / w;
		dst.z = c / w;
	}
	T x[4][4];
};

struct Hit {
	double u;
	double v;
	int id0{-1};
	int id1{-1};
	double t{std::numeric_limits<double>::max()};
	operator bool() const { return id0 != -1; }
};

struct DifferentialGeometry : Hit {
	Vec3<double> P;
	Vec3<double> Ng;
	Vec3<double> Ns;
};

struct Ray {
	Vec3<double> orig;
	Vec3<double> dir;
	double near{0.1};
	double far{std::numeric_limits<double>::max()};
};

class TriangleMesh {
public:
	struct Triangle {
		uint32_t v0, v1, v2;
	};

	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) {
		const Triangle& tri = triangles_[dg.id1];
		Vec3<double> p0 = position_[tri.v0];
		Vec3<double> p1 = position_[tri.v1];
		Vec3<double> p2 = position_[tri.v2];

		double u = dg.u, v = dg.v, w = 1.f - u - v, t = dg.t;

		const Vec3<double> dPdu = p1 - p0, dPdv = p2 - p0;
		dg.P = ray.orig + t * ray.dir;
		dg.Ng = dPdv.Cross(dPdu).Normalize();
	
		if (normals_.size()) {
			const Vec3<double> n0 = normals_[tri.v0], n1 = normals_[tri.v1], n2 = normals_[tri.v2];
			Vec3<double> Ns = w * n0 + u * n1 + v * n2;
			double len2 = Ns.Dot(Ns);
			Ns = len2 > 0 ? Ns / std::sqrt(len2) : dg.Ng;
			if (Ns.Dot(dg.Ng) < 0) Ns = -Ns;
			dg.Ns = Ns;
		}
		else 
			dg.Ns = dg.Ng;
	}

	std::vector<Vec3<double>> position_;
	std::vector<Vec3<double>> normals_;
	std::vector<Triangle> triangles_;
	Vec3<double> emission_{0};
};

class Sphere : public TriangleMesh {
public:
	Sphere(Vec3<double> center, double radius) 
		: center_(center)
		, radius_(radius) {
		Triangulate();
	}
private:
	Vec3<double> SphericalToCartesian(double theta, double phi) {
		return Vec3<double>(
			std::sin(theta) * std::cos(phi),
			std::cos(theta), 
			std::sin(theta) * std::sin(phi));
	}
	void Triangulate() {
		for (uint32_t theta = 0; theta <= num_theta_; ++theta) {
			for (uint32_t phi = 0; phi < num_phi_; ++phi) {
				Vec3<double> p = SphericalToCartesian(theta * M_PI / num_theta_, phi * 2 * M_PI / num_phi_);
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
	uint32_t num_theta_{20};
	uint32_t num_phi_{20};
	Vec3<double> center_;
	double radius_;
};

constexpr double DegToRadians(const double& deg) {
	return M_PI / 180.f * deg;
}

class Light {
public:
	virtual ~Light() = default;
	virtual Vec3<double> Sample(const DifferentialGeometry&, const Vec2<double>&, Vec3<double>&, double&, double&) const = 0;
	double intensity_{96};
	Vec3<double> color_{1,1,1};	
};

class AreaLight : public Light {
public:
	AreaLight(Matrix44<double> m) {
		Vec3<double> v1, v2;
		m.MultVecMatrix(Vec3<double>(-1, 1,0), v0_);
		m.MultVecMatrix(Vec3<double>( 1, 1,0), v1);
		m.MultVecMatrix(Vec3<double>(-1,-1,0), v2);
		e0_ = v1 - v0_;
		e1_ = v2 - v0_;
		pdf_ = 1 / e0_.Length() * e1_.Length();
	}
	Vec3<double> Sample(const DifferentialGeometry& dg, const Vec2<double>& r, Vec3<double>& wi, double& pdf, double &t_max) const {
		Vec3<double> sample = v0_ + e0_ * r.x + e1_ * r.y;
		wi = sample - dg.P;
		t_max = wi.Length();
		wi /= t_max;
		pdf = pdf_ * t_max * t_max;
		return Vec3<double>(std::powf(2, 27));
	}
	double pdf_{0};
	Vec3<double> v0_, e0_, e1_;
};

/**
 * Extracts the sign bit of a double, returning -0.0 for negative and 0.0 for
 * positive or zero. Uses SIMD operations for efficiency. _mm_set_ss sets
 * double x in a 128-bit vector, while _mm_set1_epi32(0x80000000) creates a
 * mask to isolate the sign bit. _mm_and_ps applies the mask, and _mm_cvtss_f32
 * converts the result back to a double.
 */
__forceinline double signmsk(const double x) {
	return _mm_cvtss_f32(_mm_and_ps(_mm_set_ss(x),_mm_castsi128_ps(_mm_set1_epi32(0x80000000))));
}


/**
 * xorf performs a bitwise XOR on double x and y, returning a double.
 * - If x and y are both positive or both negative: No sign change in x.
 * - If x and y have different signs: The sign of x is "inverted".
 * This operation can flip the sign of x or leave it unchanged,
 * depending on y's sign.
 */
__forceinline double xorf(const double x, const double y) { 
	return _mm_cvtss_f32(_mm_xor_ps(_mm_set_ss(x),_mm_set_ss(y)));
}

void Intersect(const Ray& ray, 
			   Hit& hit, 
			   int obj_id, 
			   int tri_id, 
			   const TriangleMesh::Triangle& tri, 
			   const Vec3<double>* verts) {
	const Vec3<double> p0 = verts[tri.v0];
	const Vec3<double> p1 = verts[tri.v1];
	const Vec3<double> p2 = verts[tri.v2];
	const Vec3<double> e1 = p0 - p1;
	const Vec3<double> e2 = p2 - p0;
	const Vec3<double> Ng = e1.Cross(e2);

	const Vec3<double> C = p0 - ray.orig;
	const Vec3<double> R = ray.dir.Cross(C);
	const double det = Ng.Dot(ray.dir);
	const double abs_det = std::abs(det);
	const double sng_det = signmsk(det);
	if (det == 0) [[unlikely]] return;
	
	const double U = xorf(R.Dot(e2), sng_det);
	if (U < 0) [[likely]] return;
	
	const double V = xorf(R.Dot(e1), sng_det);
	if (V < 0) [[likely]] return;
	
	const double W = abs_det - U - V;
	if (W < 0) [[likely]] return;
	
	const double T = xorf(Ng.Dot(C), sng_det);
	if (T < abs_det * ray.near || abs_det * hit.t < T) [[unlikely]] return;

	hit.u = U / abs_det;
	hit.v = V / abs_det;
	hit.t = T / abs_det;
	hit.id0 = obj_id;
	hit.id1 = tri_id;
}

bool Occluded(const Ray& ray, 
			  const TriangleMesh::Triangle& tri, 
			  const Vec3<double>* verts) {
	const Vec3<double> p0 = verts[tri.v0];
	const Vec3<double> p1 = verts[tri.v1];
	const Vec3<double> p2 = verts[tri.v2];
	const Vec3<double> e1 = p0 - p1;
	const Vec3<double> e2 = p2 - p0;
	const Vec3<double> Ng = e1.Cross(e2);

	const Vec3<double> C = p0 - ray.orig;
	const Vec3<double> R = ray.dir.Cross(C);
	const double det = Ng.Dot(ray.dir);
	const double abs_det = abs(det);
	const double sgn_det = signmsk(det);
	if (det == 0.f) [[unlikely]] return false;

	const double U = xorf(R.Dot(e2),sgn_det);
	if (U < 0.f) [[likely]] return false;

	const double V = xorf(R.Dot(e1), sgn_det);
	if (V < 0.f) [[likely]] return false;

	const double W = abs_det - U - V;
	if (W < 0.f) [[likely]] return false;

	const double T = xorf(Ng.Dot(C), sgn_det);
	if (T < abs_det * ray.near || abs_det * ray.far < T) [[unlikely]] return false;

	return true;
}
			  
bool Occluded(const Ray& ray, const std::vector<std::unique_ptr<TriangleMesh>>& prims) {
	for (int i = 0; i < (int)prims.size(); ++i) {
		const std::vector<Vec3<double>>& pos = prims[i]->position_;
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
constexpr double angle_of_view = 60.f;

void GenerateCoordinateSystem(const Vec3<double>& normal, Vec3<double>& tangent, Vec3<double>& bitangent) {
	if (std::abs(normal.x) > std::abs(normal.y))
        tangent = Vec3<double>(normal.z, 0, -normal.x) / std::sqrt(normal.x * normal.x + normal.z * normal.z);
    else
        tangent = Vec3<double>(0, -normal.z, normal.y) / std::sqrt(normal.y * normal.y + normal.z * normal.z);
    bitangent = normal.Cross(tangent);
}

Vec3<double> OrientWithNormal(
	const Vec3<double>& sample_dir, 
	const Vec3<double>& normal, 
	const Vec3<double>& tangent, 
	const Vec3<double>& bitangent) {
    return {
        sample_dir.x * tangent.x + sample_dir.y * normal.x + sample_dir.z * bitangent.x,
        sample_dir.x * tangent.y + sample_dir.y * normal.y + sample_dir.z * bitangent.y,
        sample_dir.x * tangent.z + sample_dir.y * normal.z + sample_dir.z * bitangent.z
    };
}

int main() {
	
	// Building the scene
	std::vector<std::unique_ptr<TriangleMesh>> prims;
	prims.push_back(std::make_unique<Sphere>(Vec3<double>(0,5100,0), 2280));

	std::vector<Vec3<double>> verts = {
		// ground
		{-6805, 0, 6805}, 
		{ 6805, 0, 6805}, 
		{ 6805, 0,-6805},
		{-6805, 0,-6805},
		// mesh area light
		{ 3575, 8390, 3575},
		{-3575, 8390, 3575},
		{-3575, 8390,-3575},
		{ 3575, 8390,-3575}
	};

	std::vector<Vec3<double>> nors = {{0,1,0},{0,-1,0}};
	
	Matrix44<double> m(3575, 0, 0, 0, 0, 0, -3575, 0, 0, 3575, 0, 0, 0, 8390, 0, 1);
	AreaLight rectangular_light(m);

	for (uint32_t i = 0; i < 2; ++i) {
		TriangleMesh* mesh = new TriangleMesh;
		for (uint32_t j = 0; j < 4; ++j) {
			mesh->position_.push_back(verts[i * 4 + j]);
			mesh->normals_.push_back(nors[i]);
		}
		mesh->triangles_.push_back({2,1,0});
		mesh->triangles_.push_back({3,2,0});
		mesh->emission_ = Vec3<double>(i * std::pow(2, 27));
		prims.push_back(std::unique_ptr<TriangleMesh>(mesh));
	}

	double scale = std::tan(angle_of_view * 0.5 * M_PI / 180.f);
	double aspect_ratio = width / static_cast<double>(height);
	std::unique_ptr<uint8_t[]> buf = std::make_unique<uint8_t[]>(width * height * 3);
	uint8_t* pbuf = buf.get();
	std::memset(pbuf, 0x0, width * height * 3);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dist(0,1);

	const uint32_t num_samples = 1;

	for (uint32_t y = 0; y < height; ++y) {
		for (uint32_t x = 0; x < width; ++x, pbuf += 3) {
			double px = (2.f * (x + 0.5) / static_cast<double>(width) - 1.f) * scale;
			double py = (1.f - 2.0f * (y + 0.5) / static_cast<double>(height)) * scale / aspect_ratio;
			Vec3<double> dir(px, py, -1);
			dir.Normalize();
			Ray ray = {Vec3<double>(0,3000,23500), dir};
			DifferentialGeometry dg;
			for (int i = 0; i < (int)prims.size(); ++i) {
				const std::vector<Vec3<double>>& pos = prims[i]->position_;
				const std::vector<TriangleMesh::Triangle>& tris = prims[i]->triangles_;
				for (size_t j = 0; j < tris.size(); ++j) {
					Intersect(ray, dg, i, j, tris[j], pos.data());  
				}			
			}
#ifdef SOLIDANGLE_INT
			if (dg) {
				prims[dg.id0]->PostIntersect(ray, dg);
				Vec3<double> tangent, bitangent;
				GenerateCoordinateSystem(dg.Ns, tangent, bitangent);
				Vec3<double> L = 0;
				for (uint32_t n = 0; n < num_samples; ++n) {
					double cos_theta = dist(gen);
					double sin_theta = std::sqrt(1 - cos_theta * cos_theta);
					double phi = 2 * M_PI * dist(gen);

					double x = sin_theta * std::cos(phi);
					double y = cos_theta;
					double z = sin_theta * std::sin(phi);
					
					Vec3<double> light_ray_dir = OrientWithNormal(Vec3<double>(x, y, z), dg.Ns, tangent, bitangent);
					light_ray_dir.Normalize();
					Ray light_ray(dg.P + dg.Ns * 2, light_ray_dir);
					DifferentialGeometry dgl;
					for (int i = 0; i < (int)prims.size(); ++i) {
						const std::vector<Vec3<double>>& pos = prims[i]->position_;
						const std::vector<TriangleMesh::Triangle>& tris = prims[i]->triangles_;
						for (size_t j = 0; j < tris.size(); ++j) {
							Intersect(light_ray, dgl, i, j, tris[j], pos.data());  
						}			
					}
					if (dgl) {
						prims[dgl.id0]->PostIntersect(light_ray, dgl);
						if (prims[dgl.id0]->emission_.x > 0) {
							// The PDF value is 1/(2*pi) and the diffuse BRDF value is 1/pi. When these values are combined,
							// they simplify to (2*pi)/pi. The pi terms cancel each other out, leaving a factor of 2.
							// We then multiply this by 2 * Vec3<double>(2.62541) * cos_theta to compute the final value.
							// The code below is intentionally left unsimplified to help readers fully grasp the equation
							// used in this context.
							float pdf = 1 / 2 * M_PI;
							L += (1 / M_PI) * Vec3<double>(2.62541) * cos_theta / pdf;
						}
					}
				}
				L /= num_samples;
				pbuf[0] = static_cast<uint8_t>(std::clamp(prims[dg.id0]->emission_.x + L.x, 0., 1.) * 255);
				pbuf[1] = static_cast<uint8_t>(std::clamp(prims[dg.id0]->emission_.y + L.y, 0., 1.) * 255);
				pbuf[2] = static_cast<uint8_t>(std::clamp(prims[dg.id0]->emission_.z + L.z, 0., 1.) * 255);
			}
#else
			if (dg) {
				/*
				std::vector<Vec2<double>> samples(8 * 8);
				for (uint32_t j = 0; j < 8; ++j) {
					for (uint32_t i = 0; i < 8; ++i) {
						samples[j * 8 + i] = {(i + 0.5) / 8, (j + 0.5) / 8};
					}	
				}
				*/
				const int N = 8; // Grid size
				std::vector<double> intervals(N);
				std::vector<std::pair<double, double>> samples(N * N);
				//std::mt19937 gen(std::random_device{}()); // Random number generator

				// Initialize intervals for both dimensions
				for (int i = 0; i < N; ++i) {
					intervals[i] = i / static_cast<double>(N);
				}

				// Generate Latin Hypercube Samples
				for (int dim = 0; dim < 2; ++dim) { // Loop over each dimension
					std::shuffle(intervals.begin(), intervals.end(), gen);
					//shuffle_vector(intervals, gen); // Shuffle intervals for the current dimension
					
					for (int i = 0; i < N; ++i) {
						// Assign shuffled intervals to samples
						// Each dimension is handled separately to maintain independence
						if (dim == 0) {
							for (int j = 0; j < N; ++j) {
								samples[i * N + j].first = dist(gen);//intervals[i] + (1.0 / N) * (gen() % N) / static_cast<double>(N);
							}
						} else {
							for (int j = 0; j < N; ++j) {
								samples[j * N + i].second = dist(gen);//intervals[i] + (1.0 / N) * (gen() % N) / static_cast<double>(N);
							}
						}
					}
				}


				Vec3<double> L = 0;
				if (dg.id0 != 2) { // not the area light
					prims[dg.id0]->PostIntersect(ray, dg);
					double pdf, t_max;
					for (uint32_t n = 0; n < num_samples; ++n) {
						Vec3<double> wi;
						Vec2<double> sample = {0.5, 0.5};//samples[n].first, samples[n].second};//samples[n];//{(n + 0.5 * dist(gen)) / num_samples, (n + 0.5 * dist(gen)) / num_samples};
						Vec3<double> Le = rectangular_light.Sample(dg, sample, wi, pdf, t_max);
						std::cerr << dg.P + wi * t_max << std::endl; abort();
						//bool in_shadow = Occluded({dg.P + dg.Ns * 4, wi, 0.01f, t_max - 10}, prims);
						//if (in_shadow)
						//	continue;
						L += 1 / M_PI * Le * std::max(0., dg.Ns.Dot(wi)) * std::max(0., -wi.Dot(Vec3<double>(0,-1,0))) / pdf;
					}
					L /= num_samples;
				}
				pbuf[0] = static_cast<uint8_t>(std::clamp(prims[dg.id0]->emission_.x + L.x, 0., 1.) * 255);
				pbuf[1] = static_cast<uint8_t>(std::clamp(prims[dg.id0]->emission_.y + L.y, 0., 1.) * 255);
				pbuf[2] = static_cast<uint8_t>(std::clamp(prims[dg.id0]->emission_.z + L.z, 0., 1.) * 255);
			}
#endif
			std::cerr << "\r" << (int)((100.f * y) / height) << "%";
		}
	}
	
	std::ofstream ofs("./test.ppm", std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	ofs.write(reinterpret_cast<char*>(buf.get()), width * height * 3);
	ofs.close();

	return 0;
};