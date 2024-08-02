// (c) www.scratchapixel.com - 2024.
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -Wall -Wextra -std=c++23 -o area-light-impl.exe area-light-impl.cc -O3

#define _USE_MATH_DEFINES
#include <iostream>
#include <limits>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <chrono>
#include "math.h"

using Vec2f = Vec2<float>;
using Vec3f = Vec3<float>;
using Matrix44f = Matrix44<float>;

struct Ray {
	Vec3f orig;
	Vec3f dir;
	float near{0};
	float far{std::numeric_limits<float>::infinity()};
};

struct Hit {
	int id0{-1};
	int id1{-1};
	float u;
	float v;
	float t{std::numeric_limits<float>::infinity()};
	operator bool() const { return id0 != -1; }
};

class Camera {
public:
	void SetRay(const Vec2f& pixel, Ray& ray) const {
		ray.orig = Vec3f(cam_to_world_[3][0], cam_to_world_[3][1], cam_to_world_[3][2]);
		float angle = std::tan(60 * 0.5 * M_PI / 180.f);
		float aspect_ratio = 780.f / 585;
		Vec3f dir((2 * pixel.x - 1) * angle, (1 - 2 * pixel.y) * angle / aspect_ratio, -1);
		cam_to_world_.MultDirMatrix(dir, ray.dir);
		ray.dir.Normalize();
		ray.near = 0;
		ray.far = std::numeric_limits<float>::infinity();
	}
	Matrix44<float> cam_to_world_;
};

struct BuildTriangle {
	int v0, v1, v2;
	int id0{0}, id1{0};
};

///////////////////////////////////////////////////////////////////////////////
// Shape related structs/classes

struct DifferentialGeometry : public Hit { // renderer/shapes/differentialgeometry.h
	class Material* material{nullptr};
	class AreaLight* light{nullptr};

	Vec3f P;
	Vec3f Ng;
	Vec3f Ns;
	Vec2f st;
	float error;
};

class Shape : public std::enable_shared_from_this<Shape> {
public:
	virtual ~Shape() = default;
	virtual size_t GetNumVertices() const = 0;
	virtual size_t GetNumTriangles() const = 0;
	virtual Box3f Extract(size_t id, BuildTriangle* triangles, size_t& num_triangles, Vec3f* vertices, size_t& num_vertices) const = 0;
	virtual std::shared_ptr<Shape> Transform(const Matrix44f& m) const = 0;
	virtual void PostIntersect(const Ray& ray, DifferentialGeometry& dg) const = 0;
};

class Triangle : public Shape {
public:
	Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2)
		: v0_(v0)
		, v1_(v1)
		, v2_(v2)
		, Ng_((v1 - v0).Cross(v2 - v0).Normalize()) {
	}
	std::shared_ptr<Shape> Transform(const Matrix44<float>& m) const override {
		Vec3f v0, v1, v2;
		m.MultVecMatrix(v0_, v0);
		m.MultVecMatrix(v1_, v1);
		m.MultVecMatrix(v2_, v2);
		return std::make_shared<Triangle>(v0, v1, v2);
	}
	size_t GetNumTriangles() const override { return 1; }
	size_t GetNumVertices() const override { return 3; }
	Box3f Extract(size_t id, BuildTriangle* triangles, size_t& num_triangles, Vec3f* vertices, size_t& num_vertices) const override {
		Box3f bounds;
		triangles[num_triangles++] = {(int)(num_vertices), (int)(num_vertices + 1), (int)(num_vertices + 2), (int)id};
		vertices[num_vertices++] = v0_;
		vertices[num_vertices++] = v1_;
		vertices[num_vertices++] = v2_;
		bounds.ExtendBy(v0_);
		bounds.ExtendBy(v1_);
		bounds.ExtendBy(v2_);
		return bounds;
	}

	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) const override {
		dg.P = ray.orig + dg.t * ray.dir;
		dg.Ng = Ng_;
		dg.Ns = Ng_;
		dg.st = Vec2f(dg.u, dg.v);
		dg.error = std::max(std::abs(dg.t), dg.P.Max());
	}

	Vec3f v0_;
	Vec3f v1_;
	Vec3f v2_;
	Vec3f Ng_;
};

template<>
const Matrix44<float> Matrix44<float>::kIdentity = Matrix44<float>();

class TriangleMesh : public Shape {
public:	
	struct Triangle {
		uint32_t v0;
		uint32_t v1;
		uint32_t v2;
	};
	size_t GetNumVertices() const override {
		return vertices_.size();
	}
	size_t GetNumTriangles() const override {
		return triangles_.size();
	}
	Box3f Extract(size_t id, BuildTriangle* triangles, size_t& num_triangles, Vec3f* vertices, size_t& num_vertices) const override {
		Box3f bounds;
		for (size_t i = 0; i < triangles_.size(); ++i) {
			const Triangle& tri = triangles_[i];
			triangles[num_triangles++] = {(int)(num_vertices + tri.v0), (int)(num_vertices + tri.v1), (int)(num_vertices + tri.v2), (int)id, (int)i};
		}
		for (size_t i = 0; i < vertices_.size(); ++i) {
			const Vec3f& v = vertices_[i];
			vertices[num_vertices++] = v;
			bounds.ExtendBy(v);
		}
		return bounds;
	}
	std::shared_ptr<Shape> Transform([[maybe_unused]] const Matrix44f& m) const override {
		if (m == Matrix44f::kIdentity)
			return const_cast<TriangleMesh*>(this)->shared_from_this();

		std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();

		mesh->vertices_.resize(vertices_.size());
		for (size_t i = 0; i < vertices_.size(); ++i) m.MultVecMatrix(vertices_[i], mesh->vertices_[i]); 

		mesh->normals_.resize(normals_.size());
		Matrix44f m_inverse = m.Transposed();
		for (size_t i = 0; i < normals_.size(); ++i) m_inverse.MultDirMatrix(normals_[i], mesh->normals_[i]); 
		mesh->triangles_ = triangles_;

		return mesh;
	}

	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) const override {
		const Triangle& tri = triangles_[dg.id1];
		const Vec3f& v0 = vertices_[tri.v0];
		const Vec3f& v1 = vertices_[tri.v1];
		const Vec3f& v2 = vertices_[tri.v2];

		[[maybe_unused]] const float u = dg.u, v = dg.v, w = 1 - u - v, t = dg.t;

		const Vec3f dPdu = v1 - v0;
		const Vec3f dPdv = v2 - v0;
		dg.P = ray.orig + t * ray.dir;
		dg.Ng = dPdu.Cross(dPdv).Normalize();
		dg.error = std::max(std::abs(dg.t), dg.P.Max());
	}
	std::vector<Vec3f> vertices_;
	std::vector<Vec3f> normals_;
	std::vector<Triangle> triangles_;
};

class Sphere : public TriangleMesh {
public:
	Sphere(Vec3f center, float radius) : center_(center), radius_(radius) {
		Triangulate();
	}
private:
	Vec3f Eval(float theta, float phi) const {
		return Vec3f(std::cos(phi) * std::sin(theta), std::cos(theta), std::sin(phi) * std::sin(theta));
	}
	void Triangulate() {
		float rcp_num_theta = 1.f / num_theta_;
		float rcp_num_phi = 1.f / num_phi_;

		for (uint32_t theta = 0; theta <= num_theta_; theta++) {
			for (uint32_t phi = 0; phi < num_phi_; phi++) {
			  
				Vec3f p = Eval(theta * M_PI * rcp_num_theta, phi * 2.0f * M_PI * rcp_num_phi);
				Vec3f dpdu = Eval((theta + 0.001f) * M_PI * rcp_num_theta, phi * 2.0f * M_PI * rcp_num_phi) - center_;
				Vec3f dpdv = Eval(theta * M_PI * rcp_num_theta,(phi + 0.001f) * 2.0f * M_PI * rcp_num_phi) - center_;
				p = center_ + radius_ * p;

				vertices_.push_back(p);
				normals_.push_back(dpdv.Cross(dpdu).Normalize());
			}
			if (theta == 0) continue;
			for (uint32_t phi = 1; phi <= num_phi_; phi++) {
				uint32_t p00 = (theta -1) * num_phi_ + phi - 1;
				uint32_t p01 = (theta -1) * num_phi_ + phi % num_phi_;
				uint32_t p10 = theta * num_phi_ + phi - 1;
				uint32_t p11 = theta * num_phi_ + phi % num_phi_;
				if (theta > 1) triangles_.push_back({p10, p01, p00});
				if (theta < num_theta_) triangles_.push_back({p11, p01, p10});
			}
		}
	}
	
	Vec3f center_{0};
	float radius_{1};
	uint32_t num_theta_{10};
	uint32_t num_phi_{10};
};

template<typename T> 
struct Sample {
	Sample() {}
	Sample(const T value, float pdf) : value(value), pdf(pdf) {
	}		
	// Type conversion operator overload (e.g. Vec3f(light_sample.wi))
	operator const T&() const { return value; }
	operator T&() { return value; }
	T value;
	float pdf;
};

using Sample3f = Sample<Vec3f>;

struct LightSample {
	Sample3f wi;
	float tmax;
	Vec3f L;
};

inline Vec3f UniformSampleTriangle(const float& u, const float& v, const Vec3f& A, const Vec3f& B, const Vec3f& C) {
	float su = std::sqrt(u);
	return Vec3f(C + (1.f - su) * (A - C) + (v * su) * (B - C));
}

inline Vec3f UniformSampleSphere(const float& r1, const float& r2) {
	float z = 1.f - 2.f * r1; // cos(theta)
	float sin_theta = std::sqrt(1 - z * z);
	float phi = 2 * M_PI * r2; // azimuthal angle
	return {std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, z};
}

class Light {
public:
	virtual ~Light() = default;
	virtual std::shared_ptr<Light> Transform(const Matrix44f& xfm) const = 0;
	virtual std::shared_ptr<Shape> GetShape() { return nullptr; }
	virtual Vec3f Sample(const DifferentialGeometry& dg, Sample3f& wi, float &tmax, const Vec2f& sample) const = 0;
};

class AreaLight : public Light, std::enable_shared_from_this<Shape> {
public:
	AreaLight(const Vec3f& Le) : Le_(Le) {
	}
	virtual Vec3f Le() const { return Le_; }

protected:
	Vec3f Le_;
};

class TriangleLight : public AreaLight {
public:
	TriangleLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Vec3f& Le)
		: AreaLight(Le)
		, v0_(v0)
		, v1_(v1)
		, v2_(v2)
		, e1_(v1 - v0)
		, e2_(v2 - v0)
		, Ng_(e1_.Cross(e2_))
		, tri_(new Triangle(v0, v1, v2)) {
	}

	std::shared_ptr<Light> Transform(const Matrix44f& xfm) const override {
		Vec3f v0, v1, v2;
		xfm.MultVecMatrix(v0_, v0);
		xfm.MultVecMatrix(v1_, v1);
		xfm.MultVecMatrix(v2_, v2);
		return std::make_shared<TriangleLight>(v0, v1, v2, Le_);
	}

	std::shared_ptr<Shape> GetShape() override { return tri_; }

	Vec3f Sample(const DifferentialGeometry& dg, Sample3f& wi, float &tmax, const Vec2f& sample) const override {
		Vec3f d = UniformSampleTriangle(sample.x, sample.y, tri_->v0_, tri_->v1_, tri_->v2_) - dg.P;
		tmax = d.Length();
		float d_dot_Ng = d.Dot(Ng_);
		if (d_dot_Ng >= 0) return 0;
		wi = Sample3f(d / tmax, (2.f * tmax * tmax * tmax) / std::abs(d_dot_Ng));
		return Le_;
	}

	Vec3f v0_;
	Vec3f v1_;
	Vec3f v2_;

private:
	Vec3f e1_;
	Vec3f e2_;
	Vec3f Ng_;
	std::shared_ptr<Triangle> tri_;
};

inline void CoordinateSystem(const Vec3f& n, Vec3f& b1, Vec3f& b2) {
#if 0
	if (std::fabs(v1.x) > std::fabs(v1.y)) {
        float inv_len = 1 / std::sqrt(v1.x * v1.x + v1.z * v1.z);
        v2 = Vec3f(v1.z * inv_len, 0, -v1.x * inv_len);
    }
    else {
        float inv_len = 1 / std::sqrt(v1.y * v1.y + v1.z * v1.z);
        v2 = Vec3f(0, -v1.z * inv_len, v1.y * inv_len); 
    }
    v3 = v1.Cross(v2);
#else
	float sign = std::copysign(1.0f, n.z);
	const float a = -1.0f / (sign + n.z);
	const float b = n.x * n.y * a;
	b1 = Vec3f(1.0f + sign * n.x * n.x * a, sign * b, -sign * n.x);
	b2 = Vec3f(b, sign + n.y * n.y * a, -n.y);
#endif
}

inline Vec3f UniformSampleCone(float r1, float r2, float cos_theta_max, const Vec3f& x, const Vec3f& y, const Vec3f& z) {
	float cos_theta = (1.f - r1) + r1 * cos_theta_max;//std::lerp(cos_theta_max, 1.f, r1);
	float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
	float phi = 2 * M_PI * r2;
	return std::cos(phi) * sin_theta * x + std::sin(phi) * sin_theta * y + cos_theta * z;
}

inline bool Quadratic(float A, float B, float C, float &t0, float &t1) {
    float discrim = B * B - 4 * A * C;
    if (discrim < 0) return false;
    float root_discrim = std::sqrt(discrim);

    float q;
    if (B < 0) 
		q = -.5f * (B - root_discrim);
    else 
		q = -.5f * (B + root_discrim);
    t0 = q / A;
    t1 = C / q;
    if (t0 > t1) std::swap(t0, t1);
    return true;
}

class SphereLight : public AreaLight {
public:
	SphereLight(Vec3f center = 0, float radius = 1, const Vec3f& Le = 1)
		: AreaLight(Le)
		, center_(center)
		, radius_(radius)
		, sphere_(new Sphere(center, radius)) {
	}

	std::shared_ptr<Light> Transform(const Matrix44f& xfm) const override {
		Vec3f center_world_pos;
		Vec3f scale;
		ExtractScaling(xfm, scale);
		assert(scale.x == scale.y && scale.x == scale.z);
		xfm.MultVecMatrix(center_, center_world_pos);
		return std::make_shared<SphereLight>(center_world_pos, radius_ * scale.x, Le_);
	}

	std::shared_ptr<Shape> GetShape() override { return sphere_; }

	Vec3f Sample(const DifferentialGeometry& dg, Sample3f& wi, float &tmax, const Vec2f& sample) const override {
#ifdef AREA_SAMPLING
		Vec3f n = UniformSampleSphere(sample.x, sample.y);
		Vec3f p = center_ + n * radius_;
		Vec3f d = p - dg.P;
		tmax = d.Length();
#elif defined(INTERSECT_METHOD)
		Vec3f dz = center_ - dg.P;
		float dz_len_2 = dz.Dot(dz);
		float dz_len = std::sqrtf(dz_len_2);
		dz /= dz_len;
		Vec3f dx, dy;

		CoordinateSystem(dz, dy, dx);

		// skip check for x inside the sphere
		float sin_theta_max_2 = radius_ * radius_ / dz_len_2;
		float cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max_2));
		Vec3f sample_dir = UniformSampleCone(sample.x, sample.y, cos_theta_max, dx, dy, dz);

		if (!Intersect(dg.P, sample_dir, tmax)) {
			tmax = (center_ - dg.P).Dot(sample_dir);
		}	
		Vec3f p = dg.P + sample_dir * tmax;
		Vec3f d = p - dg.P;
		Vec3f n = (p - center_).Normalize();
#else
		Vec3f dz = center_ - dg.P;
		float dz_len_2 = dz.Dot(dz);
		float dz_len = std::sqrtf(dz_len_2);
		dz /= -dz_len;
		Vec3f dx, dy;

		CoordinateSystem(dz, dx, dy);

		float sin_theta_max_2 = radius_ * radius_ / dz_len_2;
		float sin_theta_max = std::sqrt(sin_theta_max_2);
		float cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max_2));

		float cos_theta = 1 + (cos_theta_max - 1) * sample.x;
		float sin_theta_2 = 1.f - cos_theta * cos_theta;

		float cos_alpha = sin_theta_2 / sin_theta_max + cos_theta * std::sqrt(1 - sin_theta_2 / (sin_theta_max * sin_theta_max));
		float sin_alpha = std::sqrt(1 - cos_alpha * cos_alpha);
		float phi = 2 * M_PI * sample.y;

		Vec3f n = std::cos(phi) * sin_alpha * dx + std::sin(phi) * sin_alpha * dy + cos_alpha * dz;
		Vec3f p = center_ + n * radius_;

		Vec3f d = p - dg.P;
		tmax = d.Length();
#endif

		float d_dot_n = d.Dot(n);
		if (d_dot_n >= 0) return 0;

#if AREA_SAMPLING
		wi = Sample3f(d / tmax, (2.f * tmax * tmax * tmax) / std::abs(d_dot_n));
#else
		float pdf = 1.f / (2.f * M_PI * (1.f - cos_theta_max));
		wi = Sample3f(d / tmax, pdf);
#endif
		//abort();
		return Le_;
	}

private:
	bool Intersect(const Vec3f& orig, const Vec3f& dir, float &thit) const {
#if 0
		Vec3f l = center_ - orig;
		float tca = l.Dot(dir);
		if (tca < 0) { std::cerr << "nopppee\n"; return false; }
		float d2 = l.Dot(l) - tca * tca;
		if (d2 > radius_ * radius_) return false;
		float thc = std::sqrt(radius_ * radius_ - d2);
		float t0 = tca - thc;
		float t1 = tca + thc;
		thit = t0;
#else
		Vec3f o = orig - center_;
		Vec3f d = dir;
		float a = d.Dot(d);
		float b = 2 * o.Dot(d);
		float c = o.Dot(o) - radius_ * radius_;
		float t0, t1;
		if (!Quadratic(a, b, c, t0, t1)) return false;
		thit = t0;
#endif
		return true;
	}


	Vec3f center_{0};
	float radius_{1};
	std::shared_ptr<Sphere> sphere_;
};

class Material {
public:
	Vec3f color_;
};

class Primitive {
public:
	Primitive(const std::shared_ptr<Shape>& shape, const std::shared_ptr<Material>& material, const Matrix44f& transform) 
		: shape_(shape)
		, material_(material)
		, transform_(transform) {
	}
	Primitive(const std::shared_ptr<Light>& light, const Matrix44f& transform)
		: light_(light)
		, transform_(transform) {
	}
	std::shared_ptr<Shape> shape_;
	std::shared_ptr<Material> material_;
	std::shared_ptr<Light> light_;
	Matrix44f transform_;
};

class Instance {
public:
	Instance(size_t id, const std::shared_ptr<Shape>& shape, const std::shared_ptr<Material>& material, const std::shared_ptr<AreaLight>& light)
		: id_(id)
		, shape_(shape)
		, material_(material)
		, light_(light) {
	}
	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) const {
		dg.material = material_.get();
		dg.light = light_.get();
		shape_->PostIntersect(ray, dg);
	}
	size_t id_;
	std::shared_ptr<Shape> shape_;
	std::shared_ptr<Material> material_;
	std::shared_ptr<AreaLight> light_;
};

class Scene {
public:
	size_t Add(std::unique_ptr<Instance> instance) {
		geometry_.push_back(std::move(instance));
		return geometry_.size() - 1;
	}
	void Add(const std::shared_ptr<Light>& light) {
		lights_.push_back(light);
	}
	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) const {
		if (dg) geometry_[dg.id0]->PostIntersect(ray, dg);
	}
	std::vector<std::unique_ptr<Instance>> geometry_;
	std::vector<std::shared_ptr<Light>> lights_;

	std::unique_ptr<BuildTriangle[]> triangles_;
	std::unique_ptr<Vec3f[]> vertices_;
	size_t num_triangles_;
};

struct Triangle1v {
	const Vec3f& v0;
	const Vec3f& v1;
	const Vec3f& v2;
};

__forceinline double signmsk(const double x) {
	return _mm_cvtss_f32(_mm_and_ps(_mm_set_ss(x),_mm_castsi128_ps(_mm_set1_epi32(0x80000000))));
}

__forceinline double xorf(const double x, const double y) { 
	return _mm_cvtss_f32(_mm_xor_ps(_mm_set_ss(x),_mm_set_ss(y)));
}

void Intersect(const Ray& ray, Hit& hit, const BuildTriangle& tri, const Vec3f* vertices) {
	const Vec3f& p0 = vertices[tri.v0];
	const Vec3f& p1 = vertices[tri.v1];
	const Vec3f& p2 = vertices[tri.v2];
	const Vec3f e1 = p0 - p1;
	const Vec3f e2 = p2 - p0;
	const Vec3f Ng = e1.Cross(e2);

	const Vec3f C = p0 - ray.orig;
	const Vec3f R = ray.dir.Cross(C);
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
	hit.id0 = tri.id0;
	hit.id1 = tri.id1;
}

bool Occluded(const Ray& ray, const BuildTriangle& tri, const Vec3f* vertices) {
	const Vec3f& p0 = vertices[tri.v0];
	const Vec3f& p1 = vertices[tri.v1];
	const Vec3f& p2 = vertices[tri.v2];
	const Vec3f e1 = p0 - p1;
	const Vec3f e2 = p2 - p0;
	const Vec3f Ng = e1.Cross(e2);

	const Vec3f C = p0 - ray.orig;
	const Vec3f R = ray.dir.Cross(C);
	const float det = Ng.Dot(ray.dir);
	const float abs_det = std::abs(det);
	const float sng_det = signmsk(det);
	if (det == 0) [[unlikely]] return false;
	
	const float U = xorf(R.Dot(e2), sng_det);
	if (U < 0) [[likely]] return false;
	
	const float V = xorf(R.Dot(e1), sng_det);
	if (V < 0) [[likely]] return false;
	
	const float W = abs_det - U - V;
	if (W < 0) [[likely]] return false;
	
	const float T = xorf(Ng.Dot(C), sng_det);
	if (T < abs_det * ray.near || abs_det * ray.far < T) [[unlikely]] return false;

	return true;
}

class Integrator {
public:
	Vec3f Li(const Ray& ray, const std::unique_ptr<Scene>& scene) {
		const BuildTriangle* tris = scene->triangles_.get();
		const Vec3f* vertices = scene->vertices_.get();
		DifferentialGeometry dg;
		for (size_t i = 0; i < scene->num_triangles_; ++i) {
			Intersect(ray, dg, tris[i], vertices); 
		}

		scene->PostIntersect(ray, dg);

		if (!dg) {
			return Vec3f(0.18); // bg color
		}
		Vec3f L = 0;

		if (dg.light)
			L += dg.light->Le();

		size_t num_samples = 512;
		for (size_t i = 0; i < scene->lights_.size(); ++i) {
			Vec3f L_light = 0;
			for (size_t n = 0; n < num_samples; ++n) {
				LightSample ls;
				ls.L = scene->lights_[i]->Sample(dg, ls.wi, ls.tmax, Vec2f(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX));
				if (ls.L == 0 || ls.wi.pdf == 0 || dg.Ng.Dot(Vec3f(ls.wi)) <= 0.f) continue;
				bool in_shadow = [&]() -> bool {
					for (size_t i = 0; i < scene->num_triangles_; ++i) {
						if (Occluded(Ray(dg.P, ls.wi, dg.error * espsilon_, ls.tmax - dg.error * espsilon_), tris[i], vertices))
							return true;
					}
					return false;
				}();
				if (in_shadow) continue;
				L_light += 1 / M_PI * ls.L * dg.Ng.Dot(Vec3f(ls.wi)) / ls.wi.pdf;
			}
			L += L_light / num_samples;
		}

		return L;
	}
	float espsilon_{128.f * std::numeric_limits<float>::epsilon()};
};


std::vector<std::unique_ptr<Primitive>> prims;

void CalculateSize(const std::unique_ptr<Primitive>& prim, size_t& num_triangles, size_t& num_vertices) {
	if (prim->shape_) {
		num_vertices += prim->shape_->GetNumVertices();
		num_triangles += prim->shape_->GetNumTriangles();
	}
	else if (prim->light_) {
		if (std::shared_ptr<Shape> shape = prim->light_->GetShape()) {
			num_vertices += shape->GetNumVertices();
			num_triangles += shape->GetNumTriangles();
		}
	}
}

Box3f ExtractTriangles(std::unique_ptr<Scene>& scene, const std::unique_ptr<Primitive>& prim, size_t& num_meshes, BuildTriangle* triangles,
	size_t& num_triangles, Vec3f* vertices, size_t& num_vertices) {
	Box3f bounds;
	if (prim->shape_) {
		std::shared_ptr<Shape> shape = prim->shape_->Transform(prim->transform_);
		size_t id = scene->Add(std::make_unique<Instance>(num_meshes++, shape, prim->material_, nullptr));
		bounds.ExtendBy(shape->Extract(id, triangles, num_triangles, vertices, num_vertices));
	}
	else if (prim->light_) {
		std::shared_ptr<Light> light = prim->light_->Transform(prim->transform_);
		scene->Add(light);
		if (std::shared_ptr<Shape> shape = light->GetShape()) {
			size_t id = scene->Add(std::make_unique<Instance>(num_meshes++, shape, nullptr, std::dynamic_pointer_cast<AreaLight>(light)));
			bounds.ExtendBy(shape->Extract(id, triangles, num_triangles, vertices, num_vertices));
		}
	}
	return bounds;
}

std::unique_ptr<Scene> scene;

void MakeScene() {
	Matrix44f m;
	std::shared_ptr<Material> material = std::make_shared<Material>();
	std::shared_ptr<TriangleMesh> tri = std::make_shared<TriangleMesh>();
	tri->vertices_.push_back({-1,-1,-5});
	tri->vertices_.push_back({ 1,-1,-5});
	tri->vertices_.push_back({ 0, 1,-5});
	tri->triangles_.push_back({0, 1, 2});
	prims.push_back(std::make_unique<Primitive>(tri, material, m));

	std::shared_ptr<TriangleMesh> box = std::make_shared<TriangleMesh>();
	box->vertices_.push_back({-1.5,-1.5,-2.5});
	box->vertices_.push_back({ 1.5,-1.5,-2.5});
	box->vertices_.push_back({ 1.5,-1.5,-5.5});
	box->vertices_.push_back({-1.5,-1.5,-5.5});
	box->vertices_.push_back({-1.5, 1.5,-5.5});
	box->vertices_.push_back({-1.5, 1.5,-2.5});
	box->vertices_.push_back({ 1.5, 1.5,-5.5});
	box->triangles_.push_back({0, 1, 2});
	box->triangles_.push_back({0, 2, 3});
	box->triangles_.push_back({0, 3, 4});
	box->triangles_.push_back({0, 4, 5});
	box->triangles_.push_back({3, 2, 6});
	box->triangles_.push_back({3, 6, 4});
	prims.push_back(std::make_unique<Primitive>(box, material, m));

	std::shared_ptr<TriangleLight> light0 = std::make_shared<TriangleLight>(
		Vec3f(-1, -1, 0), 
		Vec3f( 1, -1, 0), 
		Vec3f( -1, 1, 0), 
		Vec3f(5.f));
	
	std::shared_ptr<TriangleLight> light1 = std::make_shared<TriangleLight>(
		Vec3f( 1, -1, 0), 
		Vec3f( 1,  1, 0), 
		Vec3f( -1, 1, 0), 
		Vec3f(5.f));

	//Matrix44<float> xfm_light(0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, -1, 0, -4, 1);
	//prims.push_back(std::make_unique<Primitive>(light0, xfm_light));
	//prims.push_back(std::make_unique<Primitive>(light1, xfm_light));
	
	//std::shared_ptr<Sphere> sphere = std::make_shared<Sphere>();
	//Matrix44<float> xfm_sphere(.1, 0, 0, 0, 0, .1, 0, 0, 0, 0, .1, 0, 0, -1.2, -4, 1);
	//prims.push_back(std::make_unique<Primitive>(sphere, material, xfm_sphere));

	Matrix44<float> xfm_sphere(.2, 0, 0, 0, 0, .2, 0, 0, 0, 0, .2, 0, 0, 0, -4, 1);
	std::shared_ptr<SphereLight> sphere_light = std::make_shared<SphereLight>(0, 1, 20);
	prims.push_back(std::make_unique<Primitive>(sphere_light, xfm_sphere));

	// once we have all prims we need to process them
	size_t num_allocated_triangles = 0;
	size_t num_allocated_vertices = 0;
	for (const auto& prim : prims) {
		CalculateSize(prim, num_allocated_triangles, num_allocated_vertices);
	}

	size_t num_meshes = 0;
	size_t num_triangles = 0;
	size_t num_vertices = 0;

	scene = std::make_unique<Scene>();
	scene->triangles_ = std::make_unique<BuildTriangle[]>(num_allocated_triangles);
	scene->vertices_ = std::make_unique<Vec3f[]>(num_allocated_vertices);
	scene->num_triangles_ = num_allocated_triangles;

	for (const auto& prim : prims) {
		ExtractTriangles(scene, prim, num_meshes, scene->triangles_.get(), num_triangles, scene->vertices_.get(), num_vertices);
	}
}

void Render() {
	size_t width = 780; 
	size_t height = 585;
	std::unique_ptr<Camera> camera = std::make_unique<Camera>();
	camera->cam_to_world_ = Matrix44<float>(0.827081, 0, -0.562083, 0, -0.152433, 0.962525, -0.224298, 0, 0.541019, 0.271192, 0.796086, 0, 2.924339, 1.020801, 0.511729, 1);
	std::unique_ptr<Integrator> integrator = std::make_unique<Integrator>();
	std::unique_ptr<unsigned char[]> framebuffer = std::make_unique<unsigned char[]>(3 * width * height); 
	auto start = std::chrono::high_resolution_clock::now();
	for (size_t y = 0; y < height; ++y) {
		for (size_t x = 0; x < width; ++x) {
			Ray primary;
			camera->SetRay(Vec2f((x + 0.5f) / width, (y + 0.5f) / height), primary);
			Vec3f L = integrator->Li(primary, scene);
			framebuffer[(y * width + x) * 3] = std::clamp(L.x, 0.f, 1.f) * 255;
			framebuffer[(y * width + x) * 3 + 1] = std::clamp(L.y, 0.f, 1.f) * 255;
			framebuffer[(y * width + x) * 3 + 2] = std::clamp(L.z, 0.f, 1.f) * 255;
		}
	}
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> duration_seconds = end - start;
	std::cout << "Time taken: " << duration_seconds.count() << " seconds" << std::endl;
	std::ofstream ofs("./test.ppm", std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	ofs.write((char*)framebuffer.get(), 3 * width * height);
	ofs.close();
}

int main() {
	MakeScene();
	Render();

	return 0;
}