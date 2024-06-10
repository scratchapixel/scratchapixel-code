#define _USE_MATH_DEFINES
#include <iostream>
#include <limits>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <algorithm>
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

class Shape {
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

class TriangleMesh : public Shape, public std::enable_shared_from_this<TriangleMesh> {
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
	std::shared_ptr<Shape> Transform(const Matrix44f& m) const override {
		return const_cast<TriangleMesh*>(this)->shared_from_this();
	}

	void PostIntersect(const Ray& ray, DifferentialGeometry& dg) const override {
		const Triangle& tri = triangles_[dg.id1];
		const Vec3f& v0 = vertices_[tri.v0];
		const Vec3f& v1 = vertices_[tri.v1];
		const Vec3f& v2 = vertices_[tri.v2];

		const float u = dg.u, v = dg.v, w = 1.f - u - v, t = dg.t;

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

// sample.h
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

// sampler.h
struct LightSample {
	Sample3f wi;
	float tmax;
	Vec3f L;
};

Vec3f UniformSampleTriangle(const float& u, const float& v, const Vec3f& A, const Vec3f& B, const Vec3f& C) {
	float su = std::sqrt(u);
	return Vec3f(C + (1.f - su) * (A - C) + (v * su) * (B - C));
}

class Light {
public:
	virtual std::shared_ptr<Light> Transform(const Matrix44f& xfm) const = 0;
	virtual std::shared_ptr<Shape> GetShape() { return nullptr; }
	virtual Vec3f Sample(const DifferentialGeometry& dg, Sample3f& wi, float &tmax, const Vec2f& sample, bool debug) const = 0;
};

class AreaLight : public Light {
public:	
};

class TriangleLight : public AreaLight {
public:
	TriangleLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Vec3f& Le)
		: v0_(v0)
		, v1_(v1)
		, v2_(v2)
		, e1_(v1 - v0)
		, e2_(v2 - v0)
		, Ng_(e1_.Cross(e2_))
		, Le_(Le)
		, tri_(new Triangle(v0, v1, v2)) {
			std::cerr << "light normal " << Ng_ << std::endl;
	}

	std::shared_ptr<Light> Transform(const Matrix44f& xfm) const override {
		Vec3f v0, v1, v2;
		xfm.MultVecMatrix(v0_, v0);
		xfm.MultVecMatrix(v1_, v1);
		xfm.MultVecMatrix(v2_, v2);
		return std::make_shared<TriangleLight>(v0, v1, v2, Le_);
	}

	std::shared_ptr<Shape> GetShape() override { return tri_; }

	Vec3f Sample(const DifferentialGeometry& dg, Sample3f& wi, float &tmax, const Vec2f& sample, bool debug) const override {
		Vec3f d = UniformSampleTriangle(sample.x, sample.y, tri_->v0_, tri_->v1_, tri_->v2_) - dg.P;
		//std::cerr << dg.P << ", " << temp << std::endl; abort();
		tmax = d.Length();
		float d_dot_Ng = d.Dot(Ng_);
		//std::cerr << dg.P << ", " << temp << ". " << Ng_ << std::endl; abort();
		if (d_dot_Ng >= 0) return 0;
		//if (debug) std::cerr << "Sample " << d << std::endl;
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
	Vec3f Le_;
	std::shared_ptr<Triangle> tri_;
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
	//Primitive(const std::shared_ptr<Shape>& shape, const std::shared_ptr<LIght>& light, const 
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

	// will go away when we implement the bvh
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

/*
void Intersect2(const Ray& ray, Hit& hit, const BuildTriangle& tri, const Vec3f* vertices) {
	const Vec3f& O = ray.orig;
	const Vec3f& D = ray.dir;
	const Vec3f v0 = vertices[tri.v0] - O;
	const Vec3f v1 = vertices[tri.v1] - O;
	const Vec3f v2 = vertices[tri.v2] - O;
	const Vec3f e0 = v2 - v0;
	const Vec3f e1 = v0 - v1;
	const Vec3f e2 = v1 - v2;
	const Vec3f Ng = 2.f * e1.Cross(e0);
	const float det = Ng.Dot(D);
	const float abs_det = std::abs(det); 
	const float sign_det = signmsk(det);

	// perform edge tests
	const float U = xorf(((v2+v0).Cross(e0)).Dot(D), sign_det);
	if (U < 0.f) [[unlikely]] return;
	const float V = xorf(((v0+v1).Cross(e1)).Dot(D), sign_det);
	if (V < 0.f) [[unlikely]] return;
	const float W = xorf(((v1+v2).Cross(e2)).Dot(D), sign_det);
	if (W < 0.f) [[unlikely]] return;

	// perform depth test
	const float T = xorf(v0.Dot(Ng), sign_det);
	if (abs_det * float(hit.t) < T) [[unlikely]] return;
	if (T < abs_det * float(ray.near)) [[unlikely]] return;
	if (det == 0.f) [[unlikely]] return;

	// update hit information
	const float rcp_abs_det = 1.f / abs_det;
	hit.u   = U * rcp_abs_det;
	hit.v   = V * rcp_abs_det;
	hit.t   = T * rcp_abs_det;
	hit.id0 = tri.id0;
	hit.id1 = tri.id1;
}
*/

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
	Vec3f Li(const Ray& ray, const std::unique_ptr<Scene>& scene, bool debug = false) {
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
			L += Vec3f(1);

		size_t num_samples = 2048;
		for (size_t i = 0; i < scene->lights_.size(); ++i) {
			Vec3f L_light = 0;
			for (size_t n = 0; n < num_samples; ++n) {
				LightSample ls;
				ls.L = scene->lights_[i]->Sample(dg, ls.wi, ls.tmax, Vec2f(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX), debug);
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


// todo: what happens to prims once we don't need them anymore (since they are declared as globals?)?
// todo: please review all shared and unique_ptrs... Look at scene differences betwen geoms and lights
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

	std::shared_ptr<TriangleLight> light = std::make_shared<TriangleLight>(
		Vec3f(-1, -1, 0), 
		Vec3f( 1, -1, 0), 
		Vec3f( 0,  1, 0), 
		Vec3f(5.f));

	Matrix44<float> xfm_light(0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, -1, 0, -4, 1);
	prims.push_back(std::make_unique<Primitive>(light, xfm_light));

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
	for (size_t y = 0; y < height; ++y) {
		for (size_t x = 0; x < width; ++x) {
			Ray primary;
			camera->SetRay(Vec2f((x + 0.5f) / width, (y + 0.5f) / height), primary);
			Vec3f L = integrator->Li(primary, scene, (x == 330 && y == 230));
			framebuffer[(y * width + x) * 3] = std::clamp(L.x, 0.f, 1.f) * 255;
			framebuffer[(y * width + x) * 3 + 1] = std::clamp(L.y, 0.f, 1.f) * 255;
			framebuffer[(y * width + x) * 3 + 2] = std::clamp(L.z, 0.f, 1.f) * 255;
		}
	}
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