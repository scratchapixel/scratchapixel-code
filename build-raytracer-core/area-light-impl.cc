#define _USE_MATH_DEFINES 
#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
#include <cassert>

#include <atomic> // ref count
#include <map>

#ifdef _WIN32
#include <malloc.h>  // Windows-specific header for _aligned_malloc/_aligned_free
#endif

template<typename T>
class Vec2 {
public:
	using type = T;
	Vec2() noexcept : x(0), y(0) {}
	Vec2(T xx) : x(xx), y(xx) {}
	Vec2(T xx, T yy) : x(xx), y(yy) {}
	T x, y;
};

template<typename T>
class Vec3 {
public:
	using type = T;
	Vec3() noexcept : x(0), y(0), z(0) {}
	Vec3(T xx) : x(xx), y(xx), z(xx) {}
	Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
	constexpr unsigned int Dimensions() const noexcept {
		return 3;
	}
	constexpr T& operator[](int i) noexcept {
		return (&x)[i];
	}
	constexpr const T& operator[](int i) const noexcept {
		return (&x)[i];
	}
	// Return a reference to the instance it modifies
	const Vec3& Normalize() noexcept {
		T len = std::sqrt(x * x + y * y + z * z);
		if (len != 0) [[likely]] {
			x /= len;
			y /= len;
			z /= len;
		}
		return *this;
	}
	constexpr Vec3 Cross(const Vec3& v) const noexcept {
		return {y * v.z - z * v.y,
				z * v.x - x * v.z,
				x * v.y - y * v.x};
	}
	constexpr Vec3 operator-(const Vec3& v) const noexcept {
		return {x - v.x, y - v.y, z - v.z};
	}
	constexpr Vec3 operator+(const Vec3& v) const noexcept {
		return {x + v.x, y + v.y, z + v.z};
	}
	constexpr Vec3 operator*(T a) const noexcept {
		return {x * a, y * a, z * a};
	}
	friend constexpr Vec3 operator*(T a, const Vec3& v) noexcept{ 
		return {a * v.x, a * v.y, a * v.z};
	}
	// @todo not safe
	friend constexpr Vec3 operator/(T a, const Vec3& v) noexcept{ 
		return {a / v.x, a / v.y, a / v.z};
	}
	T x, y, z;
};

template<typename V> 
class Box {
public:
	V min_, max_;
	void MakeEmpty() noexcept {
		min_ = std::numeric_limits<typename V::type>::max();
		max_ = std::numeric_limits<typename V::type>::lowest();
	}
	void ExtendBy(const V& point) noexcept {
		for (unsigned int i = 0; i < min_.Dimensions(); ++i) {
			if (point[i] < min_[i]) min_[i] = point[i];
			if (point[i] > max_[i]) max_[i] = point[i];
		}
	}
	void ExtendBy(const Box& box) noexcept {
		for (unsigned int i = 0; i < min_.Dimensions(); ++i) {
			if (box.min_[i] < min_[i]) min_[i] = box.min_[i];
			if (box.max_[i] > max_[i]) max_[i] = box.max_[i];
		}
	}
};

using Box3f = Box<Vec3<float>>;

template<typename T>
class Matrix44 {
public:
	constexpr Matrix44() noexcept {
		x[0][0] = 1;
		x[0][1] = 0;
		x[0][2] = 0;
		x[0][3] = 0;
		x[1][0] = 0;
		x[1][1] = 1;
		x[1][2] = 0;
		x[1][3] = 0;
		x[2][0] = 0;
		x[2][1] = 0;
		x[2][2] = 1;
		x[2][3] = 0;
		x[3][0] = 0;
		x[3][1] = 0;
		x[3][2] = 0;
		x[3][3] = 1;
	}
	constexpr Matrix44(
		T a, T b, T c, T d,
		T e, T f, T g, T h,
		T i, T j, T k, T l,
		T m, T n, T o, T p) noexcept {
		x[0][0] = a; x[0][1] = b; x[0][2] = c; x[0][3] = d;
		x[1][0] = e; x[1][1] = f; x[1][2] = g; x[1][3] = h;
		x[2][0] = i; x[2][1] = j; x[2][2] = k; x[2][3] = l;
		x[3][0] = m; x[3][1] = n; x[3][2] = o; x[3][3] = p;
	}
	constexpr Matrix44(const T* m) {
		std::memcpy(&x[0], m, sizeof(T) * 16);
	}
	constexpr bool operator==(const Matrix44& rhs) const noexcept {
		return x[0][0] == rhs.x[0][0] && x[0][1] == rhs.x[0][1] &&
			   x[0][2] == rhs.x[0][2] && x[0][3] == rhs.x[0][3] &&
			   x[1][0] == rhs.x[1][0] && x[1][1] == rhs.x[1][1] &&
			   x[1][2] == rhs.x[1][2] && x[1][3] == rhs.x[1][3] &&
			   x[2][0] == rhs.x[2][0] && x[2][1] == rhs.x[2][1] &&
			   x[2][2] == rhs.x[2][2] && x[2][3] == rhs.x[2][3] &&
			   x[3][0] == rhs.x[3][0] && x[3][1] == rhs.x[3][1] &&
			   x[3][2] == rhs.x[3][2] && x[3][3] == rhs.x[3][3];
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
	T* operator[] (int i) noexcept{
		return x[i];
	}
	const T* operator[] (int i) const noexcept {
		return x[i];
	}
public:
	T x[4][4];
};

///////////////////////////////////////////////////////////////////////////////
// rtCore/common/accel.h

struct BuildTriangle {
	int v0, v1, v2;
	int id0, id1;
};

struct BuildVertex {
	float x, y, z;
	int align;
};

//rtCore/common/ray.h
struct Ray {
	Ray() = default;
	Ray(const Vec3<float>& orig, const Vec3<float>& dir, float near = 0, float far = std::numeric_limits<float>::infinity())
		: orig(orig)
		, dir(dir)
		, inv_dir(1 / dir) // @todo you probably want something safer here
		, near(near)
		, far(far) {
	}
	Vec3<float> orig;
	Vec3<float> dir;
	Vec3<float> inv_dir; // 1 / dir
	float near;
	float far;
};

///////////////////////////////////////////////////////////////////////////////


// in common/sys/ref.h
class RefCount {
public:
	RefCount() : ref_counter_(0) {}
	virtual ~RefCount() = default;
	virtual void IncRef() { ref_counter_++; }
	virtual void DecRef() { if (--ref_counter_ == 0) delete this; }

//private:
	std::atomic<size_t> ref_counter_;
};

// The instance held by a Ref has to be derived from RefCount, since
// Ref methods are calling IncRef() and DecRef() from RefCount.
// So effectively this is a "specialized" class that can only act
// on object that are reference counted.
template<typename Type>
class Ref {
public:
	Type* const ptr_; // pointer is const, can't change the address
	Ref() : ptr_(nullptr) {}
	Ref(std::nullptr_t) : ptr_(nullptr) {}
	Ref(const Ref& input) : ptr_(input.ptr_) { 
		if (ptr_) ptr_->IncRef(); 
		//if (ptr_) std::cerr << "In Ref copy ctor: " << ((RefCount*)ptr_)->ref_counter_ << std::endl;
	}
	Ref(Type* const input) : ptr_(input) { 
		//if (!input) std::cerr << "Invalid input ..." << std::endl;
		if (ptr_) ptr_->IncRef(); 
		//if (ptr_ != nullptr) 
		//	std::cerr << "In Ref copy ctor (pointer type): " << ptr_ << " " << ((RefCount*)ptr_)->ref_counter_ << std::endl; 
		//else
		//	std::cerr << "Invalid ptr check ..." << std::endl;
	}
	~Ref() { if (ptr_) ptr_->DecRef(); }
	Ref& operator=(const Ref& input) {
		//std::cerr << "-- assign operator const Ref&" << std::endl;
		if (input.ptr_) input.ptr_->IncRef();
		if (ptr_) ptr_->DecRef();
		const_cast<Type*&>(ptr_) = input.ptr_;
		//*(Type**)&ptr_ = input.ptr_;
		return *this;
	}
	// Useful for things like this->camera_ = nullptr; where camera is Ref<Camera>
	// Since we don't use the ref anylonger we need to be sure the object's ref
	// counter is decremented.
	Ref& operator=(std::nullptr_t) {
        if (ptr_) ptr_->DecRef();
		const_cast<Type*&>(ptr_) = nullptr;
        return *this;
    }
	operator bool() const { return ptr_ != nullptr; }
	const Type& operator*() const { return *ptr_; }
	Type& operator*() { return *ptr_; }
	const Type* operator->() const { return ptr_; }
	Type* operator->() { return ptr_; }
};

///////////////////////////////////////////////////////////////////////////////
// Data & Params

class Data : public RefCount {
public:
	Data(size_t bytes) : bytes_(bytes) {
		ptr_ = _aligned_malloc(bytes, 64); // use std::aligned_alloc if on Linux
		if (ptr_ == nullptr)
			throw std::bad_alloc();
	}
	Data(size_t bytes, const void* ptr, bool copy = true) : bytes_(bytes) {
		if (copy) {
			ptr_ = _aligned_malloc(bytes, 64); // use std::aligned_alloc if on Linux
			if (ptr_ == nullptr)
				throw std::bad_alloc();
			std::memcpy(ptr_, ptr, bytes);
		}
		else {
			ptr_ = (void*)ptr;
		}
	}
	~Data() {
		std::free(ptr_);
	}
	const char* Map() const { return (const char*)ptr_; }
	char* Map() { return (char*)ptr_; }
	size_t GetSize() const { return bytes_; }
	void* ptr_{nullptr};
	size_t bytes_;
};	

class DataStream : public RefCount {
public:
	DataStream(const Ref<Data>& ptr, size_t elements, size_t stride, size_t offset)
		: ptr_(ptr)
		, elements_(elements)
		, stride_(stride)
		, offset_(offset) {
	}
private:
	Ref<Data> ptr_;
	size_t elements_;
	size_t stride_;
	size_t offset_;
};

class Variant {
public:
	enum Type {
		kEmpty,
		kBool1,
		kBool2,
		kBool3,
		kBool4,
		kInt1,
		kInt2,
		kInt3,
		kInt4,
		kFloat1,
		kFloat2,
		kFloat3,
		kFloat4,
		kString,
		kImage,
		kTexture,
		kTransform,
	};
	Variant() : type_(kEmpty) {}
	Variant(float f0) : type_(kFloat1) { f[0] = f0; }
	Variant(const Matrix44<float>& m) : type_(kTransform) {
		f[ 0] = m[0][0]; f[ 1] = m[0][1]; f[ 2] = m[0][2]; f[ 3] = m[0][3];
		f[ 4] = m[1][0]; f[ 5] = m[1][1]; f[ 6] = m[1][2]; f[ 7] = m[1][3];
		f[ 8] = m[2][0]; f[ 9] = m[2][1]; f[10] = m[2][2]; f[11] = m[2][3];
		f[12] = m[3][0]; f[13] = m[3][1]; f[14] = m[3][2]; f[15] = m[3][3];
	}
	float GetFloat() const { return f[0]; }
	Matrix44<float> GetTransform() const {
		return Matrix44<float>(f);
	}
	Type type_;
	union {
		bool b[4];
		int i[4];
		float f[16];
	};
	std::string str_;
	Ref<DataStream> data_;
	//Ref<Image> image_;
	//Ref<Texture> texture_;
};

class Parms {
public:
	float GetFloat(const char* name, float def = 0) const {
		auto iter = m_.find(name); /* std::map<std::string,Variant>::const_iterator */
		if (iter == m_.end() || (*iter).second.type_ != Variant::kFloat1) return def;
		return (*iter).second.GetFloat();
	}
	Matrix44<float> GetTransform(const char* name) const {
		auto iter = m_.find(name); /* std::map<std::string,Variant>::const_iterator */
		if (iter == m_.end() || (*iter).second.type_ != Variant::kTransform) return Matrix44<float>();
		return (*iter).second.GetTransform();
	}
	void Add(const std::string& name, Variant data) {
		m_[name] = data;
	}
private:
	std::map<std::string, Variant> m_;
};

//static struct NullTy {
//} null;	

//
class Shape : public RefCount {
public:	
	Shape() = default;
	virtual size_t GetNumTriangles() const = 0;
	virtual size_t GetNumVertices() const = 0;
	virtual Ref<Shape> Transform(const Matrix44<float>& xfm) const = 0;
	virtual Box3f Extract(size_t id, BuildTriangle* triangles, size_t& num_triangles, BuildVertex* vertices, size_t& num_vertices) const = 0;
};

class TriangleMesh : public Shape {
protected:
	TriangleMesh() = default;
public:	
	TriangleMesh(const Parms& parms) {
	}
	struct Triangle {
		uint32_t v0;
		uint32_t v1;
		uint32_t v2;
	};
	virtual size_t GetNumTriangles() const override {
		return triangles_.size();
	}
	virtual size_t GetNumVertices() const override {
		return vertices_.size();
	}
	Ref<Shape> Transform(const Matrix44<float>& xfm) const override {
		if (xfm == Matrix44<float>())
			return (Shape*)this;
		TriangleMesh* mesh = new TriangleMesh;
		mesh->vertices_.resize(vertices_.size());
		for (size_t i = 0; i < vertices_.size(); ++i)
			xfm.MultVecMatrix(vertices_[i], mesh->vertices_[i]);
		return (Shape*)mesh;
	}
	Box3f Extract(size_t id, BuildTriangle* triangles, size_t& num_triangles, BuildVertex* vertices, size_t& num_vertices) const override {
		Box3f bounds;
		for (size_t j = 0; j < triangles_.size(); ++j) {
			const TriangleMesh::Triangle& tri = triangles_[j];
			triangles[num_triangles++] = {(int)(num_vertices + tri.v0), (int)(num_vertices + tri.v1), (int)(num_vertices + tri.v2), (int)id, (int)j};
		}
		for (size_t j = 0; j < vertices_.size(); ++j) {
			const Vec3<float>& v = vertices_[j];
			vertices[num_vertices++] = {v.x, v.y, v.z};
			bounds.ExtendBy(v);
		}
		return bounds;
	}
	std::vector<Vec3<float>> vertices_;
	std::vector<Vec3<float>> normals_;
	std::vector<Triangle> triangles_; // vector_t?
};

class Sphere : public TriangleMesh {
public:
	Sphere(const Parms& parms) {
		Triangulate();
	}
	//Sphere() {
	//	Triangulate();
	//	std::cerr << "In sphere ctor" << std::endl;
	//}
private:
	Vec3<float> Eval(float theta, float phi) {
		return {std::sin(theta) * std::cos(phi), std::cos(theta), std::sin(theta) * std::sin(phi)};
	}
	void Triangulate() {
		float rcp_num_theta = 1.f / num_theta_;
		float rcp_num_phi = 1.f / num_phi_;
		for (uint32_t theta = 0; theta <= num_theta_; ++theta) {
			for (uint32_t phi = 0; phi < num_phi_; ++phi) {
				Vec3<float> p = Eval(theta * M_PI * rcp_num_theta, phi * 2.f * M_PI * rcp_num_phi);
				Vec3<float> dpdu = Eval((theta + 0.001f) * M_PI * rcp_num_theta, phi * 2.f * M_PI * rcp_num_phi) - p;
				Vec3<float> dpdv = Eval(theta * M_PI * rcp_num_theta, (phi + 0.001f) * 2.f * M_PI * rcp_num_phi) - p;
				p = center_ + radius_ * p;
				vertices_.push_back(p);
				normals_.push_back(dpdv.Cross(dpdv).Normalize());
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
	Vec3<float> center_{0};
	float radius_{1};
	uint32_t num_theta_{8};
	uint32_t num_phi_{8};
};

class Light : public RefCount {
public:	
};

class AreaLight : public Light {
public:
};

class Material : public RefCount {
public:	
};

class Matte : public Material {
public:
	Matte(const Parms& parms) {}
	Vec3<float> reflectance_; // or albedo 
};

// api/instance.h
// A shape with a material and a light attached
class Instance : public RefCount {
public:
	Instance(size_t id, const Ref<Shape>& shape, const Ref<Material>& material, const Ref<AreaLight>& light)
		: id_(id)
		, shape_(shape)
		, material_(material)
		, light_(light) {
		std::cerr << "\tin Instance ctor " << shape_.ptr_ << std::endl;
	}
public:
	size_t id_;
	Ref<Shape> shape_;
	Ref<Material> material_;
	Ref<AreaLight> light_;
};

///////////////////////////////////////////////////////////////////////////////

class Accel : public RefCount {
public:
};

///////////////////////////////////////////////////////////////////////////////

// api/scene.h
class Scene : public RefCount {
public:
	Scene() = default;
	~Scene() {
		if (triangles_) delete [] triangles_;
		if (vertices_) delete [] vertices_;
	}
	size_t Add(const Ref<Instance>& instance) {
		geometry_.push_back(instance);
		return geometry_.size() - 1;
	}
	std::vector<Ref<Instance>> geometry_;
	BuildTriangle* triangles_{nullptr};
	BuildVertex* vertices_{nullptr};
	size_t num_triangles_{0};
};



///////////////////////////////////////////////////////////////////////////////

class Camera : public RefCount {
public:
	virtual ~Camera() {}
	virtual void SetRay(const Vec2<float>& pixel, const Vec2<float>& sample, Ray& ray) const = 0;
	float angle_;
};

class PinholeCamera : public Camera {
public:
	PinholeCamera(const Parms& parms) {
		local_to_world_ = parms.GetTransform("local2world");
		local_to_world_.MultVecMatrix(Vec3<float>(0), orig_);
		angle_ = parms.GetFloat("fieldOfView", 50);
		aspect_ratio_ = parms.GetFloat("aspectRatio", 1.f);
		std::cerr << "Angle and aspect ration for the camera " << angle_ << " " << aspect_ratio_ << std::endl;
		std::cerr << "Check matrix " << local_to_world_[3][3] << std::endl;
		scale_ = 1 / std::tan(angle_ * 0.5 * M_PI / 180.f);
	}
	void SetRay(const Vec2<float>& pixel, const Vec2<float>& sample, Ray& ray) const override {
		Vec3<float> dir_local = {(pixel.x - 0.5f) * aspect_ratio_,  (1.f - pixel.y) - 0.5f, 0.5f * scale_};
		ray.orig = Vec3<float>(local_to_world_[3][0], local_to_world_[3][1], local_to_world_[3][2]);
		local_to_world_.MultDirMatrix(dir_local, ray.dir);
		ray.dir.Normalize();
		ray.near = 0;
		ray.far = std::numeric_limits<float>::infinity();
		ray.inv_dir = 1 / ray.dir;
	}
	float aspect_ratio_, scale_;
	Vec3<float> orig_;
	Matrix44<float> local_to_world_;
};

///////////////////////////////////////////////////////////////////////////////
// Renderer

class Integrator : public RefCount {
public:
	virtual ~Integrator() = default;
	virtual Vec3<float> Li(const Ray& ray, const Ref<Scene>& scene) = 0;
};

class PathTraceIntegrator : public Integrator {
public:
	PathTraceIntegrator(const Parms& parms) {
	}
	Vec3<float> Li(const Ray& ray, const Ref<Scene>& scene) override {
		for (size_t i = 0; i < scene->num_triangles_; ++i) {
			
		}
		return Vec3<float>(0);
	}
};

class Renderer : public RefCount {
public:
	virtual ~Renderer() = default;
	virtual void RenderFrame(const Ref<Camera>& camera, const Ref<Scene>& scene) = 0;
};

class IntegratorRenderer : public Renderer {
public:
	IntegratorRenderer(const Parms& parms) {
		integrator_ = new PathTraceIntegrator(parms);
	}
	void RenderFrame(const Ref<Camera>& camera, const Ref<Scene>& scene) override {
		this->camera_ = camera;
		this->scene_ = scene;
		RenderThread();
		this->camera_ = nullptr;
		this->scene_ = nullptr;
	}
private:
	void RenderThread() {
		std::cerr << "Rendering the frame" << std::endl;
		uint32_t width = 640, height = 480;
		for (uint32_t y = 0; y < height; ++y) {
			for (uint32_t x = 0; x < width; ++x) {
				Ray primary;
				camera_->SetRay(Vec2<float>((x + 0.5f) / width, (y + 0.5f) / height), 0, primary);
				Vec3<float> L = integrator_->Li(primary, scene_);
			}	
		}
	}

	Ref<Integrator> integrator_;

	Ref<Camera> camera_;
	Ref<Scene> scene_;
};

///////////////////////////////////////////////////////////////////////////////

// in api/handle.h
struct _RTHandle {
	std::atomic<size_t> counter;
	_RTHandle() : counter(1) {}
	virtual ~_RTHandle() = default;
	void IncRef() { counter++; };
	void DecRef() {
		if (--counter == 0)
			delete this;
	}
	virtual void Create() = 0;
	virtual void Set(const std::string& property, const Variant& data) = 0;
};

template<typename B> 
class BaseHandle : public _RTHandle {
public:
	Parms parms_;
	Ref<B> instance_;
};

template<typename T, typename B> 
class NormalHandle : public BaseHandle<B> {
public:
	void Create() { this->instance_ = new T(this->parms_); }
	void Set(const std::string& property, const Variant& data) {
		this->parms_.Add(property, data);
	}
};

template<typename T>
class ConstHandle : public _RTHandle {
public:
	ConstHandle(const Ref<T>& ptr) : instance_(ptr) {}
	void Create() { throw std::runtime_error("cannot modify constant handle"); }
	void Set(const std::string& property, const Variant& data) { throw std::runtime_error("cannot modify constant handle"); }
public:
	Ref<T> instance_;
};

// in api/render_device.h/.cpp
class PrimitiveHandle : public _RTHandle {
public:
	PrimitiveHandle(const Ref<Shape>& shape, const Ref<Material>& material, const Matrix44<float>& transform) 
		: shape_(shape)
		, material_(material)
		, transform_(transform) {
		std::cerr << "\tin prim handle " << shape_.ptr_ << std::endl;
	}
	void Create() { throw std::runtime_error("cannot modify constant handle"); }
	void Set(const std::string& property, const Variant& data) { throw std::runtime_error("cannot modify constant handle"); }
public:
	Ref<Shape> shape_;
	Ref<Material> material_;
	Ref<Light> light_;
	Matrix44<float> transform_;
};

///////////////////////////////////////////////////////////////////////////////

// in api/device.h
class Device {
public:
	typedef struct __RTHandle{}* RTHandle;
	typedef struct __RTShape : public __RTHandle{}* RTShape;
	typedef struct __RTPrimitive : public __RTHandle{}* RTPrimitive;
	typedef struct __RTScene : public __RTHandle{}* RTScene;
	typedef struct __RTMaterial : public __RTHandle{}* RTMaterial;
	typedef struct __RTCamera : public __RTHandle{}* RTCamera;
	typedef struct __RTRenderer : public __RTHandle{}* RTRenderer;
	virtual RTShape rtNewShape() = 0;
	virtual RTPrimitive rtNewShapePrimitive(Device::RTShape shape, Device::RTMaterial material, const float* transform) = 0;
	virtual RTScene rtNewScene(RTPrimitive* prims, size_t size) = 0;
	virtual void rtCommit(RTHandle handle) = 0;
	virtual void rtIncRef(RTHandle handle) = 0;
	virtual void rtDecRef(RTHandle handle) = 0; 
};

class RenderDevice : public Device {
public:
	Device::RTShape rtNewShape() {
		return (Device::RTShape) new NormalHandle<Sphere,Shape>;
	}
	Device::RTMaterial rtNewMaterial() {
		return (Device::RTMaterial) new NormalHandle<Matte,Material>;
	}
	Device::RTPrimitive rtNewShapePrimitive(Device::RTShape shape, Device::RTMaterial material, const float* transform) {
		BaseHandle<Shape>* shape_handle = dynamic_cast<BaseHandle<Shape>*>((_RTHandle*)shape);
		BaseHandle<Material>* material_handle = dynamic_cast<BaseHandle<Material>*>((_RTHandle*)material);
		Matrix44<float> space = transform ? Matrix44<float>(transform) : Matrix44<float>();
		return (Device::RTPrimitive) new PrimitiveHandle(shape_handle->instance_, material_handle->instance_, space);
	}
	Device::RTScene rtNewScene(Device::RTPrimitive* prims, size_t size) {
		Ref<Scene> scene = new Scene;
		size_t num_allocated_triangles = 0;
		size_t num_allocated_vertices = 0;
		for (size_t i = 0; i < size; ++i) {
			CalculateSize(prims[i], num_allocated_triangles, num_allocated_vertices);
		}
		std::cerr << "Num alloc vertices " << num_allocated_vertices << ", Num alloc triangles " << num_allocated_triangles << std::endl;
		size_t num_meshes = 0;
		size_t num_triangles = 0;
		size_t num_vertices = 0;
		BuildTriangle* triangles = new BuildTriangle[num_allocated_triangles];
		BuildVertex* vertices = new BuildVertex[num_allocated_vertices];
		std::cerr << "Extracting...(size: " << size << ")" << std::endl;
		Box3f bounds;
		for (size_t i = 0; i < size; ++i) {
			bounds.ExtendBy(ExtractTriangles(scene, prims[i], num_meshes, triangles, num_triangles, vertices, num_vertices));
		}
		std::cerr << "Done with processing shapes " << std::endl;
		scene->triangles_ = triangles;
		scene->vertices_ = vertices;
		scene->num_triangles_ = num_triangles;
		return (Device::RTScene) new ConstHandle<Scene>(scene);
	}
	Device::RTCamera rtNewCamera() {
		return (Device::RTCamera) new NormalHandle<PinholeCamera, Camera>;
	}
	Device::RTRenderer rtNewRenderer() {
		return (Device::RTRenderer) new NormalHandle<IntegratorRenderer, Renderer>;
	}
	void rtCommit(Device::RTHandle handle) {
		if (!handle) throw std::runtime_error("invalid handle");
		((_RTHandle*)handle)->Create();
	}
	void rtIncRef(Device::RTHandle handle) {
		((_RTHandle*)handle)->IncRef();
	}
	void rtDecRef(Device::RTHandle handle) {
		((_RTHandle*)handle)->DecRef();
	}
	void rtSetFloat1(Device::RTHandle handle, const char* property, float x) {
		((_RTHandle*)handle)->Set(property, Variant(x));
	}
	void rtSetTransform(Device::RTHandle handle, const char* property, const Matrix44<float>& transform) {
		((_RTHandle*)handle)->Set(property, Variant(transform));
	}
	void rtRenderFrame(Device::RTRenderer renderer_i, Device::RTCamera camera_i, Device::RTScene scene_i) {
		BaseHandle<Renderer>* renderer = dynamic_cast<BaseHandle<Renderer>*>((_RTHandle*)renderer_i);
		BaseHandle<Camera>* camera = dynamic_cast<BaseHandle<Camera>*>((_RTHandle*)camera_i);
		ConstHandle<Scene>* scene = dynamic_cast<ConstHandle<Scene>*>((_RTHandle*)scene_i);
		renderer->instance_->RenderFrame(camera->instance_, scene->instance_);
	}
private:
	void CalculateSize(Device::RTPrimitive primitive, size_t& num_triangles, size_t& num_vertices) {
		PrimitiveHandle* prim = dynamic_cast<PrimitiveHandle*>((_RTHandle*)primitive);
		if (prim->shape_) {
			num_vertices += prim->shape_->GetNumVertices();
			num_triangles += prim->shape_->GetNumTriangles();
		}
	}
	Box3f ExtractTriangles(Ref<Scene>& scene, 
							Device::RTPrimitive primitive, 
							size_t& num_meshes, 
							BuildTriangle* triangles,
							size_t num_triangles, 
							BuildVertex* vertices,
							size_t& num_vertices) {
		Box3f bounds;
		PrimitiveHandle* prim = dynamic_cast<PrimitiveHandle*>((_RTHandle*)primitive);
		if (!prim) throw std::runtime_error("invalid primitive");
		if (prim->shape_) {
			std::cerr << "Generating shape with transform" << std::endl;
			Ref<Shape> shape = prim->shape_->Transform(prim->transform_);
			std::cerr << "Adding primitive to the scene " << shape.ptr_ << " " << prim->material_.ptr_ << std::endl;
			Ref<Instance> instance = new Instance(num_meshes++, shape, prim->material_, nullptr);
			//size_t id = scene->Add(new Instance(num_meshes++, shape, prim->material_, nullptr));
			size_t id = scene->Add(instance);
			std::cerr << "Extracting..." << std::endl;
			bounds.ExtendBy(shape->Extract(id, triangles, num_triangles, vertices, num_vertices));
		}
		else throw std::runtime_error("invalid primitive");

		return bounds;
	}
};

// in viewer/handle.h
RenderDevice device;

template<typename Type>
class Handle {
public:
	Handle() = default;
	Handle(Type const input) : handle_(input) { std::cerr << "in here 1\n"; }
	Handle(const Handle& input) : handle_(input.handle_) { std::cerr << "in here 2\n"; if (handle_) device.rtIncRef(handle_); }
	~Handle() { if (handle_) device.rtDecRef(handle_); }
	Handle& operator=(const Handle& input) {
		std::cerr << "in here 3\n";
		if (input.handle_) device.rtIncRef(input.handle_);
		if (handle_) device.rtDecRef(handle_);
		handle_ = input.handle_;
		return *this;
	}
	//operator bool() const { return handle_ != nullptr; }
	operator Type() const { return handle_; }
//private:
	Type handle_{nullptr};
};

std::vector<Handle<Device::RTPrimitive>> prims;
Handle<Device::RTRenderer> renderer = nullptr;

void SetScene() {
	Handle<Device::RTShape> sphere = device.rtNewShape();
	std::cerr << "Before commit\n";
	device.rtCommit(sphere);
	std::cerr << "Commit done\n";
	Handle<Device::RTMaterial> material = device.rtNewMaterial();
	device.rtCommit(material);
	//prims.push_back(device.rtNewShapePrimitive(sphere));
	//Handle<Device::RTPrimitive> toto = device.rtNewShapePrimitive(sphere); // implicit type conversions, see Handle::operator()
	prims.push_back(device.rtNewShapePrimitive(sphere, material, nullptr));
	std::cerr << "counter for sphere once stored in prims " << ((_RTHandle*)(prims.back().handle_))->counter << std::endl;
}

void Render() {
	Handle<Device::RTRenderer> renderer = device.rtNewRenderer(); // pathtracer by default
	device.rtCommit(renderer);

	Handle<Device::RTScene> scene = device.rtNewScene((Device::RTPrimitive*)(prims.size() == 0 ? nullptr : &prims[0]), prims.size());
	
	int width = 640;
	int height = 480;
	Matrix44<float> space;
	space[3][3] = 13;
	Handle<Device::RTCamera> camera = device.rtNewCamera(); // pinhole by default for now 
	device.rtSetFloat1(camera, "fieldOfView", 60);
	device.rtSetFloat1(camera, "aspectRatio", width / float(height));
	device.rtSetTransform(camera, "local2world", space);
	device.rtCommit(camera);

	device.rtRenderFrame(renderer, camera, scene);
}

int main() {
	SetScene();
	Render();
	//std::cerr << ((_RTHandle*)(prims.back().handle_))->counter << std::endl;
	//Matrix44<float> m;
	//std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
	//std::unique_ptr<Shape> mesh_transform = mesh->Transform(m);
	return 0;
}