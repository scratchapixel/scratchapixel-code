// (c) www.scratchapixel.com - 2024.
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -Wall -Wextra -std=c++23 -o objimporter.exe objimporter.cc -O3

#define _USE_MATH_DEFINES

#include <cstdint>
#include <iostream>
#include <fstream>

#include <math.h>
#include <cmath>
#include <chrono>

#include <cassert>
#include <string>
#include <sstream>
#include <deque>

uint32_t image_width = 640;
uint32_t image_height = 480;

template<typename T>
class Vec3 {
public:
	Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
	Vec3(T x) : x(x), y(x), z(x) {}
	Vec3(T x, T y, T z) : x(x), y(y), z(z) {}
	Vec3& normalize() {
		T len = std::sqrt(x * x + y * y + z * z);
		x /= len, y /= len, z /= len;
		return *this;
	}
	constexpr T Dot(const Vec3<T>& v) const {
		return x * v.x + y * v.y + z * v.z;
	}
	constexpr Vec3<T> Cross(const Vec3<T>& v) const {
		return Vec3<T>(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x);
	}
	constexpr Vec3<T> operator-(const Vec3<T>& v) const {
		return Vec3<T>(x - v.x, y - v.y, z - v.z);
	}
	constexpr Vec3<T> operator*(const T& r) const {
		return Vec3<T>(x * r, y * r, z * r);
	}
	constexpr Vec3<T> operator+(const Vec3<T>& v) const {
		return Vec3<T>(x + v.x, y + v.y, z + v.z);
	}
	friend std::ostream& operator<<(std::ostream& os, const Vec3<T>& v) {
		return os << v.x << " " << v.y << " " << v.z;
	}
	T x, y, z;
};

template<typename T>
class Vec2 {
public:
	Vec2() : x(T(0)), y(T(0)) {}
	Vec2(T x ) : x(x), y(x) {}
	Vec2(T x, T y, T z) : x(x), y(y) {}
	T x, y;
	friend std::ostream& operator<<(std::ostream& os, const Vec2<T>& v) {
		return os << v.x << " " << v.y;
	}
};

using Vec2f = Vec2<float>;
using Vec3f = Vec3<float>;

struct FaceVertex {
	int vertex_index{-1};
	int st_coord_index{-1};
	int normal_index{-1};
	friend std::ostream& operator<<(std::ostream& os, const FaceVertex& fv) {
		return os << fv.vertex_index
				  << "/"
				  << (fv.st_coord_index != -1 ? std::to_string(fv.st_coord_index) : "") 
				  << "/" 
				  << (fv.normal_index != -1 ? std::to_string(fv.normal_index) : "");
	}
};

void ParseFaceVertex(const std::string& tuple, FaceVertex& face_vertex) {
	std::istringstream stream(tuple);
	std::string part;

	std::getline(stream, part, '/');
    assert(!part.empty());
	face_vertex.vertex_index = std::stoi(part) - 1;

    if (std::getline(stream, part, '/') && !part.empty()) {
        face_vertex.st_coord_index = std::stoi(part) - 1;
    }

    if (std::getline(stream, part, '/') && !part.empty()) {
        face_vertex.normal_index = std::stoi(part) - 1;
    }
}

void ProcessFace(const std::vector<std::string>& tuples, 
				 std::vector<FaceVertex>& face_vertices) {
	assert(tuples.size() == 3);
	for (const auto& tuple : tuples) {
		FaceVertex face_vertex;
		ParseFaceVertex(tuple, face_vertex);
		face_vertices.push_back(face_vertex);
	}
}

std::vector<Vec3f> vertices, normals;
std::vector<Vec2f> tex_coordinates;

struct FaceGroup {
	std::vector<FaceVertex> face_vertices;
	std::string name;
};

/**
 * Avoid using std::vector for storing elements when we need to maintain 
 * stable pointers or references to those elements across insertions or 
 * deletions. This is because the underlying storage of a std::vector may 
 * be reallocated as it grows, potentially invalidating existing pointers 
 * and references. To ensure that pointers and references remain valid 
 * despite container growth, use std::deque or std::list instead, as these 
 * containers provide stable references even when new elements are added 
 * or existing elements are removed.
 */
std::deque<FaceGroup> face_groups;

void ParseObj(const char* file) {
	std::ifstream ifs(file);
	std::string line;
	face_groups.emplace_back();
	FaceGroup* cur_face_group = &face_groups.back();
	while (std::getline(ifs, line)) {
		std::istringstream stream(line);
		std::string type;
		stream >> type;
		if (type == "v") {
			Vec3f v;
			stream >> v.x >> v.y >> v.z;
			vertices.push_back(v);
			if (cur_face_group->face_vertices.size() != 0) [[unlikely]] {
				face_groups.emplace_back();
				cur_face_group = &face_groups.back();
			}
		}
		else if (type == "vt") {
			Vec2f st;
			stream >> st.x >> st.y;
			tex_coordinates.push_back(st);
		}
		else if (type == "vn") {
			Vec3f n;
			stream >> n.x >> n.y >> n.z;
			normals.push_back(n);
		}
		else if (type  == "f") {
			std::vector<std::string> face;
			std::string tuple;
			while (stream >> tuple)
				face.push_back(tuple);
			ProcessFace(face, cur_face_group->face_vertices);
		}
		else if (type == "g") {
			if (cur_face_group->face_vertices.size() != 0) {
				face_groups.emplace_back();
				cur_face_group = &face_groups.back();
			}
			stream >> cur_face_group->name;
		}
	}
	std::cerr << face_groups.size() << std::endl;
	for (const auto& group : face_groups) {
		std::cerr << group.name << " " << group.face_vertices.size() / 3 << std::endl;
	}
	ifs.close();
}

template<typename T>
T DegreesToRadians(const T& degrees) {
	return M_PI * degrees / T(180);
}

float angle = 50.f;
constexpr float super_far = 1.e6;

struct Hit {
	bool hit{false};
	float t{super_far};
	float u, v;
};

inline float xorf(const float x, const float y) {
    std::uint32_t ix, iy;

    std::memcpy(&ix, &x, sizeof(float));
    std::memcpy(&iy, &y, sizeof(float));

    std::uint32_t resultInt = ix ^ iy;

    float result;
    std::memcpy(&result, &resultInt, sizeof(float));

    return result;
}

void intersect(const Vec3<float>& ray_orig, 
			   const Vec3<float>& ray_dir,
			   const Vec3<float>& p0, 
			   const Vec3<float>& p1, 
			   const Vec3<float>& p2, 
			   Hit& hit) {
	const float ray_near = 0.1;
	const Vec3<float> e1 = p0 - p1;
	const Vec3<float> e2 = p2 - p0;
	const Vec3<float> Ng = e1.Cross(e2);
	
	const Vec3<float> C = p0 - ray_orig;
	const Vec3<float> R = ray_dir.Cross(C);
	const float det = Ng.Dot(ray_dir);
	const float abs_det = std::abs(det);
	const float sign_det = std::copysign(0.f, det);
	if (det == 0) [[unlikely]] return;

	const float U = xorf(R.Dot(e2), sign_det);
	if (U < 0) [[likely]] return;
	
	const float V = xorf(R.Dot(e1), sign_det);
	if (V < 0) [[likely]] return;

	const float W = abs_det - U - V;
	if (W < 0) [[likely]] return;

	const float T = xorf(Ng.Dot(C), sign_det);
	if (T < abs_det * ray_near || abs_det * hit.t < T) [[unlikely]] return;

	const float rcp_abs_det = 1.f / abs_det;
	hit.u = U * rcp_abs_det;
	hit.v = V * rcp_abs_det;
	hit.t = T * rcp_abs_det;
}

void DoSomeWork() {
	auto start = std::chrono::high_resolution_clock::now();
	const Vec3<float> ray_orig(0,0,12);
	float aspect_ratio = image_width / static_cast<float>(image_height);
	float scale = std::tan(DegreesToRadians(0.5f * angle));
	auto buf = std::make_unique<uint8_t[]>(image_width * image_height);
	uint8_t* pbuf = buf.get();
	std::memset(pbuf, 0x0, image_width * image_height);
	for (uint32_t j = 0; j < image_height; ++j) {
		float y = (1 - 2 * (j + 0.5f) / static_cast<float>(image_height)) * scale * 1 / aspect_ratio;
		for (uint32_t i = 0; i < image_width; ++i, ++pbuf) {
			float x = (2 * (i + 0.5f) / static_cast<float>(image_width) - 1) * scale;
			Vec3f ray_dir(x, y, -1);
			ray_dir.normalize();
			float t = super_far;
			for (const auto& group : face_groups) {
				for (size_t n = 0; n < group.face_vertices.size() ; n += 3) {
					const Vec3f& v0 = vertices[group.face_vertices[n].vertex_index];
					const Vec3f& v1 = vertices[group.face_vertices[n + 1].vertex_index];
					const Vec3f& v2 = vertices[group.face_vertices[n + 2].vertex_index];
					Hit hit;
					intersect(ray_orig, ray_dir, v0, v1, v2, hit);
					if (hit.t < t) {
						t = hit.t;
						const Vec3f& n0 = normals[group.face_vertices[n].normal_index];
						const Vec3f& n1 = normals[group.face_vertices[n + 1].normal_index];
						const Vec3f& n2 = normals[group.face_vertices[n + 2].normal_index];
						Vec3f nor = n1 * hit.u + n2 * hit.v + n0 * (1 - (hit.u + hit.v));
						nor.normalize();
						*pbuf = static_cast<uint8_t>(255 * std::max(0.f, nor.z));
					}
				}
			}
		}
		fprintf(stderr, "\r%03u", static_cast<uint32_t>(j / static_cast<float>(image_height) * 100));
	}
	auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Render time: " << duration.count() / 1000.0 << " seconds." << std::endl;
	std::ofstream ofs("./result.ppm", std::ios::binary);
	ofs << "P6\n" << image_width << " " << image_height << "\n255\n";
	for (uint32_t i = 0; i < image_width * image_height; ++i)
		ofs << buf[i] << buf[i] <<  buf[i];
	ofs.close();
}

int main() {
	ParseObj("./zombie.obj");
	DoSomeWork();
	return 0;
}
