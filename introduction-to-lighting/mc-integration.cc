#include <iostream>
#include <random>
#include <cmath>
#include <sstream>

template<typename T>
class Vec3 {
public:
	Vec3() : x(0), y(0), z(0) {}
	Vec3(T xx) : x(xx), y(xx), z(xx) {}
	Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
	T Dot(const Vec3& v) const noexcept {
		return x * v.x + y * v.y + z * v.z;
	}
	Vec3 operator*(const Vec3& v) const noexcept {
		return {x * v.x, y * v.y, z * v.z};
	}
	Vec3 operator*(T r) const noexcept {
		return {x * r, y * r, z * r};
	}
	template<typename S>
	Vec3& operator/=(S r) noexcept {
		x /= r;
		y /= r;
		z /= r;
		return *this;
	}
	template<typename S>
	Vec3 operator/(S r) const noexcept {
		return {x / r, y / r, z / r};
	}
	Vec3 operator-(const Vec3& v) const noexcept {
		return {x - v.x, y - v.y, z - v.z};
	}
	T Length() const noexcept {
		return std::sqrt(x * x + y * y + z * z);
	}
	Vec3& Normalize() noexcept {
		T len = Length();
		if (len != 0) [[likely]]
			x /= len, y /= len, z /= len;
		return *this;
	}
	Vec3& operator+=(const Vec3& v) noexcept {
		x += v.x, y += v.y, z += v.z;
		return *this;
	}
	friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
		return os << v.x << " " << v.y << " " << v.z;
	}
	T x, y, z;
};

template<typename T>
class Vec2 {
public:
	Vec2() : x(0), y(0) {}
	T x, y;	
};

Vec3<double> Le(Vec3<double> x) {
	return (x.x < -0.4) ? 1 : 0.001;
}

int num_samples = 16;

void example1() {
	Vec3<double> sum = 0;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dist(-0.5,0.5);
	Vec3<double> x(0,0,5);
	Vec3<double> Nf(0,0,-1);
	Vec3<double> Nl(0,0,1);
	for (int n = 0; n < num_samples; ++n) {
		Vec3<double> sample = {dist(gen), dist(gen), 0};
		Vec3<double> d = sample - x;
		double r = d.Length();
		d.Normalize();
		//std::cerr << "n: " << n << ", sample: " << sample << ", Le(sample): " << Le(sample) << ", Nf.Dot(d): " << Nf.Dot(d) << std::endl;
		std::cerr << "emit -o \"part2\" -pos " << sample << ";\n";
		sum += Le(sample) * Nf.Dot(d);
	}
	sum /= num_samples;
	std::cerr << "Example 1: " << sum << std::endl;
}

void example2() {
	Vec3<double> sum = 0;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dist(0, 1);
	Vec3<double> x(0,0,5);
	Vec3<double> Nf(0,0,-1);
	Vec3<double> Nl(0,0,1);
	for (int n = 0; n < num_samples; ++n) {
		double rand = dist(gen);
		Vec3<double> sample;
		double pdf;
		if (dist(gen) <= 0.8) {
			std::uniform_real_distribution<double> dist_sample(-0.5, -0.4);		
			sample = {dist_sample(gen), (2 * dist(gen) - 1) * 0.5, 0};
			pdf = 0.8 / 0.1; // probability / segment length
		}
		else {
			std::uniform_real_distribution<double> dist_sample(-0.4, 0.5);
			sample = {dist_sample(gen), (2 * dist(gen) - 1) * 0.5, 0};
			pdf = 0.2 / 0.9; // probability / segment length
		}
		Vec3<double> d = sample - x;
		double r = d.Length();
		d.Normalize();
		//std::cerr << "n: " << n << ", sample: " << sample << ", Le(sample): " << Le(sample) << ", Nf.Dot(d): " << Nf.Dot(d) << std::endl;
		//std::cerr << "emit -o \"part2\" -pos " << sample << ";\n";
		sum += Le(sample) * Nf.Dot(d) / pdf;
	}
	sum /= num_samples;
	std::cerr << "Example 2: " << sum << std::endl;
}

int main() {
	example1();
	example2();
	return 0;
}
