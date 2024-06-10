
#define _USE_MATH_DEFINES

#include <random>
#include <cmath>
#include <iostream>

struct Vec3 {
	float x, y, z;
	friend Vec3 operator*(float a, const Vec3& v) {
		return {v.x * a, v.y * a, v.z * a};
	}
};

Vec3 SampleUnitSphereNaive(float r1, float r2) {
	float theta = 2 * M_PI * r1; // azimuthal angle
	float phi = M_PI * r2; // polar angle
	return { 
		std::cos(theta) * std::sin(phi), 
		std::sin(theta) * std::sin(phi),
		std::cos(phi) };
}

int main() {
	float radius = 2.f;
	std::random_device rd; // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(0.0, 1.0); // Define the range
	for (size_t i = 0; i < 1024; ++i) {
		Vec3 sample = radius * SampleUnitSphereNaive(distr(gen), distr(gen));
		std::cout << "emit -o \"particleShape1\" -pos " << sample.x << " " << sample.z << " " << -sample.y << ";\n";
	}
	return 0;
};