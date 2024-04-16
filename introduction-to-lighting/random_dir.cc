
#define _USE_MATH_DEFINES

#include <random>
#include <cmath>
#include <iostream>


int main() {
	std::random_device rd; // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(0.0, 1.0); // Define the range
	for (size_t i = 0; i < 16; ++i) {
		float theta = M_PI * 0.5 * distr(gen);
		float phi = 2 * M_PI * distr(gen);
		std::cerr << "curve -d 1 -p 0 0 0 -p " << 
			std::cos(phi) * sin(theta) << " " << 
			std::sin(theta) * std::sin(phi) << " " <<
			cos(theta) << " -k 0 -k 1;" << std::endl;
	}
	return 0;
};
