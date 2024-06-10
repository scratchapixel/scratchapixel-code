#include <iostream>
#include <cmath>

int main() {
	float last_x = 0;
	int i = 0;
	for (float x = 0.1; x <= 1; x += 0.1) {
		float xx = 1 - x;
		std::cerr << "curve -d 1 -p 0 " << xx << " 0 -p 1 " << xx << " 0;//" << std::sqrt(x) << std::endl;
		float sqrt_x = std::sqrtf(x);
		if (x == 0.1f) {
			// calcualte triangle area
			//std::cerr << ++i << " area: " << sqrt_x * sqrt_x * 0.5 << std::endl; 
		}
		else {
			float height = sqrt_x - last_x;
			//std::cerr << ++i << " area: " << height * last_x + height * height * 0.5 << std::endl;
		}
		last_x = sqrt_x;
	}
	return 0;
}
