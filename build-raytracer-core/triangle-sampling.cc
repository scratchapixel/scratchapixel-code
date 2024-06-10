#include <cstdint>
#include <random>
#include <iostream>

template<typename T>
class Vec2 {
public:
	Vec2() : x(T(0)), y(T(0)) {}
	Vec2(T xx) : x(xx), y(xx) {}
	Vec2(T xx, T yy) : x(xx), y(yy) {}
	Vec2 operator+(const Vec2 v) const noexcept {
		return {x + v.x, y + v.y};
	}
	friend Vec2 operator*(T a, const Vec2& v) {
		return {a * v.x, a * v.y};
	}
	friend std::ostream& operator<<(std::ostream& os, const Vec2& v) {
		return os << v.x << " " << v.y;
	}
	T x, y;
};

using Vec2f = Vec2<float>;

constexpr uint32_t nsamples = 512;

Vec2f NaiveMethod(const Vec2f& v1, const Vec2f& v2, float u, float v) {
	return u * v1 + (1.f - u) * v * v2;
}

Vec2f BarycentricCoordinatesMethodBad(const Vec2f& v1, const Vec2f& v2, float u, float v, float w) {
	float sum = u + v + w; // we need to normalize the barycentric coordinates
	return /* (u / sum) * v0 + */ (v / sum) * v1 + (w / sum) * v2;
}

Vec2f BarycentricCoordinatesMethodGood(const Vec2f& v1, const Vec2f& v2, float r1, float r2) {
	float q = std::sqrt(r1);
	// v0 = (0,0) so skip calculation
	return /* (1 - q) * v0 + */ q * (1 - r2) * v1 + q * r2 * v2;
}

Vec2f KraemerMethod(const Vec2f& v1, const Vec2f& v2, float r1, float r2) {
#if 1
	if (r1 > r2) std::swap(r1, r2);
	float u = r1, v = r2 - r1, w = 1 - r2;
#else
    float q = std::abs(r1 - r2);
    float u = q, v = 0.5f * (r1 + r2 - q), w = 1 - 0.5 * (q + r1 + r2);
#endif
    return /* u * v0 + */ v * v1 + w * v2;
}

template<typename  SamplingMethod>
void SampleTriangle(SamplingMethod sample) {
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_real_distribution<float> distr(0.f, 1.f);
	for (size_t i = 0; i < nsamples; ++i) {
		// We assume v0 = (0,0)
		Vec2f x = sample({2,0}, {1,2}, distr(eng), distr(eng));
		std::cout << "emit -object \"particleShape1\" -pos " << x << " 0;\n"; 
	}
}

int main() {
	SampleTriangle(BarycentricCoordinatesMethodGood);
};