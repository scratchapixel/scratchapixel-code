// (c) www.scratchapixel.com - 2024.
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/

#ifndef _MATH_H_
#define _MATH_H_

#include <iomanip>

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
		if (len != T(0)) [[likely]] {
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
	constexpr T Dot(const Vec3& v) const noexcept {
		return x * v.x + y * v.y + z * v.z;
	}
	constexpr Vec3 operator-(const Vec3& v) const noexcept {
		return {x - v.x, y - v.y, z - v.z};
	}
	constexpr Vec3 operator+(const Vec3& v) const noexcept {
		return {x + v.x, y + v.y, z + v.z};
	}
	Vec3& operator+=(const Vec3& v) noexcept {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	template<typename S>
	Vec3& operator/=(const S a) noexcept {
		x /= a;
		y /= a;
		z /= a;
		return *this;
	}
	constexpr Vec3 operator*(T a) const noexcept {
		return {x * a, y * a, z * a};
	}
	constexpr Vec3 operator/(T a) const noexcept {
		return {x / a, y / a, z / a};
	}
	friend constexpr Vec3 operator*(T a, const Vec3& v) noexcept{ 
		return {a * v.x, a * v.y, a * v.z};
	}
	// @todo not safe
	friend constexpr Vec3 operator/(T a, const Vec3& v) noexcept{ 
		return {a / v.x, a / v.y, a / v.z};
	}
	constexpr T Max() const noexcept {
		return std::max(std::max(x, y), z);
	}
	constexpr T Length() const noexcept {
		return std::sqrt(x * x + y * y + z * z);
	}
	constexpr bool operator==(const Vec3& v) const noexcept {
		return x == v.x && y == v.y && z == v.z;
	}
	friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
		return os << v.x << " " << v.y << " " << v.z;
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
	static const Matrix44<T> kIdentity;
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
	constexpr Matrix44<T>& operator=(const Matrix44& v) noexcept {
		x[0][0] = v.x[0][0];
		x[0][1] = v.x[0][1];
		x[0][2] = v.x[0][2];
		x[0][3] = v.x[0][3];
		x[1][0] = v.x[1][0];
		x[1][1] = v.x[1][1];
		x[1][2] = v.x[1][2];
		x[1][3] = v.x[1][3];
		x[2][0] = v.x[2][0];
		x[2][1] = v.x[2][1];
		x[2][2] = v.x[2][2];
		x[2][3] = v.x[2][3];
		x[3][0] = v.x[3][0];
		x[3][1] = v.x[3][1];
		x[3][2] = v.x[3][2];
		x[3][3] = v.x[3][3];
		return *this;
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
	constexpr Matrix44<T> Transposed () const noexcept {
		return Matrix44 (
			x[0][0],
			x[1][0],
			x[2][0],
			x[3][0],
			x[0][1],
			x[1][1],
			x[2][1],
			x[3][1],
			x[0][2],
			x[1][2],
			x[2][2],
			x[3][2],
			x[0][3],
			x[1][3],
			x[2][3],
			x[3][3]);
	}
	constexpr const Matrix44<T>& Invert() noexcept {
		*this = Inverse();
		return *this;
	}
	constexpr Matrix44<T> Inverse() const noexcept {
		if (x[0][3] != 0 || x[1][3] != 0 || x[2][3] != 0 || x[3][3] != 1)
			abort();
			//	return gjInverse();

		Matrix44 s (
			x[1][1] * x[2][2] - x[2][1] * x[1][2],
			x[2][1] * x[0][2] - x[0][1] * x[2][2],
			x[0][1] * x[1][2] - x[1][1] * x[0][2],
			0,

			x[2][0] * x[1][2] - x[1][0] * x[2][2],
			x[0][0] * x[2][2] - x[2][0] * x[0][2],
			x[1][0] * x[0][2] - x[0][0] * x[1][2],
			0,

			x[1][0] * x[2][1] - x[2][0] * x[1][1],
			x[2][0] * x[0][1] - x[0][0] * x[2][1],
			x[0][0] * x[1][1] - x[1][0] * x[0][1],
			0,

			0,
			0,
			0,
			1);

		T r = x[0][0] * s.x[0][0] + x[0][1] * s.x[1][0] + x[0][2] * s.x[2][0];

		if (std::abs(r) >= 1) {
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					s.x[i][j] /= r;
				}
			}
		}
		else {
			T mr = std::abs (r) / std::numeric_limits<T>::min ();

			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					if (mr > std::abs (s.x[i][j])) {
						s.x[i][j] /= r;
					}
					else {
						return Matrix44 ();
					}
				}
			}
		}

		s.x[3][0] = -x[3][0] * s.x[0][0] - x[3][1] * s.x[1][0] - x[3][2] * s.x[2][0];
		s.x[3][1] = -x[3][0] * s.x[0][1] - x[3][1] * s.x[1][1] - x[3][2] * s.x[2][1];
		s.x[3][2] = -x[3][0] * s.x[0][2] - x[3][1] * s.x[1][2] - x[3][2] * s.x[2][2];

		return s;
	}
	friend std::ostream& operator<< (std::ostream& s, const Matrix44<T>& m) {
		std::ios_base::fmtflags oldFlags = s.flags();
		int width;

		if (s.flags() & std::ios_base::fixed) {
			s.setf(std::ios_base::showpoint);
			width = static_cast<int> (s.precision ()) + 5;
		}
		else {
			s.setf(std::ios_base::scientific);
			s.setf(std::ios_base::showpoint);
			width = static_cast<int> (s.precision ()) + 8;
		}

		s << "(" << std::setw(width) << m[0][0] << " " << std::setw(width)
		  << m[0][1] << " " << std::setw(width) << m[0][2] << " "
		  << std::setw(width) << m[0][3] << "\n"
		  <<

			" " << std::setw(width) << m[1][0] << " " << std::setw(width)
		  << m[1][1] << " " << std::setw(width) << m[1][2] << " "
		  << std::setw(width) << m[1][3] << "\n"
		  <<

			" " << std::setw(width) << m[2][0] << " " << std::setw(width)
		  << m[2][1] << " " << std::setw(width) << m[2][2] << " "
		  << std::setw(width) << m[2][3] << "\n"
		  <<

			" " << std::setw(width) << m[3][0] << " " << std::setw(width)
		  << m[3][1] << " " << std::setw(width) << m[3][2] << " "
		  << std::setw(width) << m[3][3] << ")\n";

		s.flags(oldFlags);
		return s;
	}
public:
	T x[4][4];
};

template<class T>
void ExtractScaling(const Matrix44<T>& mat, Vec3<T>& scale) {
	Vec3<T> row[3];

    row[0] = Vec3<T>(mat[0][0], mat[0][1], mat[0][2]);
    row[1] = Vec3<T>(mat[1][0], mat[1][1], mat[1][2]);
    row[2] = Vec3<T>(mat[2][0], mat[2][1], mat[2][2]);

	scale.x = row[0].Length();
	scale.y = row[1].Length();
	scale.z = row[2].Length();
}

template<typename T>
const Matrix44<T> Matrix44<T>::kIdentity = Matrix44<T>();

#endif