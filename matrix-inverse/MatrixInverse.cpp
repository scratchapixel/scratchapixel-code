// Open a Terminal (GitBash) and compile with:
// clang++ -std=c++20 -o MatrixInverse MatrixInverse.cpp
//
// Copyright (C) 2023  www.scratchapixel.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>

class Matrix4
{
public:
	Matrix4()
	{
		memset(&matrix[0][0], 0x0, sizeof(float) * 16);
		matrix[0][0] = matrix[1][1] = matrix[2][2] = matrix[3][3] = 1.f;
	}
	Matrix4(const Matrix4& m)
	{
		for (uint32_t j = 0; j < 4; ++j) {
			for (uint32_t i = 0; i < 4; ++i) {
				matrix[j][i] = m[j][i];
			}
		}
	}
	Matrix4(float a, float b, float c, float d, 
			float e, float f, float g, float h,
			float i, float j, float k, float l,
			float m, float n, float o, float p)
	{
		matrix[0][0] = a, matrix[0][1] = b, matrix[0][2] = c, matrix[0][3] = d,
		matrix[1][0] = e, matrix[1][1] = f, matrix[1][2] = g, matrix[1][3] = h,
		matrix[2][0] = i, matrix[2][1] = j, matrix[2][2] = k, matrix[2][3] = l,
		matrix[3][0] = m, matrix[3][1] = n, matrix[3][2] = o, matrix[3][3] = p;
	}
    float* operator[] (size_t i) { return &matrix[i][0]; };
    const float* operator[] (size_t i) const { return &matrix[i][0]; };
    Matrix4 Inverse() const;
    friend std::ostream& operator<<(std::ostream&, const Matrix4&);
   
    float matrix[4][4];
};

std::ostream& operator<<(std::ostream& os, const Matrix4& m)
{
	os << "[";
	for (uint32_t i = 0; i < 4; ++i) {
		if (i) os << ", ";
		os << "[";
		for (uint32_t j = 0; j < 4; ++j) {
			if (j) os << ", ";
			os << m[i][j];
		}
		os << "]";
	}
	os << "]";

	return os;
}

inline
Matrix4 Matrix4::Inverse() const
{
    Matrix4 s;
    Matrix4 t(*this);

    // Forward elimination
    for (uint32_t i = 0; i < 3; i++) {
		
		// Step 1: choose a pivot
        uint32_t pivot = i;

        float pivotsize = t[i][i];

        if (pivotsize < 0) pivotsize = -pivotsize;

        for (uint32_t j = i + 1; j < 4; j++) {
            float tmp = t[j][i];

            if (tmp < 0) tmp = -tmp;

            if (tmp > pivotsize) {
                pivot = j;
                pivotsize = tmp;
            }
        }

        if (pivotsize == 0) { return Matrix4(); }

        if (pivot != i) {
            for (uint32_t j = 0; j < 4; j++) {
                float tmp;

                tmp = t[i][j];
                t[i][j] = t[pivot][j];
                t[pivot][j] = tmp;

                tmp = s[i][j];
                s[i][j] = s[pivot][j];
                s[pivot][j] = tmp;
            }
        }

		// Step 2: eliminate all the numbers below the diagonal
        for (uint32_t j = i + 1; j < 4; j++) {
            float f = t[j][i] / t[i][i];

            for (uint32_t k = 0; k < 4; k++) {
                t[j][k] -= f * t[i][k];
                s[j][k] -= f * s[i][k];
            }
			// Set the column value to exactly 0 in case
			// numeric roundoff left it a very tiny number
			t[j][i] = 0.f;
        }
    }
	
	// Step 3: set elements along the diagonal to 1.0
	for (uint32_t i = 0; i < 4; i++) {
		float divisor = t[i][i];
		for (uint32_t j = 0; j < 4; j++) {
			t[i][j] = t[i][j] / divisor;
			s[i][j] = s[i][j] / divisor;
		}
		// set the diagonal to 1.0 exactly to avoid
		// possible round-off error
		t[i][i] = 1.f;
	}

	// Step 4: eliminate all the numbers above the diagonal
	for (uint32_t i = 0; i < 3; i++) {
		for (uint32_t j = i + 1; j < 4; j++) {
			float constant = t[i][j];
			for (uint32_t k = 0; k < 4; k++) {
				t[i][k] -= t[j][k] * constant;
				s[i][k] -= s[j][k] * constant;
			}
			t[i][j] = 0.f; // in case of round-off error
		}
	}

    return s;
}

int main()
{
	Matrix4 mat(0.832921, 0, 0.553392, 0, 0.291613, 0.849893, -0.438913, 0, -0.470323, 0.526956, 0.707894, 0, -2.574104, 3.650642, 4.868381, 1);
	std::cerr << mat << std::endl;
	Matrix4 inv = mat.Inverse();

	// expected result
	// 0.832921 0.291613 -0.470323 0 0 0.849893 0.526956 0 0.553392 -0.438913 0.707894 0 -0.550095 -0.215218 -6.580685 1
	std::cerr << inv << std::endl;
	return 1;
}