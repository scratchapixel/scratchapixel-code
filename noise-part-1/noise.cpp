//[header]
// A simple program to demonstrate the concept of value noise
//[/header]
//[compile]
// Download the noise.cpp file to a folder.
// Open a shell/terminal, and run the following command where the file is saved:
//
// c++ -o noise noise.cpp -std=c++11 -O3
//
// Run with: ./noise. Open the file ./noise.ppm in Photoshop or any program
// reading PPM files.
//[/compile]
//[ignore]
// Copyright (C) 2012  www.scratchapixel.com
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
//[/ignore]

// if you use windows uncomment these two lines
//#include "stdafx.h"
//#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdio>
#include <random>
#include <functional>
#include <iostream>
#include <fstream>


template<typename T>
class Vec2
{
public:
    Vec2() : x(T(0)), y(T(0)) {}
    Vec2(T xx, T yy) : x(xx), y(yy) {}
    Vec2 operator * (const T &r) const { return Vec2(x * r, y * r); }
    Vec2& operator *= (const T &r) { x *= r, y *= r; return *this; }
    T x, y;
};

typedef Vec2<float> Vec2f;

// [comment]
// Linear interpolation
// [/comment]
template<typename T = float>
inline T lerp(const T &lo, const T &hi, const T &t)
{
    return lo * (1 - t) + hi * t;
}

// [comment]
// The smoothstep function
// [/comment]
inline
float smoothstep(const float &t)
{
    return t * t * (3 - 2 * t);
}

class ValueNoise
{
public:
    ValueNoise(unsigned seed = 2016)
    {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<float> distrFloat;
        auto randFloat = std::bind(distrFloat, gen);

        // create an array of random values and initialize permutation table
        for (unsigned k = 0; k < kMaxTableSize; ++k) {
            r[k] = randFloat();
            permutationTable[k] = k;
        }

        // shuffle values of the permutation table
        std::uniform_int_distribution<unsigned> distrUInt;
        auto randUInt = std::bind(distrUInt, gen);
        for (unsigned k = 0; k < kMaxTableSize; ++k) {
            unsigned i = randUInt() & kMaxTableSizeMask;
            std::swap(permutationTable[k], permutationTable[i]);
            permutationTable[k + kMaxTableSize] = permutationTable[k];
        }
    }

    float eval(Vec2f &p) const
    {
        int xi = std::floor(p.x);
        int yi = std::floor(p.y);

        float tx = p.x - xi;
        float ty = p.y - yi;

        int rx0 = xi & kMaxTableSizeMask;
        int rx1 = (rx0 + 1) & kMaxTableSizeMask;
        int ry0 = yi & kMaxTableSizeMask;
        int ry1 = (ry0 + 1) & kMaxTableSizeMask;

        // random values at the corners of the cell using permutation table
        const float & c00 = r[permutationTable[permutationTable[rx0] + ry0]];
        const float & c10 = r[permutationTable[permutationTable[rx1] + ry0]];
        const float & c01 = r[permutationTable[permutationTable[rx0] + ry1]];
        const float & c11 = r[permutationTable[permutationTable[rx1] + ry1]];

        // remapping of tx and ty using the Smoothstep function 
        float sx = smoothstep(tx);
        float sy = smoothstep(ty);

        // linearly interpolate values along the x axis
        float nx0 = lerp(c00, c10, sx);
        float nx1 = lerp(c01, c11, sx);

        // linearly interpolate the nx0/nx1 along they y axis
        return lerp(nx0, nx1, sy);
    }
    static const unsigned kMaxTableSize = 256;
    static const unsigned kMaxTableSizeMask = kMaxTableSize - 1;
    float r[kMaxTableSize];
    unsigned permutationTable[kMaxTableSize * 2];
};

int main(int argc, char **argv)
{
    unsigned imageWidth = 512;
    unsigned imageHeight = 512;
    float *noiseMap = new float[imageWidth * imageHeight]{ 0 };
#if 0
    // [comment]
    // Generate white noise
    // [/comment]
    unsigned seed = 2016;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> distr;
    auto dice = std::bind(distr, gen); // std::function<float()>

    for (unsigned j = 0; j < imageHeight; ++j) {
        for (unsigned i = 0; i < imageWidth; ++i) {
            // generate a float in the range [0:1]
            noiseMap[j * imageWidth + i] = dice();
        }
    }
#elif 0
    // [comment]
    // Generate value noise
    // [/comment]
    ValueNoise noise;
    float frequency = 0.05f;
    for (unsigned j = 0; j < imageHeight; ++j) {
        for (unsigned i = 0; i < imageWidth; ++i) {
            // generate a float in the range [0:1]
            noiseMap[j * imageWidth + i] = noise.eval(Vec2f(i, j) * frequency);
        }
    }
#elif 0
    // [comment]
    // Generate fractal pattern
    // [/comment]
    ValueNoise noise;
    float frequency = 0.02f;
    float frequencyMult = 1.8;
    float amplitudeMult = 0.35;
    unsigned numLayers = 5;
    float maxNoiseVal = 0;
    for (unsigned j = 0; j < imageHeight; ++j) {
        for (unsigned i = 0; i < imageWidth; ++i) {
            Vec2f pNoise = Vec2f(i, j) * frequency;
            float amplitude = 1;
            for (unsigned l = 0; l < numLayers; ++l) {
                noiseMap[j * imageWidth + i] += noise.eval(pNoise) * amplitude;
                pNoise *= frequencyMult;
                amplitude *= amplitudeMult;
            }
            if (noiseMap[j * imageWidth + i] > maxNoiseVal) maxNoiseVal = noiseMap[j * imageWidth + i];
        }
    }
    for (unsigned i = 0; i < imageWidth * imageHeight; ++i) noiseMap[i] /= maxNoiseVal;
#elif 0
    // [comment]
    // Generate turbulence pattern
    // [/comment]
    ValueNoise noise;
    float frequency = 0.02f;
    float frequencyMult = 1.8;
    float amplitudeMult = 0.35;
    unsigned numLayers = 5;
    float maxNoiseVal = 0;
    for (unsigned j = 0; j < imageHeight; ++j) {
        for (unsigned i = 0; i < imageWidth; ++i) {
            Vec2f pNoise = Vec2f(i, j) * frequency;
            float amplitude = 1;
            for (unsigned l = 0; l < numLayers; ++l) {
                noiseMap[j * imageWidth + i] += std::fabs(2 * noise.eval(pNoise) - 1) * amplitude;
                pNoise *= frequencyMult;
                amplitude *= amplitudeMult;
            }
            if (noiseMap[j * imageWidth + i] > maxNoiseVal) maxNoiseVal = noiseMap[j * imageWidth + i];
        }
    }
    for (unsigned i = 0; i < imageWidth * imageHeight; ++i) noiseMap[i] /= maxNoiseVal;
#elif 0
    // [comment]
    // Generate marble pattern
    // [/comment]
    ValueNoise noise;
    float frequency = 0.02f;
    float frequencyMult = 1.8;
    float amplitudeMult = 0.35;
    unsigned numLayers = 5;
    for (unsigned j = 0; j < imageHeight; ++j) {
        for (unsigned i = 0; i < imageWidth; ++i) {
            Vec2f pNoise = Vec2f(i, j) * frequency;
            float amplitude = 1;
            float noiseValue = 0;
            // compute some fractal noise
            for (unsigned l = 0; l < numLayers; ++l) {
                noiseValue += noise.eval(pNoise) * amplitude;
                pNoise *= frequencyMult;
                amplitude *= amplitudeMult;
            }
            // we "displace" the value i used in the sin() expression by noiseValue * 100
            noiseMap[j * imageWidth + i] = (sin((i + noiseValue * 100) * 2 * M_PI / 200.f) + 1) / 2.f;
        }
    }
#else 1
    // [comment]
    // Generate wood pattern
    // [/comment]
    ValueNoise noise;
    float frequency = 0.01f;
    for (unsigned j = 0; j < imageHeight; ++j) {
        for (unsigned i = 0; i < imageWidth; ++i) {
            float g = noise.eval(Vec2f(i, j) * frequency) * 10;
            noiseMap[j * imageWidth + i] = g - (int)g;
        }
    }
#endif

    // output noise map to PPM
    std::ofstream ofs;
    ofs.open("./noise.ppm", std::ios::out | std::ios::binary);
    ofs << "P6\n" << imageWidth << " " << imageHeight << "\n255\n";
    for (unsigned k = 0; k < imageWidth * imageHeight; ++k) {
        unsigned char n = static_cast<unsigned char>(noiseMap[k] * 255);
        ofs << n << n << n;
    }
    ofs.close();

    delete[] noiseMap;

    return 0;
}
