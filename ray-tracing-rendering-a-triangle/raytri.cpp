// Copyright (C) 2009-2024 www.scratchapixel.com
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -o raytri.exe raytri.cpp -O3 -std=c++23 (optional: -DMOLLER_TRUMBORE)

// needed on Windows
#define _USE_MATH_DEFINES

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <random>

#include "geometry.h"

constexpr float kEpsilon = 1e-8;

inline
float deg2rad(const float &deg)
{ return deg * M_PI / 180; }

inline
float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

bool rayTriangleIntersect(
    const Vec3f &orig, const Vec3f &dir,
    const Vec3f &v0, const Vec3f &v1, const Vec3f &v2,
    float &t, float &u, float &v)
{
#ifdef MOLLER_TRUMBORE
    Vec3f e0 = v0 - v2;
    Vec3f e1 = v1 - v2;
    Vec3f pvec = dir.crossProduct(e1);
    float det = e0.dotProduct(pvec);
#ifdef CULLING
    // if the determinant is negative the triangle is backfacing
    // if the determinant is close to 0, the ray misses the triangle
    if (det < kEpsilon) return false;
#else
    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;
#endif
    float invDet = 1 / det;

    Vec3f tvec = orig - v2;
    u = tvec.dotProduct(pvec) * invDet;
    if (u < 0 || u > 1) return false;

    Vec3f qvec = tvec.crossProduct(e0);
    v = dir.dotProduct(qvec) * invDet;
    if (v < 0 || u + v > 1) return false;
    
    t = e1.dotProduct(qvec) * invDet;
    
    return true;
#else
    // compute plane's normal
    Vec3f e0 = v2 - v1;
    Vec3f e1 = v0 - v2;
    // no need to normalize
    Vec3f N = e0.crossProduct(e1); // N
    float denom = N.dotProduct(N);
    
    // Step 1: finding P
    
    // check if ray and plane are parallel ?
    float NdotRayDirection = N.dotProduct(dir);

    if (fabs(NdotRayDirection) < kEpsilon) // almost 0
        return false; // they are parallel so they don't intersect ! 

    // compute d parameter using equation 2
    float d = -N.dotProduct(v0);
    
    // compute t (equation 3)
    t = -(N.dotProduct(orig) + d) / NdotRayDirection;
    
    // check if the triangle is in behind the ray
    if (t < 0) return false; // the triangle is behind
 
    // compute the intersection point using equation 1
    Vec3f P = orig + t * dir;
 
    // Step 2: inside-outside test
    Vec3f C; // vector perpendicular to triangle's plane
 
	// Calculate u (for triangle BCP)
    Vec3f v1p = P - v1;
    C = e0.crossProduct(v1p);
	if ((u = N.dotProduct(C)) < 0) return false; // P is on the right side
 
    // Calculate v (for triangle CAP)
    Vec3f v2p = P - v2;
    C = e1.crossProduct(v2p);
    if ((v = N.dotProduct(C)) < 0) return false; // P is on the right side

    Vec3f e2 = v1 - v0;
    Vec3f v0p = P - v0;
    C = e2.crossProduct(v0p);
    if (N.dotProduct(C) < 0) return false; // P is on the right side

    u /= denom;
    v /= denom;

    return true; // this ray hits the triangle
#endif
}

int main(int argc, char **argv)
{
    Vec3f v0(-2, -1, -5);
    Vec3f v1( 1, -1, -5);
    Vec3f v2( 0,  1, -5);
    
    const uint32_t width = 640;
    const uint32_t height = 480;
	// v0 is yellow, v1 is cyan and v2 is magenta
    Vec3f cols[3] = {{1.f, 1.f, 0.f}, {0, 1.f, 1.f}, {1.f, 0.f, 1.f}};
    Vec3f *framebuffer = new Vec3f[width * height];
    Vec3f *pix = framebuffer;
    float fov = 51.52;
    float scale = tan(deg2rad(fov * 0.5));
    float imageAspectRatio = width / (float)height;
    Vec3f orig(0);
    for (uint32_t j = 0; j < height; ++j) {
        for (uint32_t i = 0; i < width; ++i) {
            // compute primary ray
            float x = (2 * (i + 0.5) / (float)width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)height) * scale;
            Vec3f dir(x, y, -1);
            dir.normalize();
            float t, u, v;
            if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v)) {
                *pix = u * cols[0] + v * cols[1] + (1 - u - v) * cols[2];
                // uncomment this line if you want to visualize the row barycentric coordinates
                *pix = Vec3f(u, v, 1 - u - v);
            }
            pix++;
        }
    }
    
    // Save result to a PPM image (keep these flags if you compile under Windows)
    std::ofstream ofs("./out.ppm", std::ios::out | std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (uint32_t i = 0; i < height * width; ++i) {
        char r = (char)(255 * clamp(0, 1, framebuffer[i].x));
        char g = (char)(255 * clamp(0, 1, framebuffer[i].y));
        char b = (char)(255 * clamp(0, 1, framebuffer[i].z));
        ofs << r << g << b;
    }

    ofs.close();

    delete [] framebuffer;

    return 0;
}
