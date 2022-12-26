//[header]
// Rendering volumetric object using ray-marching. A basic implementation (chapter 1 & 2)
//
// https://www.scratchapixel.com/lessons/advanced-rendering/volume-rendering-for-developers/ray-marching-algorithm
//[/header]
//[compile]
// Download the raymarch-chap2.cpp file to a folder.
// Open a shell/terminal, and run the following command where the file is saved:
//
// clang++ -O3 raymarch-chap2.cpp -o render -std=c++17 (optional: -DBACKWARD_RAYMARCHING)
//
// You can use c++ if you don't use clang++
//
// Run with: ./render. Open the resulting image (ppm) in Photoshop or any program
// reading PPM files.
//[/compile]
//[ignore]
// Copyright (C) 2022  www.scratchapixel.com
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

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <fstream>
#include <memory>
#include <algorithm>
#include <vector>
#include <random>

struct vec3
{
    float x{ 0 }, y{ 0 }, z{ 0 };
    vec3& nor()
    {
        float len = x * x + y * y + z * z;
        if (len != 0) len = sqrtf(len);
        x /= len, y /= len, z /= len;
        return *this;
    }
    float length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }
    float operator * (const vec3& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    vec3 operator - (const vec3& v) const
    {
        return vec3{x - v.x, y - v.y, z - v.z};
    }
    vec3 operator + (const vec3& v) const
    {
        return vec3{x + v.x, y + v.y, z + v.z};
    }
    vec3& operator += (const vec3& v)
    {
        x += v.x, y += v.y, z += v.z;
        return *this;
    }
    vec3& operator *= (const float& r)
    {
        x *= r, y *= r, z *= r;
        return *this;
    }
    friend vec3 operator * (const float& r, const vec3& v)
    {
        return vec3{v.x * r, v.y * r, v.z * r};
    }
    friend std::ostream& operator << (std::ostream& os, const vec3& v)
    {
        os << v.x << " " << v.y << " " << v.z;
        return os;
    }
    vec3 operator * (const float &r) const
    {
        return vec3{ x * r, y * r, z * r };
    }
};

constexpr vec3 background_color{ 0.572f, 0.772f, 0.921f };
constexpr float floatMax = std::numeric_limits<float>::max();

struct IsectData
{
    float t0{ floatMax }, t1{ floatMax };
    vec3 pHit;
    vec3 nHit;
    bool inside{ false };
};

struct Object
{
public:
    vec3 color;
    int type{ 0 };
    virtual bool intersect(const vec3&, const vec3&, IsectData&) const = 0;
    virtual ~Object() {}
    Object() {}
};

bool solveQuadratic(float a, float b, float c, float& r0, float& r1)
{
    float d = b * b - 4 * a * c;
    if (d < 0) return false;
    else if (d == 0) r0 = r1 = -0.5f * b / a;
    else {
        float q = (b > 0) ? -0.5f * (b + sqrtf(d)) : -0.5f * (b - sqrtf(d));
        r0 = q / a;
        r1 = c / q;
    }

    if (r0 > r1) std::swap(r0, r1);

    return true;
}

struct Sphere : Object
{
public:
    Sphere() { color = vec3{ 1, 0, 0 }; type = 1; }
    bool intersect(const vec3& rayOrig, const vec3& rayDir, IsectData& isect) const override
    {
        vec3 rayOrigc = rayOrig - center;
        float a = rayDir * rayDir;
        float b = 2 * (rayDir * rayOrigc);
        float c = rayOrigc * rayOrigc - radius * radius;

        if (!solveQuadratic(a, b, c, isect.t0, isect.t1)) return false;

        if (isect.t0 < 0) {
            if (isect.t1 < 0) return false;
            else {
                isect.inside = true;
                isect.t0 = 0;
            }
        }

        return true;
    }

    float radius{ 1 };
    vec3 center{ 0, 0, -4 };
};

std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(0.0, 1.0);

vec3 integrate(const vec3& ray_orig, const vec3& ray_dir, const std::vector<std::unique_ptr<Object>> &objects)
{
    const Object* hit_object = nullptr;
    IsectData isect;
    for (const auto& object : objects) {
        IsectData isect_object;
        if (object->intersect(ray_orig, ray_dir, isect_object)) {
            hit_object = object.get();
            isect = isect_object;
        }
    }
	
    if (!hit_object)
        return background_color;

    float step_size = 0.2;
    float absorption = 0.1;
    float scattering = 0.1;
    float density = 1;
    int ns = std::ceil((isect.t1 - isect.t0) / step_size);
    step_size = (isect.t1 - isect.t0) / ns;

    vec3 light_dir{ 0, 1, 0 };
    vec3 light_color{ 1.3, 0.3, 0.9 };
    IsectData isect_vol;

    float transparency = 1; // initialize transmission to 1 (fully transparent)
    vec3 result{ 0 }; // initialize volumetric sphere color to 0

#ifdef BACKWARD_RAYMARCHING
    // [comment]
    // The ray-marching loop (backward, march from t1 to t0)
    // [/comment]
    for (int n = 0; n < ns; ++n) {
        float t = isect.t1 - step_size * (n + 0.5);
        vec3 sample_pos = ray_orig + t * ray_dir;

        float sample_transparency = exp(-step_size * (scattering + absorption));
        transparency *= sample_transparency;

        if (hit_object->intersect(sample_pos, light_dir, isect_vol) && isect_vol.inside) {
            float light_attenuation = exp(-density * isect_vol.t1 * (scattering + absorption));
            result += light_color * light_attenuation * scattering * density * step_size;
        }
        else
            std::cerr << "oops\n";

        result *= sample_transparency;
    }

    return background_color * transparency + result;
#else
	// [comment]
	// The ray-marching loop (forward, march from t0 to t1)
	// [/comment]
    for (int n = 0; n < ns; ++n) {
        float t = isect.t0 + step_size * (n + 0.5);
        vec3 sample_pos = ray_orig + t * ray_dir;

        // compute sample transmission
        float sample_attenuation = exp(-step_size * (scattering + absorption));
        transparency *= sample_attenuation;

        // In-scattering. Find distance light travels through volumetric sphere to the sample.
        // Then use Beer's law to attenuate the light contribution due to in-scattering.
        if (hit_object->intersect(sample_pos, light_dir, isect_vol) && isect_vol.inside) {
            float light_attenuation = exp(-density * isect_vol.t1 * (scattering + absorption));
            result += transparency * light_color * light_attenuation * scattering * density * step_size;
        }
        else
            std::cerr << "oops\n";
    }

    // combine background color and volumetric sphere color
    return background_color * transparency + result;
#endif
}

int main()
{
    unsigned int width = 640, height = 480;

    auto buffer = std::make_unique<unsigned char[]>(width * height * 3);

    auto frameAspectRatio = width / float(height);
    float fov = 45;
    float focal = tan(M_PI / 180 * fov * 0.5);

    std::vector<std::unique_ptr<Object>> geo;
    std::unique_ptr<Sphere> sph = std::make_unique<Sphere>();
    sph->radius = 5;
    sph->center.x = 0;
    sph->center.y = 0;
    sph->center.z = -20;
    geo.push_back(std::move(sph));

    vec3 rayOrig, rayDir; // ray origin & direction

    unsigned int offset = 0;
    for (unsigned int j = 0; j < height; ++j) {
        for (unsigned int i = 0; i < width; ++i) {
            rayDir.x = (2.f * (i + 0.5f) / width - 1) * focal;
            rayDir.y = (1 - 2.f * (j + 0.5f) / height) * focal * 1 / frameAspectRatio; // Maya style
            rayDir.z = -1.f;

            rayDir.nor();

            vec3 c = integrate(rayOrig, rayDir, geo);

            buffer[offset++] = std::clamp(c.x, 0.f, 1.f) * 255;
            buffer[offset++] = std::clamp(c.y, 0.f, 1.f) * 255;
            buffer[offset++] = std::clamp(c.z, 0.f, 1.f) * 255;
        }
    }

    // writing file
    std::ofstream ofs;
    ofs.open("./image.ppm", std::ios::binary);
    ofs << "P6\n" << width << " " <<  height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(buffer.get()), width * height * 3);
    ofs.close();

    return 0;
}
