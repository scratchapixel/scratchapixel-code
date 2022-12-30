//[header]
// Rendering volumetric object using ray-marching. A basic implementation (chapter 1 & 2)
//
// https://www.scratchapixel.com/lessons/advanced-rendering/volume-rendering-for-developers/ray-marching-algorithm
//[/header]
//[compile]
// Download the raymarch-chap4.cpp file to a folder.
// Open a shell/terminal, and run the following command where the file is saved:
//
// clang++ -O3 raymarch-chap4.cpp -o render -std=c++17
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
#include <cassert>

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
    // dot product
    float operator * (const vec3& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    vec3 operator - (const vec3& v) const
    {
        return vec3{ x - v.x, y - v.y, z - v.z };
    }
    vec3 operator - () const
    {
        return vec3{ -x, -y, -z };
    }
    vec3 operator + (const vec3& v) const
    {
        return vec3{ x + v.x, y + v.y, z + v.z };
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
        return vec3{ v.x * r, v.y * r, v.z * r };
    }
    friend std::ostream& operator << (std::ostream& os, const vec3& v)
    {
        os << v.x << " " << v.y << " " << v.z;
        return os;
    }
    vec3 operator * (const float& r) const
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

struct Sphere
{
public:
    Sphere() { color = vec3{ 1, 0, 0 }; }
    bool intersect(const vec3& rayOrig, const vec3& rayDir, IsectData& isect) const
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
    vec3 color;
    float radius{ 1 };
    vec3 center{ 0, 0, -4 };
};

std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(0.0, 1.0);

int permutation[256] = {
    151, 160, 137,  91,  90,  15, 131,  13, 201,  95,  96,  53, 194, 233,   7, 225,
    140,  36, 103,  30,  69, 142,   8,  99,  37, 240,  21,  10,  23, 190,   6, 148,
    247, 120, 234,  75,   0,  26, 197,  62,  94, 252, 219, 203, 117,  35,  11,  32,
     57, 177,  33,  88, 237, 149,  56,  87, 174,  20, 125, 136, 171, 168,  68, 175,
     74, 165,  71, 134, 139,  48,  27, 166,  77, 146, 158, 231,  83, 111, 229, 122,
     60, 211, 133, 230, 220, 105,  92,  41,  55,  46, 245,  40, 244, 102, 143,  54,
     65,  25,  63, 161,   1, 216,  80,  73, 209,  76, 132, 187, 208,  89,  18, 169,
    200, 196, 135, 130, 116, 188, 159,  86, 164, 100, 109, 198, 173, 186,   3,  64,
     52, 217, 226, 250, 124, 123,   5, 202,  38, 147, 118, 126, 255,  82,  85, 212,
    207, 206,  59, 227,  47,  16,  58,  17, 182, 189,  28,  42, 223, 183, 170, 213,
    119, 248, 152,   2,  44, 154, 163,  70, 221, 153, 101, 155, 167,  43, 172,   9,
    129,  22,  39, 253,  19,  98, 108, 110,  79, 113, 224, 232, 178, 185, 112, 104,
    218, 246,  97, 228, 251,  34, 242, 193, 238, 210, 144,  12, 191, 179, 162, 241,
     81,  51, 145, 235, 249,  14, 239, 107,  49, 192, 214,  31, 181, 199, 106, 157,
    184,  84, 204, 176, 115, 121,  50,  45, 127,   4, 150, 254, 138, 236, 205,  93,
    222, 114,  67,  29,  24,  72, 243, 141, 128, 195,  78,  66, 215,  61, 156, 180
};

int p[512];

float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
float lerp(float t, float a, float b) { return a + t * (b - a); }
float grad(int hash, float x, float y, float z)
{
    int h = hash & 15;
    float u = h < 8 ? x : y,
          v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

// [comment]
// We use a Perlin noise procedural texture to create a space varying density field.
// This function returns values in the range [-1,1].
// [/comment]
float noise(float x, float y, float z)
{
    int X = static_cast<int>(std::floor(x)) & 255,
        Y = static_cast<int>(std::floor(y)) & 255,
        Z = static_cast<int>(std::floor(z)) & 255;
    x -= std::floor(x);
    y -= std::floor(y);
    z -= std::floor(z);
    float u = fade(x),
          v = fade(y),
          w = fade(z);
    int A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z,
        B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;

    return lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z),
                                   grad(p[BA], x - 1, y, z)),
                           lerp(u, grad(p[AB], x, y - 1, z),
                                   grad(p[BB], x - 1, y - 1, z))),
                   lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1),
                                   grad(p[BA + 1], x - 1, y, z - 1)),
                           lerp(u, grad(p[AB + 1], x, y - 1, z - 1),
                                   grad(p[BB + 1], x - 1, y - 1, z - 1))));
}

// The Henyey-Greenstein phase function
float phaseHG(const vec3& view_dir, const vec3& light_dir, const float& g)
{
    float cos_theta = view_dir * light_dir;
    return 1 / (4 * M_PI) * (1 - g * g) / powf(1 + g * g - 2 * g * cos_theta, 1.5);
}

float smoothstep(float lo, float hi, float x)
{
    float t = std::clamp((x - lo) / (hi - lo), 0.f, 1.f);
    return t * t * (3.0 - (2.0 * t));
}

size_t frame = 0;

// [comment]
// Return the density of our volume object at position p. Uses a Perlin noise procedural
// texture to create this space varying density field. We need to remap the noise() function
// to the range [0,1].
// [/comment]
float eval_density(const vec3& p, const vec3& center, const float& radius)
{ 
    vec3 vp = p - center;
    vec3 vp_xform;

    float theta = (frame - 1) / 120.f * 2 * M_PI;
    vp_xform.x =  cos(theta) * vp.x + sin(theta) * vp.z;
    vp_xform.y = vp.y;
    vp_xform.z = -sin(theta) * vp.x + cos(theta) * vp.z;

	float dist = std::min(1.f, vp.length() / radius);
	float falloff = smoothstep(0.8, 1, dist);
    float freq = 0.5;
	size_t octaves = 5;
	float lacunarity = 2;
	float H = 0.4;
    vp_xform *= freq;
	float fbmResult = 0;
	float offset = 0.75;
	for (size_t k = 0; k < octaves; k++) {
		fbmResult += noise(vp_xform.x , vp_xform.y, vp_xform.z) * pow(lacunarity, -H * k);
        vp_xform *= lacunarity;
	}
    return std::max(0.f, fbmResult) * (1 - falloff);//(1 - falloff);//std::max(0.f, fbmResult);// * (1 - falloff));
}

vec3 integrate(const vec3& ray_orig, const vec3& ray_dir, const std::vector<std::unique_ptr<Sphere>>& spheres)
{
    const Sphere* hit_sphere = nullptr;
    IsectData isect;
    for (const auto& sph : spheres) {
        IsectData isect_sph;
        if (sph->intersect(ray_orig, ray_dir, isect_sph)) {
            hit_sphere = sph.get();
            isect = isect_sph;
        }
    }

    if (!hit_sphere)
        return background_color;

    const float step_size = 0.1;
    float sigma_a = 0.5;
    float sigma_s = 0.5;
    float sigma_t = sigma_a + sigma_s;
    float g = 0; // henyey-greenstein asymmetry factor
    uint8_t d = 2; // russian roulette "probability"

    int ns = std::ceil((isect.t1 - isect.t0) / step_size);
    float stride = (isect.t1 - isect.t0) / ns;

    vec3 light_dir{ -0.315798, 0.719361, 0.618702 };
    vec3 light_color{ 20, 20, 20 };

    float transparency = 1; // initialize transmission to 1 (fully transparent)
    vec3 result{ 0 }; // initialize volumetric sphere color to 0

    // The ray-marching loop (forward, march from t0 to t1)
    for (int n = 0; n < ns; ++n) {

        // Jitter the sample position
        float t = isect.t0 + stride * (n + distribution(generator));
        vec3 sample_pos = ray_orig + t * ray_dir;

        // [comment]
		// Get the density at this sample location
		// [/comment]
        float density = eval_density(sample_pos, hit_sphere->center, hit_sphere->radius);
        float sample_attenuation = exp(-step_size * density * sigma_t);
        transparency *= sample_attenuation;

        // In-scattering. Find distance light travels through volumetric sphere to the sample.
        // Then use Beer's law to attenuate the light contribution due to in-scattering.
        IsectData isect_light_ray;
        if (density > 0 && hit_sphere->intersect(sample_pos, light_dir, isect_light_ray) && isect_light_ray.inside) {
            size_t num_steps_light = std::ceil(isect_light_ray.t1 / step_size);
            float stide_light = isect_light_ray.t1 / num_steps_light;
            float tau = 0;
			// [comment]
			// March along the light ray and accumulate densities
			// [/comment]
            for (size_t nl = 0; nl < num_steps_light; ++nl) {
                float t_light = stide_light * (nl + 0.5);
                vec3 light_sample_pos = sample_pos + light_dir * t_light;
                tau += eval_density(light_sample_pos, hit_sphere->center, hit_sphere->radius);
            }
            float light_ray_att = exp(-tau * stide_light * sigma_t);
            result += light_color * light_ray_att * phaseHG(-ray_orig, light_dir, g) * sigma_s * transparency * stride * density;
        }

        // Russian roulette
        if (transparency < 1e-3) {
            if (distribution(generator) > 1.f / d)
                break;
            else
                transparency *= d;
        }
    }

    // combine background color and volumetric sphere color
    return background_color * transparency + result;
}

void render()
{
    fprintf(stderr, "Rendering frame: %zu\n", frame);

    unsigned int width = 640, height = 480;

    auto buffer = std::make_unique<unsigned char[]>(width * height * 3);

    auto frameAspectRatio = width / float(height);
    float fov = 45;
    float focal = tan(M_PI / 180 * fov * 0.5);

    std::vector<std::unique_ptr<Sphere>> spheres;
    std::unique_ptr<Sphere> sph = std::make_unique<Sphere>();
    sph->radius = 5;
    sph->center.x = 0;
    sph->center.y = 0;
    sph->center.z = -20;
    spheres.push_back(std::move(sph));

    vec3 rayOrig, rayDir; // ray origin & direction

    unsigned int offset = 0;
    for (unsigned int j = 0; j < height; ++j) {
        for (unsigned int i = 0; i < width; ++i) {
            rayDir.x = (2.f * (i + 0.5f) / width - 1) * focal;
            rayDir.y = (1 - 2.f * (j + 0.5f) / height) * focal * 1 / frameAspectRatio; // Maya style
            rayDir.z = -1.f;

            rayDir.nor();

            vec3 c = integrate(rayOrig, rayDir, spheres);

            buffer[offset++] = std::clamp(c.x, 0.f, 1.f) * 255;
            buffer[offset++] = std::clamp(c.y, 0.f, 1.f) * 255;
            buffer[offset++] = std::clamp(c.z, 0.f, 1.f) * 255;
        }
        fprintf(stderr, "\r%3d%c", uint32_t((j + 1) / (float)height * 100), '%');
    }
    fprintf(stderr, "\r");

    // writing file
    char filename[256];
    sprintf(filename, "./image.%04zu.ppm", frame);
    std::ofstream ofs;
    ofs.open(filename, std::ios::binary);

    ofs << "P6\n" << width << " " << height << "\n255\n";

    ofs.write(reinterpret_cast<const char*>(buffer.get()), width * height * 3);

    ofs.close();
}

int main()
{
    // init noise permutation table
    for (size_t i = 0; i < 256; i++)
        p[256 + i] = p[i] = permutation[i];

    for (frame = 1; frame < 120; ++frame)
        render();

    return 0;
}