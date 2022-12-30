//[header]
// Rendering volumetric object using ray-marching. A basic implementation (chapter 1 & 2)
//
// https://www.scratchapixel.com/lessons/advanced-rendering/volume-rendering-for-developers/ray-marching-algorithm
//[/header]
//[compile]
// Download the raymarch-chap5.cpp file to a folder and the cachefiles.zip archive.
// Unzip the content of the archive (the program file and the cache files should be in the same location).
// Open a shell/terminal, and run the following command where the file is saved:
//
// clang++ -O3 raymarch-chap5.cpp -o render -std=c++17
//
// You can use c++ if you don't use clang++
//
// Run with: ./render. Open the resulting images (ppm) in Photoshop or any other program
// capable of reading PPM files.
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
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <memory>
#include <fstream>
#include <algorithm>

struct Matrix
{
    const float operator [] (size_t i) const { return (m)[i]; }
    float& operator [] (size_t i) { return m[i]; }

    float m[16];
};

struct Vector
{
    Vector() : x(0), y(0), z(0) {}
    Vector(const float& value) : x(value), y(value), z(value) {}
    Vector(const float& xval, const float &yval, const float& zval) : x(xval), y(yval), z(zval) {}
    Vector operator * (const Matrix& m) const
    {
        Vector v;
        v.x = m[0] * x + m[4] * y + m[ 8] * z;
        v.y = m[1] * x + m[5] * y + m[ 9] * z;
        v.z = m[2] * x + m[6] * y + m[10] * z;

        return v;
    }

    Vector& operator *= (const Matrix& m) 
    { *this = *this * m; return *this; }

    template<typename U>
    Vector operator * (const U& value) const
    { return Vector(x * value, y * value, z * value); }

    Vector operator / (const Vector& v) const
    { return Vector(x / v.x, y / v.y, z / v.z); }

    float dot(const Vector& v) const
    { return x * v.x + y * v.y + z * v.z; }
 
    Vector cross(const Vector& v) const
    {
        return Vector(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    template<typename U>
    bool operator >= (const U& value)
    { return x > value && y > value && z > value; }

    template<typename U>
    bool operator <= (const U& value)
    { return x < value && y < value && z < value; }

    Vector operator - () const
    { return Vector(-x, -y, -z); }

    float length() const
    { return sqrtf(x * x + y * y + z * z); }

    Vector& normalize()
    {
        float len = length();
        x /= len, y /= len, z /= len;

        return *this;
    }

    friend Vector operator / (const float& r, const Vector& v)
    { return Vector{ r / v.x, r / v.y, r / v.z }; }

    float x, y, z;
};

struct Point
{
    Point() : x(0), y(0), z(0) {}
    Point(const float& value) : x(value), y(value), z(value) {}
    Point(const float& xval, const float& yval, const float& zval) : x(xval), y(yval), z(zval) {}
    Point operator * (const Matrix& m) const
    {
        Point p;
        p.x     = m[0] * x + m[4] * y + m[ 8] * z + m[12];
        p.y     = m[1] * x + m[5] * y + m[ 9] * z + m[13];
        p.z     = m[2] * x + m[6] * y + m[10] * z + m[14];
        float w = m[3] * x + m[7] * y + m[11] * z + m[15];

        if (w != 1) {
            p.x /= w;
            p.y /= w;
            p.z /= w;
        }

        return p;
    }

    Point operator * (const Point& p) const
    { return Point(x * p.x, y * p.y, z * p.z); }

    Point operator + (const Vector& v) const
    { return Point(x + v.x, y + v.y, z + v.z); }

    Vector operator - (const Point& p) const
    { return Vector(x - p.x, y - p.y, z - p.z); }

    Point operator / (const Point& p) const
    { return Point(x / p.x, y / p.y, z / p.z); }

    float x, y, z; /* w */
};

struct Color
{
    Color() : r(0), g(0), b(0) {}
    Color(float value) : r(value), g(value), b(value) {}
    Color(const float& rval, const float& gval, const float& bval) : r(rval), g(gval), b(bval) {}
    Color& operator += (const Color& c)
    { r += c.r, g += c.g, b += c.b; return *this; }
    Color operator * (const float& value) const
    { return Color(r * value, g * value, b * value); }
    Color operator + (const Color& c)
    { return Color(r + c.r, g + c.g, b + c.b); }

    float r, g, b;
};

Matrix cameraToWorld{ 0.844328, 0, -0.535827, 0, -0.170907, 0.947768, -0.269306, 0, 0.50784, 0.318959, 0.800227, 0, 83.292171, 45.137326, 126.430772, 1 };

struct Grid
{
    size_t baseResolution = 128;
    std::unique_ptr<float[]> densityData;
    Point bounds[2]{ Point(-30), Point(30) };
    float operator () (const int& xi, const int& yi, const int& zi) const {
        if (xi < 0 || xi > baseResolution - 1 ||
            yi < 0 || yi > baseResolution - 1 ||
            zi < 0 || zi > baseResolution - 1)
            return 0;

        return densityData[(zi * baseResolution + yi) * baseResolution + xi];
    }
};

struct Ray
{
    Ray(const Point& p, const Vector& d) : orig(p), dir(d)
    {
        invDir = 1 / dir;

        sign[0] = (invDir.x < 0);
        sign[1] = (invDir.y < 0);
        sign[2] = (invDir.z < 0);
    }
    Point operator() (const float &t) const
    { return orig + dir * t; }
    Point orig;
    Vector dir, invDir;
    bool sign[3];
};

bool raybox(const Ray &ray, const Point bounds[2], float &tmin, float &tmax)
{    
    float a, b, c, d, e, f;
    a = bounds[    ray.sign[0]].x - ray.orig.x;
    b = bounds[1 - ray.sign[0]].x - ray.orig.x;
    c = bounds[    ray.sign[1]].y - ray.orig.y;
    d = bounds[1 - ray.sign[1]].y - ray.orig.y;

    float x0 = a == 0 ? 0 : a * ray.invDir.x;
    float x1 = b == 0 ? 0 : b * ray.invDir.x;
    float y0 = c == 0 ? 0 : c * ray.invDir.y;
    float y1 = d == 0 ? 0 : d * ray.invDir.y;
    
    if ((x0 > y1) || (y0 > x1)) return false;

    tmin = (y0 > x0) ? y0 : x0;
    tmax = (y1 < x1) ? y1 : x1;

    e = bounds[    ray.sign[2]].z - ray.orig.z;
    f = bounds[1 - ray.sign[2]].z - ray.orig.z;

    float z0 = e == 0 ? 0 : e * ray.invDir.z;
    float z1 = f == 0 ? 0 : f * ray.invDir.z;

    if ((tmin > z1) || (z0 > tmax)) return false;

    tmin = std::max(z0, tmin);
    tmax = std::min(z1, tmax);

    return true;
}

float phaseHG(const Vector& viewDir, const Vector& lightDir, const float& g)
{
    float costheta = viewDir.dot(lightDir);
    return 1 / (4 * M_PI) * (1 - g * g) / powf(1 + g * g - 2 * g * costheta, 1.5);
}

//[comment]
// Function where the coordinates of the sample points are converted from world space
// to voxel space. We can then use these coordinates to read the density stored
// in the grid. We can either use a nearest neighbor search or a trilinear
// interpolation.
//[/comment]
float lookup(const Grid& grid, const Point& p)
{
    Vector gridSize = grid.bounds[1] - grid.bounds[0];
    Vector pLocal = (p - grid.bounds[0]) / gridSize;
    Vector pVoxel = pLocal * grid.baseResolution;

    Vector pLattice(pVoxel.x - 0.5, pVoxel.y - 0.5, pVoxel.z - 0.5);
    int xi = static_cast<int>(std::floor(pLattice.x));
    int yi = static_cast<int>(std::floor(pLattice.y));
    int zi = static_cast<int>(std::floor(pLattice.z));
#if 0
    // nearest neighbor seach
    return grid(xi, yi, zi);
#else
    // trilinear filtering
    float weight[3];
    float value = 0;

    for (int i = 0; i < 2; ++i) {
        weight[0] = 1 - std::abs(pLattice.x - (xi + i));
        for (int j = 0; j < 2; ++j) {
            weight[1] = 1 - std::abs(pLattice.y - (yi + j));
            for (int k = 0; k < 2; ++k) {
                weight[2] = 1 - std::abs(pLattice.z - (zi + k));
                value += weight[0] * weight[1] * weight[2] * grid(xi + i, yi + j, zi + k);
            }
        }
    }

    return value;
#endif
}

void integrate(
    const Ray &ray,                         // camera ray 
    const float &tMin, const float &tMax,   // range of integration
    Color &L,                               // radiance (out)
    float &T,                               // transmission (out)
    const Grid& grid)                       // cached data
{
    const float stepSize = 0.05;
    float sigma_a = 0.5;
    float sigma_s = 0.5;
    float sigma_t = sigma_a + sigma_s;
    float g = 0; // henyey-greenstein asymetry factor 
    size_t d = 2; // russian roulette "probability" 
    float shadowOpacity = 1;

    size_t numSteps = std::ceil((tMax - tMin) / stepSize);
    float stride = (tMax - tMin) / numSteps;

    Vector lightDir(-0.315798, 0.719361, 0.618702);
    Color lightColor(20);

    Color Lvol = 0;
    float Tvol = 1;

    for (size_t n = 0; n < numSteps; ++n) {
        float t = tMin + stride * (n + 0.5);
        Point samplePos = ray(t);

        //[comment]
        // Read density from the 3D grid
        //[/comment]
        float density = lookup(grid, samplePos);

        float Tsample = exp(-stride * density * sigma_t);
        Tvol *= Tsample;

        float tlMin, tlMax;
        Ray lightRay(samplePos, lightDir);
        if (density > 0 && raybox(lightRay, grid.bounds, tlMin, tlMax) && tlMax > 0) {
            size_t numStepsLight = std::ceil(tlMax / stepSize);
            float strideLight = tlMax / numStepsLight;
            float densityLight = 0;
            for (size_t nl = 0; nl < numStepsLight; ++nl) {
                float tLight = strideLight * (nl + 0.5);
                //[comment]
                // Read density from the 3D grid
                //[/comment]
                densityLight += lookup(grid, lightRay(tLight));
            }
            float lightRayAtt = exp(-densityLight * strideLight * sigma_t * shadowOpacity);
            Lvol += lightColor * lightRayAtt * phaseHG(-ray.dir, lightDir, g) * sigma_s * Tvol * stride * density;
        }
        
        if (Tvol < 1e-3) {
            if (rand() / (float)RAND_MAX > 1.f / d)
                break;
            else
                Tvol *= d;
        }
    }

    L = Lvol;
    T = Tvol;
}

struct RenderContext
{
    float fov{ 45 };
    size_t width{ 640 }, height{ 480 };
    float frameAspectRatio;
    float focal;
    float pixelWidth;
    Color backgroundColor{ 0.572f, 0.772f, 0.921f };
};

void initRenderContext(RenderContext& rc)
{
    rc.frameAspectRatio = rc.width / float(rc.height);
    rc.focal = tanf(M_PI / 180 * rc.fov * 0.5f);
    rc.pixelWidth = rc.focal / rc.width;
}

void trace(Ray &ray, Color &L, float &transmittance, const RenderContext& rc, const Grid& grid)
{
    float tmin, tmax;
    if (raybox(ray, grid.bounds, tmin, tmax)) {
        integrate(ray, tmin, tmax, L, transmittance, grid);
    }
}

void render(const size_t& frame)
{
    fprintf(stderr, "Rendering frame: %zu\n", frame);

    //[comment]
    // Load the density data from file into memory for this current frame
    //[/comment]
    std::ifstream ifs;
    char filename[256];
    sprintf_s(filename, "./grid.%d.bin", frame);
    ifs.open(filename, std::ios::binary);
    Grid grid;
    grid.densityData = std::make_unique<float[]>(grid.baseResolution * grid.baseResolution * grid.baseResolution);
    ifs.read((char*)grid.densityData.get(), sizeof(float) * grid.baseResolution * grid.baseResolution * grid.baseResolution);
    ifs.close();

    size_t width = 640, height = 480;
    
    RenderContext rc;
    initRenderContext(rc);

    size_t nsamples = 1;
    size_t offset = 0;

    std::unique_ptr<char[]> imgbuf = std::make_unique<char[]>(width * height * 3);

    Point rayOrig = Point(0) * cameraToWorld;

    for (unsigned int j = 0; j < height; ++j) {
        for (unsigned int i = 0; i < width; ++i) {
            Color pixelColor;
            //float  opacity = 0;
            for (unsigned jj = 0; jj < nsamples; ++jj) {
                for (unsigned ii = 0; ii < nsamples; ++ii) {
                    Vector rayDir;
                    rayDir.x = (2 * (i + 1.f / nsamples * (ii + 0.5f)) / width - 1) * rc.focal;
                    rayDir.y = (1 - 2 * (j + 1.f / nsamples * (jj + 0.5f)) / height) * rc.focal * 1 / rc.frameAspectRatio; // Maya style
                    rayDir.z = -1;

                    rayDir *= cameraToWorld;
                    rayDir.normalize();

                    Ray ray(rayOrig, rayDir);

                    Color L; // radiance for that ray (light collected)
                    float transmittance = 1;
                    trace(ray, L, transmittance, rc, grid);
                    pixelColor += rc.backgroundColor * transmittance + L;
                }
            }
            imgbuf[offset++] = static_cast<char>(std::clamp(pixelColor.r, 0.f, 1.f) * 255);
            imgbuf[offset++] = static_cast<char>(std::clamp(pixelColor.g, 0.f, 1.f) * 255);
            imgbuf[offset++] = static_cast<char>(std::clamp(pixelColor.b, 0.f, 1.f) * 255);
        }
        fprintf(stderr, "\r%3d%c", uint32_t((j + 1) / (float)height * 100), '%');
    }
    fprintf(stderr, "\r");

    // writing file
    std::ofstream ofs;
    sprintf_s(filename, "./smoke.%04d.ppm", frame);
    ofs.open(filename, std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(imgbuf.get()), width * height * 3);
    ofs.close();
}

int main()
{
    for (size_t frame = 1; frame <= 90; ++frame) {
        render(frame);
    }

    return 0;
}