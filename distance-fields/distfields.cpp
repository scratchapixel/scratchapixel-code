
//[header]
// This program generate and render the Utah teapot
//[/header]
//[compile]
// Download the distfields.cpp and geometry.h files to a folder.
// Open a shell/terminal, and run the following command where the files are saved:
//
// clang++ -o distfields distfields.cpp -std=c++14 -O3
//
// Run with: ./distfileds. Open the file ./spheretrace.ppm in Photoshop or any
// program reading PPM files.
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

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include "geometry.h"
#include <cmath>
#include <vector>
#include <limits>
#include <memory>

constexpr float kInfinity = std::numeric_limits<float>::max();

Matrix44f lookAt(const Vec3f& from, const Vec3f& to, Vec3f tmp = Vec3f(0, 1, 0))
{
    // because the forward axis in a right hand coordinate system points backward we compute -(to - from)
    Vec3f z = from - to; 
    z.normalize();
    Vec3f x = tmp.normalize().crossProduct(z);
    x.normalize(); // this is just in case Up is not normalized
    Vec3f y = z.crossProduct(x);

    Matrix44f camToWorld;

    // set x-axis
    camToWorld[0][0] = x[0];
    camToWorld[0][1] = x[1];
    camToWorld[0][2] = x[2];
    // set y-axis
    camToWorld[1][0] = y[0];
    camToWorld[1][1] = y[1];
    camToWorld[1][2] = y[2];
    // set z-axis
    camToWorld[2][0] = z[0];
    camToWorld[2][1] = z[1];
    camToWorld[2][2] = z[2];
    // set position
    camToWorld[3][0] = from[0];
    camToWorld[3][1] = from[1];
    camToWorld[3][2] = from[2];
    
    return camToWorld;
}

float degToRad(const float& angle)
{ return angle / 180.f * M_PI; }

// [comment]
// ImplicitShape base class
// [/comment]
class ImplicitShape
{
public:
    virtual float getDistance(const Vec3f& from) const = 0; 
    virtual ~ImplicitShape() {}
};

// [comment]
// Implicit sphere surface
// [/comment]
class Sphere : public ImplicitShape
{
public:
    Sphere(const Vec3f& c, const float& r) : center(c), radius(r) {}
    float getDistance(const Vec3f& from) const
    { return (from - center).length() - radius; }
    float radius;
    Vec3f center;
};

// [comment]
// Implicit plane surface
// [/comment]
class Plane : public ImplicitShape
{
public:
    Plane(const Vec3f& nn = Vec3f(0, 1, 0), const Vec3f& pp = Vec3f(0)) : n(nn), pointOnPlane(pp) {}
    float getDistance(const Vec3f& from) const 
    { return n.dotProduct(from - pointOnPlane); }
    Vec3f n, pointOnPlane;
};

// [comment]
// Implicit torus surface
// [/comment]
class Torus : public ImplicitShape
{
public:
    Torus(const float& _r0, const float& _r1) : r0(_r0), r1(_r1) {}
    float getDistance(const Vec3f& from) const 
    {
        // reduce 3D point to 2D point
        float tmpx = sqrtf(from.x * from.x + from.z * from.z) - r0;
        float tmpy = from.y;
        
        // distance to cicle
        return sqrtf(tmpx * tmpx + tmpy * tmpy) - r1;
    }
    float r0, r1;
};

// [comment]
// Implicit cube surface
// [/comment]
class Cube : public ImplicitShape
{
public:
    Cube(const Vec3f &_corner) : corner(_corner) {}
    float getDistance(const Vec3f& from) const 
    {
#if 0
        // first transform the input point into the object's "object-space".
        float scale = 2.f;
        
        // this matrix doesn't scale the object
        Matrix44f objectToWorld(0.542903, -0.545887, 0.638172, 0, 0.778733, 0.611711, -0.139228, 0, -0.314374, 0.572553, 0.7572, 0, 0, 1.459974, 0, 1);
        Matrix44f worldToObject = objectToWorld.inverse();
        
        Vec3f fromObjectSpace = from;
        worldToObject.multVecMatrix(from, fromObjectSpace);
#else
        Vec3f fromObjectSpace = from;
        float scale = 1;
#endif
        fromObjectSpace *= 1.f / scale;
        fromObjectSpace.x = std::fabs(fromObjectSpace.x);
        fromObjectSpace.y = std::fabs(fromObjectSpace.y);
        fromObjectSpace.z = std::fabs(fromObjectSpace.z);
        
        // now compute the distance from the point to the neares point on the surface of the object
        Vec3f d = fromObjectSpace - corner;
        
        Vec3f dmax = d;
        dmax.x = std::max(dmax.x, 0.f);
        dmax.y = std::max(dmax.y, 0.f);
        dmax.z = std::max(dmax.z, 0.f);
        
        // don't forget to apply the scale back
        return scale * (std::min(std::max(d.x, std::max(d.y, d.z)), 0.f) + dmax.length());
    }
    Vec3f corner{0.25, 0.25, 0.25};
};

struct unionFunc
{
    float operator() (float a, float b) const { return std::min(a, b); }
};

struct subtractFunc
{
    float operator() (float a, float b) const { return std::max(-a, b); }
};

struct intersectionFunc
{
    float operator() (float a, float b) const { return std::max(a, b); }
};

struct blendFunc
{
    blendFunc(const float &_k) : k(_k) {}
    float operator() (float a, float b) const
    {
        float res = exp(-k * a) + exp(-k * b);
        return -log(std::max(0.0001f, res)) / k;
    }
    float k;
};

struct mixFunc
{
    mixFunc(const float &_t) : t(_t) {}
    float operator() (float a, float b) const
    {
        return a * (1 -t) + b * t;
    }
    float t;
};

// [comment]
// Combining implict shapes using CSG
// [/comment]
template<typename Op, typename ... Args>
class CSG : public ImplicitShape
{
public:
    CSG(
        const std::shared_ptr<ImplicitShape> s1,
        const std::shared_ptr<ImplicitShape> s2,
        Args&& ... args) : op(std::forward<Args>(args) ...), shape1(s1), shape2(s2)
    {}
    float getDistance(const Vec3f& from) const
    {
        return op(shape1->getDistance(from), shape2->getDistance(from));
    }
    Op op;
    const std::shared_ptr<ImplicitShape> shape1, shape2;
};

// [comment]
// Blobbies
// [/comment]
class SoftObject : public ImplicitShape
{
    struct Blob
    {
        float R; // radius
        Vec3f c; // blob center
    };
public:
    SoftObject()
    {
#if 1
        blobbies.push_back({2.0, Vec3f(-1, 0, 0)});
        blobbies.push_back({1.5, Vec3f( 1, 0, 0)});
#else
        for (size_t i = 0; i < 20; ++i) {
            float radius = (0.3 + drand48() * 1.3);
            Vec3f c((0.5 - drand48()) * 3, (0.5 - drand48()) * 3, (0.5 - drand48()) * 3);
            blobbies.push_back({radius, c});
        }
#endif
    }
    float getDistance(const Vec3f& from) const
    {
        float sumDensity = 0;
        float sumRi = 0;
        float minDistance = kInfinity;
        for (const auto& blob: blobbies) {
            float r = (blob.c - from).length();
            if (r <=  blob.R) {
                // this can be factored for speed if you want
                sumDensity += 2 * (r * r * r) / (blob.R * blob.R * blob.R) -
                    3 * (r * r) / (blob.R * blob.R) + 1;
            }
            minDistance = std::min(minDistance, r - blob.R);
            sumRi += blob.R;
        }

        return std::max(minDistance, (magic - sumDensity) / ( 3 / 2.f * sumRi));
    }
    float magic{ 0.2 };
    std::vector<Blob> blobbies;
};

using Union = CSG<unionFunc>;
using Subtract = CSG<subtractFunc>;
using Intersect = CSG<intersectionFunc>;
using Blend = CSG<blendFunc, float>;
using Mix = CSG<mixFunc, float>;

class PointLight
{
public:
    PointLight(const Vec3f& p, const Vec3f& c, const float& i) : pos(p), col(c), intensity(i) {}
    Vec3f pos;
    Vec3f col;
    float intensity;
};

std::vector<std::shared_ptr<ImplicitShape>> makeScene()
{
    std::vector<std::shared_ptr<ImplicitShape>> shapes;

#if 0
    shapes.push_back(std::make_shared<Plane>(Vec3f(0, 1, 0), Vec3f(0, -2, 0)));
    shapes.push_back(std::make_shared<Cube>());
    shapes.push_back(std::make_shared<Torus>(2, 0.65));
#elif 0
    shapes.push_back(std::make_shared<Plane>(Vec3f(0, 1, 0), Vec3f(0, -2, 0)));
    shapes.push_back(std::make_shared<Blend>(
        std::make_shared<Cube>(Vec3f(1.5)),
        std::make_shared<Torus>(2, 0.65), 5));
#elif 0
    shapes.push_back(std::make_shared<Blend>(
        std::make_shared<Plane>(Vec3f(0, 1, 0), Vec3f(0, 0, 0)),
        std::make_shared<Torus>(2, 0.65), 5));
#elif 0
    shapes.push_back(std::make_shared<Plane>(Vec3f(0, 1, 0), Vec3f(0, -2, 0)));
    shapes.push_back(std::make_shared<Mix>(
        std::make_shared<Cube>(Vec3f(1)),
        std::make_shared<Sphere>(Vec3f(0), 1), 0.5));
#else
    shapes.push_back(std::make_shared<Plane>(Vec3f(0, 1, 0), Vec3f(0, -2, 0)));
    shapes.push_back(std::make_shared<SoftObject>());
#endif
    return shapes; 
}

bool sphereTraceShadow( 
    const Vec3f& rayOrigin, 
    const Vec3f& rayDirection, 
    const float& maxDistance, 
    const std::vector<std::shared_ptr<ImplicitShape>> scene)
{ 
    constexpr float threshold = 10e-5; 
    float t = 0; 
 
    while (t < maxDistance) { 
        float minDistance = kInfinity; 
        Vec3f from = rayOrigin + t * rayDirection; 
        for (auto shape : scene) { 
            float d = shape->getDistance(from); 
            if (d < minDistance) 
                minDistance = d; 
                // did we find an intersection?
                if (minDistance <= threshold * t) 
                    return true; 
        } 
 
        // no intersection, move along the ray by minDistance
        t += minDistance; 
    } 
 
    return false; 
}

Vec3f shade(
    const Vec3f& rayOrigin,
    const Vec3f& rayDirection,
    const float& t,
    const ImplicitShape *shape,
    const std::vector<std::shared_ptr<ImplicitShape>> scene,
    const std::vector<std::unique_ptr<PointLight>> &lights)
{
   constexpr float delta = 10e-5; 
    Vec3f p = rayOrigin + t * rayDirection; 
    Vec3f n = Vec3f( 
        shape->getDistance(p + Vec3f(delta, 0, 0)) - shape->getDistance(p + Vec3f(-delta, 0, 0)), 
        shape->getDistance(p + Vec3f(0, delta, 0)) - shape->getDistance(p + Vec3f(0, -delta, 0)), 
        shape->getDistance(p + Vec3f(0, 0, delta)) - shape->getDistance(p + Vec3f(0, 0, -delta))); 
    n.normalize(); 
 
    Vec3f R = 0; 
 
    // loop over all lights in the scene and add their contribution to P's brightness
    for (const auto& light: lights) {
        Vec3f lightDir = light->pos - p; 
        if (lightDir.dotProduct(n) > 0) { 
            float dist2 = lightDir.norm(); 
            lightDir.normalize(); 
            bool shadow = 1 - sphereTraceShadow(p, lightDir, sqrtf(dist2), scene); 
            R += shadow * lightDir.dotProduct(n) * light->col * light->intensity / (4 * M_PI * dist2); 
        } 
    } 
 
    return R; 
}

Vec3f sphereTrace(
    const Vec3f& rayOrigin, 
    const Vec3f& rayDirection, 
    const std::vector<std::shared_ptr<ImplicitShape>>& scene,
    const std::vector<std::unique_ptr<PointLight>>& lights)
{
    constexpr float maxDistance = 100;
    float t = 0;
    uint32_t numSteps = 0;
    const ImplicitShape *isectShape = nullptr;

    constexpr float threshold = 10e-6;

    while (t < maxDistance) {
        float minDistance = kInfinity;
        Vec3f from = rayOrigin + t * rayDirection;
        for (const auto& shape : scene) {
            float d = shape->getDistance(from);
            if (d < minDistance) {
                minDistance = d;
                isectShape = shape.get();
            }
        }

        if (minDistance <= threshold * t) {
            return shade(rayOrigin, rayDirection, t, isectShape, scene, lights);
        }
        t += minDistance; 
        numSteps++;
    }

    return 0; 
}

int main(int argc, char **argv)
{
    srand48(13);

    Matrix44f camToWorld = lookAt(Vec3f(0, 1, 9), 0);

    std::vector<std::shared_ptr<ImplicitShape>> scene = makeScene();
    std::vector<std::unique_ptr<PointLight>> lights;
    lights.push_back(std::make_unique<PointLight>(Vec3f( 20, 30,  20), Vec3f(1.0, 0.9, 0.7), 4000));
    lights.push_back(std::make_unique<PointLight>(Vec3f(-20, 30, -20), Vec3f(0.8, 0.9, 1.0), 4000));
    lights.push_back(std::make_unique<PointLight>(Vec3f( -5, 10,  20), Vec3f(1.0, 1.0, 1.0), 3000));

    constexpr uint32_t width = 640, height = 480;
    constexpr float ratio = width / static_cast<float>(height);
    constexpr float fov = 60;
    float angle = tan(degToRad(fov * 0.5));

    Vec3f *buffer = new Vec3f[width * height];

    Vec3f rayOrigin;
    camToWorld.multVecMatrix(Vec3f(0), rayOrigin);
    for (uint32_t j = 0; j < height; ++j) {
        for (uint32_t i = 0; i < width; ++i) {
            float x = (2 * i / static_cast<float>(width) - 1) * ratio * angle;
            float y = (1 - j / static_cast<float>(height) * 2) * angle;
            Vec3f rayDirection; 
            camToWorld.multDirMatrix(Vec3f(x, y, -1).normalize(), rayDirection);
            ImplicitShape *tmp;
            Vec3f pixelColor = sphereTrace(rayOrigin, rayDirection, scene, lights);
            buffer[width * j + i] = pixelColor;
        }
    }

    std::ofstream ofs;
    ofs.open("./spheretrace.ppm");
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (uint32_t i = 0; i < width * height; ++i) {
        unsigned char r = static_cast<unsigned char>(std::min(1.0f, buffer[i][0]) * 255);
        unsigned char g = static_cast<unsigned char>(std::min(1.0f, buffer[i][1]) * 255);
        unsigned char b = static_cast<unsigned char>(std::min(1.0f, buffer[i][2]) * 255);
        ofs << r << g << b;
    }
    ofs.close();

    delete [] buffer;

    return 0;
}
