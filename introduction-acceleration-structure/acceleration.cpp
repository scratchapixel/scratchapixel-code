//[header]
// Example of Acceleration Structures for Ray-Tracing (BBox, BVH and Grid)
//[/header]
//[compile]
// Download the acceleration.cpp and teapotdata.h file to a folder.
// Open a shell/terminal, and run the following command where the files is saved:
//
// clang++ -std=c++14 -o acceleration acceleration.cpp -O3 -DACCEL_BBOX
//
// clang++ -std=c++14 -o acceleration acceleration.cpp -O3 -DACCEL_BVH
//
// clang++ -std=c++14 -o acceleration acceleration.cpp -O3 -DACCEL_GRID
//
// You can use c++ if you don't use clang++
//
// Run with: ./acceleration. Open the file ./image.png in Photoshop or any program
// reading PPM files.
//[/compile]
//[ignore]
// Copyright (C) 2016  www.scratchapixel.com
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

#include <atomic>
#include <memory>
#include <cassert>
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <chrono>
#include <queue>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

const float kEpsilon = 1e-8;
const float kInfinity = std::numeric_limits<float>::max();

std::atomic<uint32_t> numPrimaryRays(0);
std::atomic<uint32_t> numRayTriangleTests(0);
std::atomic<uint32_t> numRayTriangleIntersections(0);
std::atomic<uint32_t> numRayBBoxTests(0);
std::atomic<uint32_t> numRayBoundingVolumeTests(0);

template<typename T>
class Vec3
{
public:
    Vec3() : x(0), y(0), z(0) {}
    Vec3(T xx) : x(xx), y(xx), z(xx) {}
    Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
    Vec3 operator * (const T& r) const { return Vec3(x * r, y * r, z * r); }
    Vec3 operator + (const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator - (const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    template<typename U>
    Vec3 operator / (const Vec3<U>& v) const { return Vec3(x / v.x, y / v.y, z / v.z); }
    friend Vec3 operator / (const T r, const Vec3& v)
    {
        return Vec3(r / v.x, r / v.y, r / v.z);
    }
    const T& operator [] (size_t i) const { return (&x)[i]; }
    T& operator [] (size_t i) { return (&x)[i]; }
    T length2() const { return x * x + y * y + z * z; }
    friend Vec3 operator * (const float& r, const Vec3& v)
    {
        return Vec3(v.x * r, v.y * r, v.z * r);
    }
    friend std::ostream& operator << (std::ostream& os, const Vec3<T>& v)
    {
        os << v.x << " " << v.y << " " << v.z << std::endl; return os;
    }
    T x, y, z;
};

template<typename T>
Vec3<T> cross(const Vec3<T>& a, const Vec3<T>& b)
{
    return Vec3<T>(a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

template<typename T>
T dot(const Vec3<T>& va, const Vec3<T>& vb)
{
    return va.x * vb.x + va.y * vb.y + va.z * vb.z;
}

template<typename T>
void normalize(Vec3<T>& vec)
{
    T len2 = vec.length2();
    if (len2 > 0) {
        T invLen = 1 / sqrt(len2);
        vec.x *= invLen, vec.y *= invLen, vec.z *= invLen;
    }
}

template<typename T>
class Matrix44
{
public:
    Matrix44() { /* ... define identity matrix ... */ }
    Matrix44(T m00, T m01, T m02, T m03,
        T m10, T m11, T m12, T m13,
        T m20, T m21, T m22, T m23,
        T m30, T m31, T m32, T m33)
    {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
    }
    Matrix44 inverse() const { Matrix44 matInv = *this; return matInv; }
    T* operator [] (size_t i) { return &m[i][0]; }
    const T* operator [] (size_t i) const { return &m[i][0]; }
    T m[4][4] = { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1} };
};

template<typename T>
void matVecMult(const Matrix44<T>& m, Vec3<T>& v)
{
    Vec3<T> vt;
    vt.x = v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0],
        vt.y = v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1],
        vt.z = v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2];

    v = vt;
}

template<typename T>
void matPointMult(const Matrix44<T>& m, Vec3<T>& p)
{
    Vec3<T> pt;
    pt.x = p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + m[3][0];
    pt.y = p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + m[3][1];
    pt.z = p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2] + m[3][2];
    T w = p.x * m[0][3] + p.y * m[1][3] + p.z * m[2][3] + m[3][3];
    if (w != 1) {
        pt.x /= w, pt.y /= w, pt.z /= w;
    }

    p = pt;
}

using Vec3f = Vec3<float>;
using Vec3b = Vec3<bool>;
using Vec3i = Vec3<int32_t>;
using Vec3ui = Vec3<uint32_t>;
using Matrix44f = Matrix44<float>;

template<typename T = float>
class BBox
{
public:
    BBox() {}
    BBox(Vec3<T> min_, Vec3<T> max_)
    {
        bounds[0] = min_;
        bounds[1] = max_;
    }
    BBox& extendBy(const Vec3<T>& p)
    {
        if (p.x < bounds[0].x) bounds[0].x = p.x;
        if (p.y < bounds[0].y) bounds[0].y = p.y;
        if (p.z < bounds[0].z) bounds[0].z = p.z;
        if (p.x > bounds[1].x) bounds[1].x = p.x;
        if (p.y > bounds[1].y) bounds[1].y = p.y;
        if (p.z > bounds[1].z) bounds[1].z = p.z;

        return *this;
    }
    /*inline */ Vec3<T> centroid() const { return (bounds[0] + bounds[1]) * 0.5; }
    Vec3<T>& operator [] (bool i) { return bounds[i]; }
    const Vec3<T> operator [] (bool i) const { return bounds[i]; }
    bool intersect(const Vec3<T>&, const Vec3<T>&, const Vec3b&, float&) const;
    Vec3<T> bounds[2] = { kInfinity, -kInfinity };
};

template<typename T>
bool BBox<T>::intersect(const Vec3<T>& orig, const Vec3<T>& invDir, const Vec3b& sign, float& tHit) const
{
    numRayBBoxTests++;
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    tmin = (bounds[sign[0]].x - orig.x) * invDir.x;
    tmax = (bounds[1 - sign[0]].x - orig.x) * invDir.x;
    tymin = (bounds[sign[1]].y - orig.y) * invDir.y;
    tymax = (bounds[1 - sign[1]].y - orig.y) * invDir.y;

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;

    tzmin = (bounds[sign[2]].z - orig.z) * invDir.z;
    tzmax = (bounds[1 - sign[2]].z - orig.z) * invDir.z;

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    tHit = tmin;

    return true;
}

template<typename T> inline T clamp(const T& v, const T& lo, const T& hi)
{
    return std::max(lo, std::min(v, hi));
}

class GeomPrimitive
{
public:
    GeomPrimitive(Matrix44f objectToWorld_) :
        objectToWorld(objectToWorld_), worldToObject(objectToWorld.inverse())
    {}
    virtual ~GeomPrimitive() {}
    virtual bool intersect(const Vec3f&, const Vec3f&, float&) const = 0;
    Matrix44f objectToWorld;
    Matrix44f worldToObject;
    BBox<> bbox;
    uint32_t test;
};

class Mesh : public GeomPrimitive
{
public:
    Mesh(
        uint32_t numPolygons,
        const std::vector<uint32_t>& polygonNumVertsArray,          // how many vertices are making each face (array size is num polys)
        const std::vector<uint32_t>& polygonIndicesInVertexPool,    // the index of the vertices making each face (array size is the sum of each number in prev array)
        std::vector<Vec3f> vertexPool_,                             // the vertex positions (should be at least as many as max index in previous array)
        Matrix44f objectToWorld_ = Matrix44f()) :
        GeomPrimitive(objectToWorld_),
        vertexPool(vertexPool_)
    {
        // pass by value (move constructor shouldn't even be called here ?
        for (uint32_t i = 0; i < vertexPool.size(); ++i) {
            matPointMult(objectToWorld, vertexPool[i]);
            bbox.extendBy(vertexPool[i]);
        }
        // compute total number of triangles
        for (uint32_t i = 0; i < numPolygons; ++i) {
            assert(polygonNumVertsArray[i] >= 3);
            numTriangles += polygonNumVertsArray[i] - 2;
        }
        // create array to store the triangle indices in the vertex pool
        // !! use resize() here and not reserve() -- which only affects capacity but doesn't change size of the vector
        triangleIndicesInVertexPool.resize(numTriangles * 3);
        mailbox.resize(numTriangles); // should all be initialized with 0
        // for each face
        for (uint32_t i = 0, offset = 0, currTriangleIndex = 0; i < numPolygons; ++i) {
            // for each triangle in the face
            for (uint32_t j = 0; j < polygonNumVertsArray[i] - 2; ++j) {
                triangleIndicesInVertexPool[currTriangleIndex] = polygonIndicesInVertexPool[offset];
                triangleIndicesInVertexPool[currTriangleIndex + 1] = polygonIndicesInVertexPool[offset + j + 1];
                triangleIndicesInVertexPool[currTriangleIndex + 2] = polygonIndicesInVertexPool[offset + j + 2];
                currTriangleIndex += 3;
            }
            offset += polygonNumVertsArray[i];
        }
    }
    bool intersect(const Vec3f&, const Vec3f&, float& t) const;
    uint32_t numTriangles = { 0 };
    std::vector<uint32_t> triangleIndicesInVertexPool;
    std::vector<Vec3f> vertexPool;

    // [comment]
    // Mailboxes are used by the Grid acceleration structure
    // [/comment]
    mutable std::vector<uint32_t> mailbox;
};

// use Moller-Trumbor method
bool rayTriangleIntersect(
    const Vec3f& orig, const Vec3f& dir,
    const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
    float& t, float& u, float& v)
{
    numRayTriangleTests++;
    Vec3f v0v1 = v1 - v0;
    Vec3f v0v2 = v2 - v0;
    Vec3f pvec = cross(dir, v0v2);
    float det = dot(v0v1, pvec);

    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;

    float invDet = 1 / det;

    Vec3f tvec = orig - v0;
    u = dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1) return false;

    Vec3f qvec = cross(tvec, v0v1);
    v = dot(dir, qvec) * invDet;
    if (v < 0 || u + v > 1) return false;

    t = dot(v0v2, qvec) * invDet;

    numRayTriangleIntersections++;

    return true;
}

bool Mesh::intersect(const Vec3f& rayOrig, const Vec3f& rayDir, float& tNear) const
{
    // naive approach, loop over all triangles in the mesh and return true if one 
    // of the triangles at least is intersected
    float t, u, v;
    uint32_t intersectedTriIndex;
    bool intersected = false;
    // tNear should be set inifnity first time this function is called and it 
    // will get eventually smaller as the ray intersects geometry
    for (uint32_t i = 0; i < numTriangles; ++i) {
        if (rayTriangleIntersect(rayOrig, rayDir,
            vertexPool[triangleIndicesInVertexPool[i * 3]],
            vertexPool[triangleIndicesInVertexPool[i * 3 + 1]],
            vertexPool[triangleIndicesInVertexPool[i * 3 + 2]], t, u, v) && t < tNear)
        {
            tNear = t;
            intersectedTriIndex = i;
            intersected = true;
        }
    }

    return intersected;
}

#include "teapotdata.h"

Vec3f evalBezierCurve(const Vec3f* P, const float& t)
{
    float b0 = (1 - t) * (1 - t) * (1 - t);
    float b1 = 3 * t * (1 - t) * (1 - t);
    float b2 = 3 * t * t * (1 - t);
    float b3 = t * t * t;

    return P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3;
}

Vec3f evalBezierPatch(const Vec3f* controlPoints, const float& u, const float& v)
{
    Vec3f uCurve[4];
    for (size_t i = 0; i < 4; ++i)
        uCurve[i] = evalBezierCurve(controlPoints + 4 * i, u);

    return evalBezierCurve(uCurve, v);
}

Vec3f derivBezier(const Vec3f* P, const float& t)
{
    return -3 * (1 - t) * (1 - t) * P[0] +
        (3 * (1 - t) * (1 - t) - 6 * t * (1 - t)) * P[1] +
        (6 * t * (1 - t) - 3 * t * t) * P[2] +
        3 * t * t * P[3];
}

Vec3f dUBezier(const Vec3f* controlPoints, float u, float v)
{
    Vec3f P[4];
    Vec3f vCurve[4];
    for (size_t i = 0; i < 4; ++i) {
        P[0] = controlPoints[i];
        P[1] = controlPoints[4 + i];
        P[2] = controlPoints[8 + i];
        P[3] = controlPoints[12 + i];
        vCurve[i] = evalBezierCurve(P, v);
    }

    return derivBezier(vCurve, u);
}

Vec3f dVBezier(const Vec3f* controlPoints, float u, float v)
{
    Vec3f uCurve[4];
    for (size_t i = 0; i < 4; ++i) {
        uCurve[i] = evalBezierCurve(controlPoints + 4 * i, u);
    }

    return derivBezier(uCurve, v);
}

std::vector<std::unique_ptr<const Mesh>> createUtahTeapot()
{
    Matrix44f rotate90(1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
    std::vector<std::unique_ptr<const Mesh>> meshes;
    uint32_t width = 8, height = 8;
    uint32_t numPolygons = width * height;
    std::vector<uint32_t> polyNumVertsArray(numPolygons, 4);
    std::vector<uint32_t> polyIndicesInVertPool(numPolygons * 4);
    // set indices 
    for (uint32_t y = 0, offset = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x, offset += 4) {
            // counter-clockwise to get the normal pointing in the right direction
            polyIndicesInVertPool[offset] = (width + 1) * y + x;
            polyIndicesInVertPool[offset + 3] = (width + 1) * y + x + 1;
            polyIndicesInVertPool[offset + 2] = (width + 1) * (y + 1) + x + 1;
            polyIndicesInVertPool[offset + 1] = (width + 1) * (y + 1) + x;
        }
    }
    Vec3f controlPoints[16];
    for (uint32_t i = 0; i < kTeapotNumPatches; ++i) {
        std::vector<Vec3f> vertPool((width + 1) * (height + 1));
        for (uint32_t j = 0; j < 16; ++j) {
            controlPoints[j].x = teapotVertices[teapotPatches[i][j] - 1][0],
                controlPoints[j].y = teapotVertices[teapotPatches[i][j] - 1][1],
                controlPoints[j].z = teapotVertices[teapotPatches[i][j] - 1][2];
        }
        for (uint32_t y = 0, currVertIndex = 0; y <= height; ++y) {
            float v = y / (float)height;
            for (uint32_t x = 0; x <= width; ++x, ++currVertIndex) {
                float u = x / (float)width;
                vertPool[currVertIndex] = evalBezierPatch(controlPoints, u, v);
                matVecMult(rotate90, vertPool[currVertIndex]);
                Vec3f dU = dUBezier(controlPoints, u, v);
                Vec3f dV = dVBezier(controlPoints, u, v);
                Vec3f N = cross(dU, dV);
            }
        }

        meshes.emplace_back(new Mesh(numPolygons, polyNumVertsArray, polyIndicesInVertPool, vertPool));
    }

    return meshes;
}

void makeScene(std::vector<std::unique_ptr<const Mesh>>& meshes)
{
    meshes = std::move(createUtahTeapot());
}

template<typename T>
inline
T degToRad(const T& angle) { return angle / 180.f * M_PI; }

struct Options
{
    float fov = { 90 };
    uint32_t width = { 640 };
    uint32_t height = { 480 };
    Matrix44f cameraToWorld, worldToCamera;
};

// [comment]
// The most basic acceleration class (the parent class of all the other acceleration structures)
// could have a *pure* virtual intersect() method but instead we decided in this implementation
// to have it supporting the basic ray-mesh intersection routine.
// [/comment]
class AccelerationStructure
{
public:
    // [comment]
    // We transfer ownership of the mesh list to the acceleration structure. This makes
    // more sense from a functional/structure stand point because the objects/meshes themselves
    // should be destroyed/deleted when the acceleration structure is being deleted
    // Ideally this means the render function() itself should be bounded (in terms of lifespan)
    // to the lifespan of the acceleration structure (aka we should wrap the accel struc instance
    // and the render method() within the same object, so that when this object is deleted,
    // the render function can't be called anymore.
    // [/comment]
    AccelerationStructure(std::vector<std::unique_ptr<const Mesh>>& m) : meshes(std::move(m)) {}
    virtual ~AccelerationStructure() {}
    virtual bool intersect(const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId, float& tHit) const
    {
        // [comment]
        // Because we don't want to change the content of the mesh itself, just get a pointer to it so
        // it's safer to make it const (which doesn't mean we can't change its assignment, just that
        // we can't do something like intersectedMesh->color = blue. You would get something like:
        // "read-only variable is not assignable" error message at compile time)
        // [/comment]
        const Mesh* intersectedMesh = nullptr;
        float t = kInfinity;
        for (const auto& mesh : meshes) {
            if (mesh->intersect(orig, dir, t) && t < tHit) {
                intersectedMesh = mesh.get();
                tHit = t;
            }
        }

        return (intersectedMesh != nullptr);
    }

    const std::vector<std::unique_ptr<const Mesh>> meshes;
};

// [comment]
// Implementation of the ray-bbox method. If the ray intersects the bbox of a mesh then
// we test if the ray intersects the mesh contained by the bbox itself.
// [/comment]
class BBoxAcceleration : public AccelerationStructure
{
public:
    BBoxAcceleration(std::vector<std::unique_ptr<const Mesh>>& m) : AccelerationStructure(m) {}

    // [comment]
    // Implement the ray-bbox acceleration method. The method consist of intersecting the
    // ray against the bbox of the mesh first, and if the ray inteesects the boudning box
    // then test if the ray intersects the mesh itsefl. It is obvious that the ray can't
    // intersect the mesh if it doesn't intersect its boudning volume (a box in this case)
    // [/comment]
    virtual bool intersect(const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId, float& tHit) const
    {
        const Mesh* intersectedMesh = nullptr;
        const Vec3f invDir = 1 / dir;
        const Vec3b sign(dir.x < 0, dir.y < 0, dir.z < 0);
        float t = kInfinity;
        for (const auto& mesh : meshes) {
            // If you intersect the box
            if (mesh->bbox.intersect(orig, invDir, sign, t)) {
                // Then test if the ray intersects the mesh and if does then first check
                // if the intersection distance is the nearest and if we pass that test as well
                // then update tNear variable with t and keep a pointer to the intersected mesh
                if (mesh->intersect(orig, dir, t) && t < tHit) {
                    tHit = t;
                    intersectedMesh = mesh.get();
                }
            }
        }

        // Return true if the variable intersectedMesh is not null, false otherwise
        return (intersectedMesh != nullptr);
    }
};

// [comment]
// Implementation of the Bounding Volume Hieratchy (BVH) acceleration structure
// [/comment]
class BVH : public AccelerationStructure
{
    static const uint8_t kNumPlaneSetNormals = 7;
    static const Vec3f planeSetNormals[kNumPlaneSetNormals];
    struct Extents
    {
        Extents()
        {
            for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i)
                d[i][0] = kInfinity, d[i][1] = -kInfinity;
        }
        void extendBy(const Extents& e)
        {

            for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i) {
                if (e.d[i][0] < d[i][0]) d[i][0] = e.d[i][0];
                if (e.d[i][1] > d[i][1]) d[i][1] = e.d[i][1];
            }
        }
        /* inline */
        Vec3f centroid() const
        {
            return Vec3f(
                d[0][0] + d[0][1] * 0.5,
                d[1][0] + d[1][1] * 0.5,
                d[2][0] + d[2][1] * 0.5);
        }
        bool intersect(const float*, const float*, float&, float&, uint8_t&) const;
        float d[kNumPlaneSetNormals][2];
        const Mesh* mesh;
    };

    struct Octree
    {
        Octree(const Extents& sceneExtents)
        {
            float xDiff = sceneExtents.d[0][1] - sceneExtents.d[0][0];
            float yDiff = sceneExtents.d[1][1] - sceneExtents.d[1][0];
            float zDiff = sceneExtents.d[2][1] - sceneExtents.d[2][0];
            float maxDiff = std::max(xDiff, std::max(yDiff, zDiff));
            Vec3f minPlusMax(
                sceneExtents.d[0][0] + sceneExtents.d[0][1],
                sceneExtents.d[1][0] + sceneExtents.d[1][1],
                sceneExtents.d[2][0] + sceneExtents.d[2][1]);
            bbox[0] = (minPlusMax - maxDiff) * 0.5;
            bbox[1] = (minPlusMax + maxDiff) * 0.5;
            root = new OctreeNode;
        }

        ~Octree() { deleteOctreeNode(root); }

        void insert(const Extents* extents) { insert(root, extents, bbox, 0); }
        void build() { build(root, bbox); };

        struct OctreeNode
        {
            OctreeNode* child[8] = { nullptr };
            std::vector<const Extents*> nodeExtentsList; // pointer to the objects extents
            Extents nodeExtents; // extents of the octree node itself
            bool isLeaf = true;
        };

        struct QueueElement
        {
            const OctreeNode* node; // octree node held by this element in the queue
            float t; // distance from the ray origin to the extents of the node
            QueueElement(const OctreeNode* n, float tn) : node(n), t(tn) {}
            // priority_queue behaves like a min-heap
            friend bool operator < (const QueueElement& a, const QueueElement& b) { return a.t > b.t; }
        };

        OctreeNode* root = nullptr; // make unique son don't have to manage deallocation
        BBox<> bbox;

    private:

        void deleteOctreeNode(OctreeNode*& node)
        {
            for (uint8_t i = 0; i < 8; i++) {
                if (node->child[i] != nullptr) {
                    deleteOctreeNode(node->child[i]);
                }
            }
            delete node;
        }

        void insert(OctreeNode*& node, const Extents* extents, const BBox<>& bbox, uint32_t depth)
        {
            if (node->isLeaf) {
                if (node->nodeExtentsList.size() == 0 || depth == 16) {
                    node->nodeExtentsList.push_back(extents);
                }
                else {
                    node->isLeaf = false;
                    // Re-insert extents held by this node
                    while (node->nodeExtentsList.size()) {
                        insert(node, node->nodeExtentsList.back(), bbox, depth);
                        node->nodeExtentsList.pop_back();
                    }
                    // Insert new extent
                    insert(node, extents, bbox, depth);
                }
            }
            else {
                // Need to compute in which child of the current node this extents should
                // be inserted into
                Vec3f extentsCentroid = extents->centroid();
                Vec3f nodeCentroid = (bbox[0] + bbox[1]) * 0.5;
                BBox<> childBBox;
                uint8_t childIndex = 0;
                // x-axis
                if (extentsCentroid.x > nodeCentroid.x) {
                    childIndex = 4;
                    childBBox[0].x = nodeCentroid.x;
                    childBBox[1].x = bbox[1].x;
                }
                else {
                    childBBox[0].x = bbox[0].x;
                    childBBox[1].x = nodeCentroid.x;
                }
                // y-axis
                if (extentsCentroid.y > nodeCentroid.y) {
                    childIndex += 2;
                    childBBox[0].y = nodeCentroid.y;
                    childBBox[1].y = bbox[1].y;
                }
                else {
                    childBBox[0].y = bbox[0].y;
                    childBBox[1].y = nodeCentroid.y;
                }
                // z-axis
                if (extentsCentroid.z > nodeCentroid.z) {
                    childIndex += 1;
                    childBBox[0].z = nodeCentroid.z;
                    childBBox[1].z = bbox[1].z;
                }
                else {
                    childBBox[0].z = bbox[0].z;
                    childBBox[1].z = nodeCentroid.z;
                }

                // Create the child node if it doesn't exsit yet and then insert the extents in it
                if (node->child[childIndex] == nullptr)
                    node->child[childIndex] = new OctreeNode;
                insert(node->child[childIndex], extents, childBBox, depth + 1);
            }
        }

        void build(OctreeNode*& node, const BBox<>& bbox)
        {
            if (node->isLeaf) {
                for (const auto& e : node->nodeExtentsList) {
                    node->nodeExtents.extendBy(*e);
                }
            }
            else {
                for (uint8_t i = 0; i < 8; ++i) {
                    if (node->child[i]) {
                        BBox<> childBBox;
                        Vec3f centroid = bbox.centroid();
                        // x-axis
                        childBBox[0].x = (i & 4) ? centroid.x : bbox[0].x;
                        childBBox[1].x = (i & 4) ? bbox[1].x : centroid.x;
                        // y-axis
                        childBBox[0].y = (i & 2) ? centroid.y : bbox[0].y;
                        childBBox[1].y = (i & 2) ? bbox[1].y : centroid.y;
                        // z-axis
                        childBBox[0].z = (i & 1) ? centroid.z : bbox[0].z;
                        childBBox[1].z = (i & 1) ? bbox[1].z : centroid.z;

                        // Inspect child
                        build(node->child[i], childBBox);

                        // Expand extents with extents of child
                        node->nodeExtents.extendBy(node->child[i]->nodeExtents);
                    }
                }
            }
        }
    };

    std::vector<Extents> extentsList;
    Octree* octree = nullptr;
public:
    BVH(std::vector<std::unique_ptr<const Mesh>>& m);
    bool intersect(const Vec3f&, const Vec3f&, const uint32_t&, float&) const;
    ~BVH() { delete octree; }
};

const Vec3f BVH::planeSetNormals[BVH::kNumPlaneSetNormals] = {
    Vec3f(1, 0, 0),
    Vec3f(0, 1, 0),
    Vec3f(0, 0, 1),
    Vec3f(sqrtf(3) / 3.f,  sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    Vec3f(-sqrtf(3) / 3.f,  sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    Vec3f(-sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    Vec3f(sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f)
};

BVH::BVH(std::vector<std::unique_ptr<const Mesh>>& m) : AccelerationStructure(m)
{
    Extents sceneExtents; // that's the extent of the entire scene which we need to compute for the octree
    extentsList.reserve(meshes.size());
    for (uint32_t i = 0; i < meshes.size(); ++i) {
        for (uint8_t j = 0; j < kNumPlaneSetNormals; ++j) {
            for (const auto vtx : meshes[i]->vertexPool) {
                float d = dot(planeSetNormals[j], vtx);
                // set dNEar and dFar
                if (d < extentsList[i].d[j][0]) extentsList[i].d[j][0] = d;
                if (d > extentsList[i].d[j][1]) extentsList[i].d[j][1] = d;
            }
        }
        sceneExtents.extendBy(extentsList[i]); // expand the scene extent of this object's extent
        extentsList[i].mesh = meshes[i].get(); // the extent itself needs to keep a pointer to the object its holds
    }

    // Now that we have the extent of the scene we can start building our octree
    // Using C++ make_unique function here but you don't need to, just to learn something... 
    octree = new Octree(sceneExtents);

    for (uint32_t i = 0; i < meshes.size(); ++i) {
        octree->insert(&extentsList[i]);
    }

    // Build from bottom up
    octree->build();
}

bool BVH::Extents::intersect(
    const float* precomputedNumerator,
    const float* precomputedDenominator,
    float& tNear,   // tn and tf in this method need to be contained
    float& tFar,    // within the range [tNear:tFar]
    uint8_t& planeIndex) const
{
    numRayBoundingVolumeTests++;
    for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i) {
        float tNearExtents = (d[i][0] - precomputedNumerator[i]) / precomputedDenominator[i];
        float tFarExtents = (d[i][1] - precomputedNumerator[i]) / precomputedDenominator[i];
        if (precomputedDenominator[i] < 0) std::swap(tNearExtents, tFarExtents);
        if (tNearExtents > tNear) tNear = tNearExtents, planeIndex = i;
        if (tFarExtents < tFar) tFar = tFarExtents;
        if (tNear > tFar) return false;
    }

    return true;
}

bool BVH::intersect(const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId, float& tHit) const
{
    tHit = kInfinity;
    const Mesh* intersectedMesh = nullptr;
    float precomputedNumerator[BVH::kNumPlaneSetNormals];
    float precomputedDenominator[BVH::kNumPlaneSetNormals];
    for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i) {
        precomputedNumerator[i] = dot(planeSetNormals[i], orig);
        precomputedDenominator[i] = dot(planeSetNormals[i], dir);
    }

    /*
    tNear = kInfinity; // set
    for (uint32_t i = 0; i < meshes.size(); ++i) {
        numRayVolumeTests++;
        float tn = -kInfinity, tf = kInfinity;
        uint8_t planeIndex;
        if (extents[i].intersect(precomputedNumerator, precomputedDenominator, tn, tf, planeIndex)) {
            if (tn < tNear) {
                intersectedMesh = meshes[i].get();
                tNear = tn;
                // normal = planeSetNormals[planeIndex];
            }
        }
    }
    */

    uint8_t planeIndex;
    float tNear = 0, tFar = kInfinity; // tNear, tFar for the intersected extents
    if (!octree->root->nodeExtents.intersect(precomputedNumerator, precomputedDenominator, tNear, tFar, planeIndex) || tFar < 0)
        return false;
    tHit = tFar;
    std::priority_queue<BVH::Octree::QueueElement> queue;
    queue.push(BVH::Octree::QueueElement(octree->root, 0));
    while (!queue.empty() && queue.top().t < tHit) {
        const Octree::OctreeNode* node = queue.top().node;
        queue.pop();
        if (node->isLeaf) {
            for (const auto& e : node->nodeExtentsList) {
                float t = kInfinity;
                if (e->mesh->intersect(orig, dir, t) && t < tHit) {
                    tHit = t;
                    intersectedMesh = e->mesh;
                }
            }
        }
        else {
            for (uint8_t i = 0; i < 8; ++i) {
                if (node->child[i] != nullptr) {
                    float tNearChild = 0, tFarChild = tFar;
                    if (node->child[i]->nodeExtents.intersect(precomputedNumerator, precomputedDenominator, tNearChild, tFarChild, planeIndex)) {
                        float t = (tNearChild < 0 && tFarChild >= 0) ? tFarChild : tNearChild;
                        queue.push(BVH::Octree::QueueElement(node->child[i], t));
                    }
                }
            }
        }
    }

    return (intersectedMesh != nullptr);
}

// [comment]
// Implementation of the Grid acceleration structure
// [/comment]
class Grid : public AccelerationStructure
{
    struct Cell
    {
        Cell() {}
        struct TriangleDesc
        {
            TriangleDesc(const Mesh* m, const uint32_t& t) : mesh(m), tri(t) {}
            const Mesh* mesh;
            uint32_t tri;
        };

        void insert(const Mesh* mesh, uint32_t t)
        {
            triangles.push_back(Grid::Cell::TriangleDesc(mesh, t));
        }

        bool intersect(const Vec3f&, const Vec3f&, const uint32_t&, float&, const Mesh*&) const;

        std::vector<TriangleDesc> triangles;
    };
public:
    Grid(std::vector<std::unique_ptr<const Mesh>>& m);
    ~Grid()
    {
        for (uint32_t i = 0; i < resolution[0] * resolution[1] * resolution[2]; ++i)
            if (cells[i] != NULL) delete cells[i];
        delete[] cells;
    }
    bool intersect(const Vec3f&, const Vec3f&, const uint32_t&, float&) const;
    Cell** cells;
    BBox<> bbox;
    Vec3<uint32_t> resolution;
    Vec3f cellDimension;
};

Grid::Grid(std::vector<std::unique_ptr<const Mesh>>& m) : AccelerationStructure(m)
{
    uint32_t totalNumTriangles = 0;
    for (const auto& m : meshes) {
        bbox.extendBy(m->bbox[0]);
        bbox.extendBy(m->bbox[1]);
        totalNumTriangles += m->numTriangles;
    }
    // Create the grid
    Vec3f size = bbox[1] - bbox[0];
    float cubeRoot = std::powf(totalNumTriangles / (size.x * size.y * size.z), 1. / 3.f);
    for (uint8_t i = 0; i < 3; ++i) {
        resolution[i] = std::floor(size[i] * cubeRoot);
        if (resolution[i] < 1) resolution[i] = 1;
        if (resolution[i] > 128) resolution[i] = 128;
    }
    cellDimension = size / resolution;

    // [comment]
    // Allocate memory - note that we don't create the cells yet at this
    // point but just an array of pointers to cell. We will create the cells
    // dynamically later when we are sure to insert something in them
    // [/comment]
    uint32_t numCells = resolution.x * resolution.y * resolution.z;
    cells = new Grid::Cell * [numCells];
    memset(cells, 0x0, sizeof(Grid::Cell*) * numCells);

    for (const auto& m : meshes) {
        for (uint32_t i = 0, off = 0; i < m->numTriangles; ++i, off += 3) {
            Vec3f min(kInfinity), max(-kInfinity);
            const Vec3f& v0 = m->vertexPool[m->triangleIndicesInVertexPool[off]];
            const Vec3f& v1 = m->vertexPool[m->triangleIndicesInVertexPool[off + 1]];
            const Vec3f& v2 = m->vertexPool[m->triangleIndicesInVertexPool[off + 2]];
            for (uint8_t j = 0; j < 3; ++j) {
                if (v0[j] < min[j]) min[j] = v0[j];
                if (v1[j] < min[j]) min[j] = v1[j];
                if (v2[j] < min[j]) min[j] = v2[j];
                if (v0[j] > max[j]) max[j] = v0[j];
                if (v1[j] > max[j]) max[j] = v1[j];
                if (v2[j] > max[j]) max[j] = v2[j];
            }
            // Convert to cell coordinates
            min = (min - bbox[0]) / cellDimension;
            max = (max - bbox[0]) / cellDimension;
            uint32_t zmin = clamp<uint32_t>(std::floor(min[2]), 0, resolution[2] - 1);
            uint32_t zmax = clamp<uint32_t>(std::floor(max[2]), 0, resolution[2] - 1);
            uint32_t ymin = clamp<uint32_t>(std::floor(min[1]), 0, resolution[1] - 1);
            uint32_t ymax = clamp<uint32_t>(std::floor(max[1]), 0, resolution[1] - 1);
            uint32_t xmin = clamp<uint32_t>(std::floor(min[0]), 0, resolution[0] - 1);
            uint32_t xmax = clamp<uint32_t>(std::floor(max[0]), 0, resolution[0] - 1);
            // Loop over the cells the triangle overlaps and insert
            for (uint32_t z = zmin; z <= zmax; ++z) {
                for (uint32_t y = ymin; y <= ymax; ++y) {
                    for (uint32_t x = xmin; x <= xmax; ++x) {
                        uint32_t index = z * resolution[0] * resolution[1] + y * resolution[0] + x;
                        if (cells[index] == NULL) cells[index] = new Grid::Cell;
                        cells[index]->insert(m.get(), i);
                    }
                }
            }
        }
    }
}

bool Grid::Cell::intersect(
    const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId,
    float& tHit, const Mesh*& intersectedMesh) const
{
    float uhit, vhit;
    for (uint32_t i = 0; i < triangles.size(); ++i) {
        // [comment]
        // Be sure that rayId is never 0 - because all mailbox values
        // in the array are initialized with 0 too
        // [/comment]
        if (rayId != triangles[i].mesh->mailbox[triangles[i].tri]) {
            triangles[i].mesh->mailbox[triangles[i].tri] = rayId;
            const Mesh* mesh = triangles[i].mesh;
            uint32_t j = triangles[i].tri * 3;
            const Vec3f& v0 = mesh->vertexPool[mesh->triangleIndicesInVertexPool[j]];
            const Vec3f& v1 = mesh->vertexPool[mesh->triangleIndicesInVertexPool[j + 1]];
            const Vec3f& v2 = mesh->vertexPool[mesh->triangleIndicesInVertexPool[j + 2]];
            float t, u, v;
            if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v)) {
                if (t < tHit) {
                    tHit = t;
                    uhit = u;
                    vhit = v;
                    intersectedMesh = triangles[i].mesh;
                }
            }
        }
    }

    return (intersectedMesh != nullptr);
}


bool Grid::intersect(const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId, float& tHit) const
{
    const Vec3f invDir = 1 / dir;
    const Vec3b sign(dir.x < 0, dir.y < 0, dir.z < 0);
    float tHitBox;
    if (!bbox.intersect(orig, invDir, sign, tHitBox)) return false;

    // initialization step
    Vec3i exit, step, cell;
    Vec3f deltaT, nextCrossingT;
    for (uint8_t i = 0; i < 3; ++i) {
        // convert ray starting point to cell coordinates
        float rayOrigCell = ((orig[i] + dir[i] * tHitBox) - bbox[0][i]);
        cell[i] = clamp<uint32_t>(std::floor(rayOrigCell / cellDimension[i]), 0, resolution[i] - 1);
        if (dir[i] < 0) {
            deltaT[i] = -cellDimension[i] * invDir[i];
            nextCrossingT[i] = tHitBox + (cell[i] * cellDimension[i] - rayOrigCell) * invDir[i];
            exit[i] = -1;
            step[i] = -1;
        }
        else {
            deltaT[i] = cellDimension[i] * invDir[i];
            nextCrossingT[i] = tHitBox + ((cell[i] + 1) * cellDimension[i] - rayOrigCell) * invDir[i];
            exit[i] = resolution[i];
            step[i] = 1;
        }
    }

    // Walk through each cell of the grid and test for an intersection if
    // current cell contains geometry
    const Mesh* intersectedMesh = nullptr;
    while (1) {
        uint32_t o = cell[2] * resolution[0] * resolution[1] + cell[1] * resolution[0] + cell[0];
        if (cells[o] != nullptr) {
            cells[o]->intersect(orig, dir, rayId, tHit, intersectedMesh);
            //if (intersectedMesh != nullptr) { ray.color = cells[o]->color; }
        }
        uint8_t k =
            ((nextCrossingT[0] < nextCrossingT[1]) << 2) +
            ((nextCrossingT[0] < nextCrossingT[2]) << 1) +
            ((nextCrossingT[1] < nextCrossingT[2]));
        static const uint8_t map[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };
        uint8_t axis = map[k];

        if (tHit < nextCrossingT[axis]) break;
        cell[axis] += step[axis];
        if (cell[axis] == exit[axis]) break;
        nextCrossingT[axis] += deltaT[axis];
    }

    return (intersectedMesh != nullptr);
}

// [comment]
// Main Render() function. Loop over each pixel in the image and trace a primary
// ray starting from the camera origin and passing through the current pixel. If they
// ray intersects geometry in the scene return some color (the color of the object)
// otherwise nothing (black or the color of the background)
// [/comment]
void render(const std::unique_ptr<AccelerationStructure>& accel, const Options& options)
{
    std::unique_ptr<Vec3f[]> buffer(new Vec3f[options.width * options.height]);
    Vec3f orig(0, 0, 5);
    matPointMult(options.cameraToWorld, orig);
    float scale = std::tan(degToRad<float>(options.fov * 0.5));
    float imageAspectRatio = options.width / static_cast<float>(options.height);
    assert(imageAspectRatio > 1);
    uint32_t rayId = 1; // Start at 1 not 0!! (see Grid code and mailboxing)
    for (uint32_t j = 0; j < options.height; ++j) {
        for (uint32_t i = 0; i < options.width; ++i) {
            Vec3f dir((2 * (i + 0.5f) / options.width - 1) * scale * imageAspectRatio,
                (1 - 2 * (j + 0.5) / options.height) * scale,
                -1);
            matVecMult(options.cameraToWorld, dir);
            normalize(dir);
            numPrimaryRays++;
            float tHit = kInfinity;
            buffer[j * options.width + i] = (accel->intersect(orig, dir, rayId++, tHit)) ? Vec3f(1) : Vec3f(0);
        }
    }

    // store to PPM file
    std::ofstream ofs;
    ofs.open("image.ppm");
    ofs << "P6\n" << options.width << " " << options.height << "\n255\n";
    for (uint32_t i = 0; i < options.width * options.height; ++i) {
        Vec3<uint8_t> pixRgb;
        pixRgb.x = static_cast<uint8_t>(255 * std::max(0.f, std::min(1.f, buffer[i].x)));
        pixRgb.y = static_cast<uint8_t>(255 * std::max(0.f, std::min(1.f, buffer[i].y)));
        pixRgb.z = static_cast<uint8_t>(255 * std::max(0.f, std::min(1.f, buffer[i].z)));
        ofs << pixRgb.x << pixRgb.y << pixRgb.z;
    }
    ofs.close();
}

void exportMesh(const std::vector<std::unique_ptr<const Mesh>>& meshes)
{
    std::ofstream f;
    f.open("mesh.obj");
    uint32_t k = 0, off = 0;
    for (const auto& mesh : meshes) {
        f << "g default" << std::endl;
        for (uint32_t i = 0; i < mesh->vertexPool.size(); ++i) {
            f << "v " << mesh->vertexPool[i].x << " " << mesh->vertexPool[i].y << " " << mesh->vertexPool[i].z << std::endl;
        }

        f << "g mesh" << k++ << std::endl;
        for (uint32_t i = 0; i < mesh->numTriangles; ++i) {
            f << "f " << mesh->triangleIndicesInVertexPool[i * 3] + 1 + off << " " <<
                mesh->triangleIndicesInVertexPool[i * 3 + 1] + 1 + off << " " <<
                mesh->triangleIndicesInVertexPool[i * 3 + 2] + 1 + off << std::endl;
        }
        off += mesh->vertexPool.size();
    }

    f.close();
}

int main(int argc, char** argv)
{
    std::vector<std::unique_ptr<const Mesh>> meshes;
    makeScene(meshes);

    // [comment]
    // Create the acceleration structure
    // [/comment]
#if defined(ACCEL_BBOX)
    std::unique_ptr<AccelerationStructure> accel(new BBoxAcceleration(meshes));
#elif defined(ACCEL_BVH)
    std::unique_ptr<AccelerationStructure> accel(new BVH(meshes));
#elif defined(ACCEL_GRID)
    std::unique_ptr<AccelerationStructure> accel(new Grid(meshes));
#else
    std::unique_ptr<AccelerationStructure> accel(new AccelerationStructure(meshes));
#endif

    //exportMesh(meshes);
    uint32_t numTriangles{ 0 };
    for (const auto& mesh : accel->meshes) {
        numTriangles += mesh->numTriangles;
    }

    Options options;
    using Time = std::chrono::high_resolution_clock;
    using fsec = std::chrono::duration<float>;

    auto t0 = Time::now();

    render(accel, options);

    auto t1 = Time::now();

    fsec fs = t1 - t0;
    std::cout << "Render time                                 | " << fs.count() << " sec" << std::endl;
    std::cout << "Total number of triangles                   | " << numTriangles << std::endl;
    std::cout << "Total number of primary rays                | " << numPrimaryRays << std::endl;
    std::cout << "Total number of ray-bbox tests              | " << numRayBBoxTests << std::endl;
    std::cout << "Total number of ray-boundvolume tests       | " << numRayBoundingVolumeTests << std::endl;
    std::cout << "Total number of ray-triangles tests         | " << numRayTriangleTests << std::endl;
    std::cout << "Total number of ray-triangles intersections | " << numRayTriangleIntersections << std::endl;

    return 0;
}
