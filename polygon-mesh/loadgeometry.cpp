//[header]
// A simple program to load geometry data from a file
//[/header]
//[compile]
// Download the loadgeometry.cpp, geometry.h and test.geo files to a folder.
// Open a shell/terminal, and run the following command where the files is saved:
//
// c++ -o loadgeometry loadgeometry.cpp -O3 -std=c++11
//
// Run with: ./loadgeometry.
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

#include <fstream>
#include <sstream>
#include <vector>
#include <cassert>
#include "geometry.h"

void createMesh(
    const uint32_t nfaces,
    std::unique_ptr<uint32_t []> &faceIndex,
    const uint32_t &vertsIndexArraySize,
    std::unique_ptr<uint32_t []> &vertsIndex,
    const uint32_t &versArraySize,
    std::unique_ptr<Vec3f []> &verts,
    std::unique_ptr<Vec3f []> &normals,
    std::unique_ptr<Vec2f []> &st)
{}

// [comment]
// Structure of the geo file (ascii):
//
//     * nfaces (integer)
//
//     * face index array (array of integers)
//
//     * vertex index array (array of integers)
//
//     * vertex array (array of Vec3f, coordinates are seperated by a space)
//
//     * normal araay (array of Vec3f, coordinates are seperated by a space)
//
//     * texture coordinates array (array of Vec2f, coordinates are seperated by a space)
// [/comment]
void loadGeoFile(const char *file)
{
    std::ifstream ifs;
    try {
        ifs.open(file);
        if (ifs.fail()) throw;
        std::stringstream ss;
        ss << ifs.rdbuf();
        uint32_t numFaces;
        ss >> numFaces;
        std::cerr << "Mesh has " << numFaces << " faces " << std::endl;
        std::unique_ptr<uint32_t []> faceIndex(new uint32_t[numFaces]);
        uint32_t vertsIndexArraySize = 0;
        // reading face index array
        for (uint32_t i = 0; i < numFaces; ++i) {
            ss >> faceIndex[i];
            vertsIndexArraySize += faceIndex[i];
            std::cerr << faceIndex[i] << std::endl;
        }
        std::cerr << "Verts index array size " << vertsIndexArraySize << std::endl;
        std::unique_ptr<uint32_t []> vertsIndex(new uint32_t[vertsIndexArraySize]);
        uint32_t vertsArraySize = 0;
        // reading vertex index array
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> vertsIndex[i];
            if (vertsIndex[i] > vertsArraySize) vertsArraySize = vertsIndex[i];
            std::cerr << vertsIndex[i] << std::endl;
        }
        vertsArraySize += 1;
        std::cerr << "Max verts index " << vertsArraySize << std::endl;
        // reading vertices
        std::unique_ptr<Vec3f []> verts(new Vec3f[vertsArraySize]);
        for (uint32_t i = 0; i < vertsArraySize; ++i) {
            ss >> verts[i].x >> verts[i].y >> verts[i].z;
            std::cerr << verts[i] << std::endl;
        }
        // reading normals
        std::cerr << "Reading normals\n";
        std::unique_ptr<Vec3f []> normals(new Vec3f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> normals[i].x >> normals[i].y >> normals[i].z;
            std::cerr << normals[i] << std::endl;
        }
        // reading st coordinates
        std::cerr << "Reading texture coordinates\n";
        std::unique_ptr<Vec2f []> st(new Vec2f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> st[i].x >> st[i].y;
            std::cerr << st[i] << std::endl;
        }
        
        createMesh(numFaces, faceIndex, vertsIndexArraySize, vertsIndex, vertsArraySize, verts, normals, st);
    }
    catch (...) {
        ifs.close();
    }
    ifs.close();
}

int main(int argc, char **argv)
{
    loadGeoFile("./test.geo");

    return 0;
}