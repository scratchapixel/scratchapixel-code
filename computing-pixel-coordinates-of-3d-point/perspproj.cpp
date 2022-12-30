//[header]
// This program renders a wireframe image of a 3D object whose description is stored
// in the program itself. The result is stored to a SVG file. To draw an image
// of each triangle making up that object, we project the vertices of each triangle
// onto the screen using perspective projection (effectively transforming the vertices
// world coordinates to 2D pixel coordinates). Triangles are stored in the SVG
// file by connecting their respective vertices to each other with lines.
//[/header]
//[compile]
// Download the perspproj.cpp and geometry.h files to the same folder.
// Open a shell/terminal, and run the following command where the files are saved:
//
// c++ perspproj.cpp  -o perspproj -std=c++11
//
// Run with: ./perspproj. Open the file ./proj.svg in any Internet browser to see
// the result.
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

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <cmath>

#include "geometry.h"

//[comment]
// List of vertices making up the object
//[/comment]
const Vec3f verts[146] = {
    {        0,    39.034,         0}, {  0.76212,    36.843,         0},
    {        3,    36.604,         0}, {        1,    35.604,         0}, 
    {   2.0162,    33.382,         0}, {        0,    34.541,         0}, 
    {  -2.0162,    33.382,         0}, {       -1,    35.604,         0}, 
    {       -3,    36.604,         0}, { -0.76212,    36.843,         0}, 
    {-0.040181,     34.31,         0}, {   3.2778,    30.464,         0}, 
    {-0.040181,    30.464,         0}, {-0.028749,    30.464,         0}, 
    {   3.2778,    30.464,         0}, {   1.2722,    29.197,         0}, 
    {   1.2722,    29.197,         0}, {-0.028703,    29.197,         0}, 
    {   1.2722,    29.197,         0}, {   5.2778,    25.398,         0}, 
    { -0.02865,    25.398,         0}, {   1.2722,    29.197,         0}, 
    {   5.2778,    25.398,         0}, {   3.3322,    24.099,         0}, 
    {-0.028683,    24.099,         0}, {   7.1957,    20.299,         0}, 
    { -0.02861,    20.299,         0}, {   5.2778,    19.065,         0}, 
    {-0.028663,    18.984,         0}, {   9.2778,    15.265,         0}, 
    {-0.028571,    15.185,         0}, {   9.2778,    15.265,         0}, 
    {   7.3772,    13.999,         0}, {-0.028625,    13.901,         0}, 
    {   9.2778,    15.265,         0}, {   12.278,    8.9323,         0}, 
    {-0.028771,    8.9742,         0}, {   12.278,    8.9323,         0}, 
    {   10.278,    7.6657,         0}, {-0.028592,    7.6552,         0}, 
    {   15.278,    2.5994,         0}, {-0.028775,    2.6077,         0}, 
    {   15.278,    2.5994,         0}, {   13.278,    1.3329,         0}, 
    {-0.028727,    1.2617,         0}, {   18.278,   -3.7334,         0}, 
    {   18.278,   -3.7334,         0}, {   2.2722,   -1.2003,         0}, 
    {-0.028727,   -1.3098,         0}, {   4.2722,        -5,         0}, 
    {   4.2722,        -5,         0}, {-0.028727,        -5,         0}, 
    {  -3.3582,    30.464,         0}, {  -3.3582,    30.464,         0}, 
    {  -1.3526,    29.197,         0}, {  -1.3526,    29.197,         0}, 
    {  -1.3526,    29.197,         0}, {  -5.3582,    25.398,         0}, 
    {  -1.3526,    29.197,         0}, {  -5.3582,    25.398,         0}, 
    {  -3.4126,    24.099,         0}, {   -7.276,    20.299,         0}, 
    {  -5.3582,    19.065,         0}, {  -9.3582,    15.265,         0}, 
    {  -9.3582,    15.265,         0}, {  -7.4575,    13.999,         0}, 
    {  -9.3582,    15.265,         0}, {  -12.358,    8.9323,         0}, 
    {  -12.358,    8.9323,         0}, {  -10.358,    7.6657,         0}, 
    {  -15.358,    2.5994,         0}, {  -15.358,    2.5994,         0}, 
    {  -13.358,    1.3329,         0}, {  -18.358,   -3.7334,         0}, 
    {  -18.358,   -3.7334,         0}, {  -2.3526,   -1.2003,         0}, 
    {  -4.3526,        -5,         0}, {  -4.3526,        -5,         0}, 
    {        0,     34.31,  0.040181}, {        0,    30.464,   -3.2778}, 
    {        0,    30.464,  0.040181}, {        0,    30.464,  0.028749}, 
    {        0,    30.464,   -3.2778}, {        0,    29.197,   -1.2722}, 
    {        0,    29.197,   -1.2722}, {        0,    29.197,  0.028703}, 
    {        0,    29.197,   -1.2722}, {        0,    25.398,   -5.2778}, 
    {        0,    25.398,   0.02865}, {        0,    29.197,   -1.2722}, 
    {        0,    25.398,   -5.2778}, {        0,    24.099,   -3.3322}, 
    {        0,    24.099,  0.028683}, {        0,    20.299,   -7.1957}, 
    {        0,    20.299,   0.02861}, {        0,    19.065,   -5.2778}, 
    {        0,    18.984,  0.028663}, {        0,    15.265,   -9.2778}, 
    {        0,    15.185,  0.028571}, {        0,    15.265,   -9.2778}, 
    {        0,    13.999,   -7.3772}, {        0,    13.901,  0.028625}, 
    {        0,    15.265,   -9.2778}, {        0,    8.9323,   -12.278}, 
    {        0,    8.9742,  0.028771}, {        0,    8.9323,   -12.278}, 
    {        0,    7.6657,   -10.278}, {        0,    7.6552,  0.028592}, 
    {        0,    2.5994,   -15.278}, {        0,    2.6077,  0.028775}, 
    {        0,    2.5994,   -15.278}, {        0,    1.3329,   -13.278}, 
    {        0,    1.2617,  0.028727}, {        0,   -3.7334,   -18.278}, 
    {        0,   -3.7334,   -18.278}, {        0,   -1.2003,   -2.2722}, 
    {        0,   -1.3098,  0.028727}, {        0,        -5,   -4.2722}, 
    {        0,        -5,   -4.2722}, {        0,        -5,  0.028727}, 
    {        0,    30.464,    3.3582}, {        0,    30.464,    3.3582}, 
    {        0,    29.197,    1.3526}, {        0,    29.197,    1.3526}, 
    {        0,    29.197,    1.3526}, {        0,    25.398,    5.3582}, 
    {        0,    29.197,    1.3526}, {        0,    25.398,    5.3582}, 
    {        0,    24.099,    3.4126}, {        0,    20.299,     7.276}, 
    {        0,    19.065,    5.3582}, {        0,    15.265,    9.3582}, 
    {        0,    15.265,    9.3582}, {        0,    13.999,    7.4575}, 
    {        0,    15.265,    9.3582}, {        0,    8.9323,    12.358}, 
    {        0,    8.9323,    12.358}, {        0,    7.6657,    10.358}, 
    {        0,    2.5994,    15.358}, {        0,    2.5994,    15.358}, 
    {        0,    1.3329,    13.358}, {        0,   -3.7334,    18.358}, 
    {        0,   -3.7334,    18.358}, {        0,   -1.2003,    2.3526}, 
    {        0,        -5,    4.3526}, {        0,        -5,    4.3526}
};

const uint32_t numTris = 128;

//[comment]
// Triangle index array. A triangle has 3 vertices. Each successive group of 3
// integers in this array represent the positions of the vertices in the vertex
// array making up one triangle of that object. For example, the first 3 integers
// from this array, 8/7/9 represent the positions of the vertices making up the
// the first triangle. You can access these vertices with the following code:
//
//     verts[8]; /* first vertex  */
//
//     verts[7]; /* second vertex */
//
//     verts[9]; /* third vertex  */
//
// 6/5/5 are the positions of the vertices in the vertex array making up the second
// triangle, and so on.
// To find the indices of the n-th triangle, use the following code:
//
//     tris[n * 3];     /* index of the first vertex in the verts array */
//
//     tris[n * 3 + 1]; /* index of the second vertexin the verts array */
//
//     tris[n * 3 + 2]; /* index of the third vertex in the verts array */
//[/comment]
const uint32_t tris[numTris * 3] = {
      8,   7,   9,   6,   5,   7,   4,   3,   5,   2,   1,   3,   0,   9,   1,
      5,   3,   7,   7,   3,   9,   9,   3,   1,  10,  12,  11,  13,  15,  14, 
     15,  13,  16,  13,  17,  16,  18,  20,  19,  17,  20,  21,  20,  23,  22, 
     20,  24,  23,  23,  26,  25,  24,  26,  23,  26,  27,  25,  26,  28,  27, 
     27,  30,  29,  28,  30,  27,  30,  32,  31,  30,  33,  32,  27,  30,  34, 
     32,  36,  35,  33,  36,  32,  36,  38,  37,  36,  39,  38,  38,  41,  40, 
     39,  41,  38,  41,  43,  42,  41,  44,  43,  44,  45,  43,  44,  47,  46, 
     44,  48,  47,  48,  49,  47,  48,  51,  50,  10,  52,  12,  13,  53,  54, 
     55,  17,  54,  13,  54,  17,  56,  57,  20,  17,  58,  20,  20,  59,  60, 
     20,  60,  24,  60,  61,  26,  24,  60,  26,  26,  61,  62,  26,  62,  28, 
     62,  63,  30,  28,  62,  30,  30,  64,  65,  30,  65,  33,  62,  66,  30, 
     65,  67,  36,  33,  65,  36,  36,  68,  69,  36,  69,  39,  69,  70,  41, 
     39,  69,  41,  41,  71,  72,  41,  72,  44,  44,  72,  73,  44,  74,  75, 
     44,  75,  48,  48,  75,  76,  48,  77,  51,  78,  80,  79,  81,  83,  82, 
     83,  81,  84,  81,  85,  84,  86,  88,  87,  85,  88,  89,  88,  91,  90, 
     88,  92,  91,  91,  94,  93,  92,  94,  91,  94,  95,  93,  94,  96,  95, 
     95,  98,  97,  96,  98,  95,  98, 100,  99,  98, 101, 100,  95,  98, 102, 
    100, 104, 103, 101, 104, 100, 104, 106, 105, 104, 107, 106, 106, 109, 108, 
    107, 109, 106, 109, 111, 110, 109, 112, 111, 112, 113, 111, 112, 115, 114, 
    112, 116, 115, 116, 117, 115, 116, 119, 118,  78, 120,  80,  81, 121, 122, 
    123,  85, 122,  81, 122,  85, 124, 125,  88,  85, 126,  88,  88, 127, 128, 
     88, 128,  92, 128, 129,  94,  92, 128,  94,  94, 129, 130,  94, 130,  96, 
    130, 131,  98,  96, 130,  98,  98, 132, 133,  98, 133, 101, 130, 134,  98, 
    133, 135, 104, 101, 133, 104, 104, 136, 137, 104, 137, 107, 137, 138, 109, 
    107, 137, 109, 109, 139, 140, 109, 140, 112, 112, 140, 141, 112, 142, 143, 
    112, 143, 116, 116, 143, 144, 116, 145, 119
};

//[comment]
// Compute the 2D pixel coordinates of a point defined in world space. This function
// requires the point original world coordinates of course, the world-to-camera
// matrix (which you can get from computing the inverse of the camera-to-world matrix,
// the matrix transforming the camera), the canvas dimension and the image width and
// height in pixels.
//
// Note that we don't check in this version of the function if the point is visible
// or not. If the absolute value of of any of the screen coordinates are greater
// that their respective maximum value (the canvas width for the x-coordinate,
// and the canvas height for the y-coordinate) then the point is not visible.
// When a SVG file is displayed to the screen, the application displaying the content
// of the file clips points and lines which are not visible. Thus we can store lines or point
// in the file whether they are visible or not. The final clipping will be done when the
// image is displayed to the screen.
//[/comment]
void computePixelCoordinates(
    const Vec3f pWorld,
    Vec2i &pRaster,
    const Matrix44f &worldToCamera,
    const float &canvasWidth,
    const float &canvasHeight,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight
)
{
    Vec3f pCamera;
    worldToCamera.multVecMatrix(pWorld, pCamera);
    Vec2f pScreen;
    pScreen.x = pCamera.x / -pCamera.z;
    pScreen.y = pCamera.y / -pCamera.z;
    Vec2f pNDC;
    pNDC.x = (pScreen.x + canvasWidth * 0.5) / canvasWidth;
    pNDC.y = (pScreen.y + canvasHeight * 0.5) / canvasHeight;
    pRaster.x = (int)(pNDC.x * imageWidth);
    pRaster.y = (int)((1 - pNDC.y) * imageHeight);
}

int main(int argc, char **argv)
{
    std::ofstream ofs;
    //[comment]
    // We will export the result to a SVG file.
    //[/comment]
    ofs.open("./proj.svg");
    ofs << "<svg version=\"1.1\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns=\"http://www.w3.org/2000/svg\" height=\"512\" width=\"512\">" << std::endl;
    //[comment]
    // We exported the camera matrix from Maya. We need to compute its inverse,
    // which is the matrix used in the computePixelCoordinates() function.
    //[/comment]
    Matrix44f cameraToWorld(0.871214, 0, -0.490904, 0, -0.192902, 0.919559, -0.342346, 0, 0.451415, 0.392953, 0.801132, 0, 14.777467, 29.361945, 27.993464, 1);
    Matrix44f worldToCamera = cameraToWorld.inverse();
    std::cerr << worldToCamera << std::endl;
    float canvasWidth = 2, canvasHeight = 2;
    uint32_t imageWidth = 512, imageHeight = 512;
    //[comment]
    // Loop over all the triangles making up the object, get the 3 vertices
    // making the current triangle, convert the 3 vertices wolrd coordinates
    // to 2D pixel coordinates using the computePixelCoordinates function.
    // Then finally store the result as 3 lines connecting the projected
    // vertices of the current triangle to each other.
    //[/comment]
    for (uint32_t i = 0; i < numTris; ++i) {
        const Vec3f &v0World = verts[tris[i * 3]];
        const Vec3f &v1World = verts[tris[i * 3 + 1]];
        const Vec3f &v2World = verts[tris[i * 3 + 2]];
        Vec2i v0Raster, v1Raster, v2Raster;
        computePixelCoordinates(v0World, v0Raster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
        computePixelCoordinates(v1World, v1Raster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
        computePixelCoordinates(v2World, v2Raster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
        std::cerr << v0Raster << ", " << v1Raster << ", " << v2Raster << std::endl;
        ofs << "<line x1=\"" << v0Raster.x << "\" y1=\"" << v0Raster.y << "\" x2=\"" << v1Raster.x << "\" y2=\"" << v1Raster.y << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />\n";
        ofs << "<line x1=\"" << v1Raster.x << "\" y1=\"" << v1Raster.y << "\" x2=\"" << v2Raster.x << "\" y2=\"" << v2Raster.y << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />\n";
        ofs << "<line x1=\"" << v2Raster.x << "\" y1=\"" << v2Raster.y << "\" x2=\"" << v0Raster.x << "\" y2=\"" << v0Raster.y << "\" style=\"stroke:rgb(0,0,0);stroke-width:1\" />\n";
    }
    ofs << "</svg>\n";
    ofs.close();

    return 0;
}
