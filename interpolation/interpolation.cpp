// (c) www.scratchapixel.com - 2024.
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// Contributors:
// - Scratchpixel
// - Kristopolous / Chris Mckenzie
// clang++ -std=c++11 -o interpolation interpolation.cpp -O3

#ifdef WIN32
#include "stdafx.h"
#endif

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <algorithm>

template<typename T>
class Color3 {
public:
    Color3() : r(0), g(0), b(0) {}
    Color3(T rr) : r(rr), g(rr), b(rr) {}
    Color3(T rr, T gg, T bb) : r(rr), g(gg), b(bb) {}
    Color3 operator * (const T &v) const {
        return Color3(r*v, g*v, b*v);
    }
    Color3 operator + (const Color3 &c) const {
        return Color3(r + c.r, g + c.g, b + c.b);
    }
    friend Color3 operator * (const float &f, const Color3 &c) {
        return Color3(c.r * f, c.g * f, c.b * f);
    }
    friend std::ostream & operator << (std::ostream &os, const Color3 &c) {
        os << c.r << " " << c.g << " " << c.b;
        return os;
    }
    float r, g, b;
};

typedef Color3<float> Color3f;

void saveToPPM(const char *fn, const Color3f *c, const int &width, const int &height) {
    std::ofstream ofs;
    // flags are necessary if your compile on Windows
    ofs.open(fn, std::ios::out | std::ios::binary);
    if (ofs.fail()) {
        fprintf(stderr, "ERROR: can't save image to file %s\n", fn);
    }
    else {
        ofs << "P6\n" << width << " " << height << "\n255\n";
        const Color3f *pc = c;
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                char r = static_cast<char>(std::min(255.f, 255 * pc->r + 0.5f));
                char g = static_cast<char>(std::min(255.f, 255 * pc->g + 0.5f));
                char b = static_cast<char>(std::min(255.f, 255 * pc->b + 0.5f));
                ofs << r << g << b;
                pc++;
            }
        }
    }
    ofs.close();
}

template<typename T>
T bilinear(
    const float &tx,
    const float &ty,
    const T &c00,
    const T &c10,
    const T &c01,
    const T &c11) {
#if 1
    T a = c00 * (1.f - tx) + c10 * tx;
    T b = c01 * (1.f - tx) + c11 * tx;
    return a * (1.f - ty) + b * ty;
#else
    return (1 - tx) * (1 - ty) * c00 +
        tx * (1 - ty) * c10 +
        (1.f - tx) * ty * c01 +
        tx * ty * c11;
#endif
}

#ifdef WIN32
    #define RANDFLOAT float(rand()) / RAND_MAX
#else
    #define RANDFLOAT drand48()
#endif

void testBilinearInterpolation() {
    // testing bilinear interpolation
    int imageWidth = 512;
    int gridSizeX = 9, gridSizeY = 9;
    Color3f *grid2d = new Color3f[(gridSizeX + 1) * (gridSizeY + 1)]; // lattices
                                                                      // fill grid with random colors
    Color3f c[4] = { Color3f(1,0,0), Color3f(0,1,0), Color3f(0,0,1), Color3f(1,1,0) };
    for (int j = 0, k = 0; j <= gridSizeY; ++j) {
        for (int i = 0; i <= gridSizeX; ++i, ++k) {
            grid2d[j * (gridSizeX + 1) + i] = Color3f(RANDFLOAT, RANDFLOAT, RANDFLOAT);
            printf("%d %d %f\n", i, j, grid2d[j * (gridSizeX + 1) + i].r);
        }
    }
    // now compute our final image using bilinear interpolation
    Color3f *imageData = new Color3f[imageWidth*imageWidth], *pixel = imageData;
    for (int j = 0; j < imageWidth; ++j) {
        for (int i = 0; i < imageWidth; ++i) {
            // convert i,j to grid coordinates
            float gx = i / float(imageWidth) * gridSizeX;	// be careful to interpolate boundaries
            float gy = j / float(imageWidth) * gridSizeY;	// be careful to interpolate boundaries
            int gxi = int(gx);
            int gyi = int(gy);
            const Color3f & c00 = grid2d[gyi * (gridSizeX + 1) + gxi];
            const Color3f & c10 = grid2d[gyi * (gridSizeX + 1) + (gxi + 1)];
            const Color3f & c01 = grid2d[(gyi + 1) * (gridSizeX + 1) + gxi];
            const Color3f & c11 = grid2d[(gyi + 1) * (gridSizeX + 1) + (gxi + 1)];
            *(pixel++) = bilinear<Color3f>(gx - gxi, gy - gyi, c00, c10, c01, c11);
        }
    }
    saveToPPM("./bilinear.ppm", imageData, imageWidth, imageWidth);
    // uncomnent this code if you want to see what the input colors look like
    pixel = imageData;
    int cellsize = imageWidth / (gridSizeX);
    fprintf(stderr, "%d\n", cellsize);
    for (int j = 0; j < imageWidth; ++j) {
        for (int i = 0; i < imageWidth; ++i) {
            float gx = (i + cellsize / 2) / float(imageWidth);
            float gy = (j + cellsize / 2) / float(imageWidth);
            int gxi = static_cast<int>(gx * gridSizeX);
            int gyi = static_cast<int>(gy * gridSizeY);
            *pixel = grid2d[gyi * (gridSizeX + 1) + gxi];
            int mx = (i + cellsize / 2) % cellsize;
            int my = (j + cellsize / 2) % cellsize;
            int ma = cellsize / 2 + 2, mb = cellsize / 2 - 2;
            if (mx < ma && mx > mb && my < ma && my > mb)
                *pixel = Color3f(0, 0, 0);
            pixel++;
        }
    }
    saveToPPM("./inputbilinear1.ppm", imageData, imageWidth, imageWidth);
    delete[] imageData;
}

#define IX(size, i, j, k) ( (i) * size * size + (j) * size + k )

/**
 * Trilinear interpolation example. We take a cube of size "gridsize" and then 
 * "upscale" it to scale * gridsize. To evaluate the result of a random 
 * point within the grid, we pick the 8 point's neighbor cells and trilinearly 
 * interpolate the results. Each of the results get written to a sequentially 
 * named ppm file. They can be viewed, in an animation that cycles through the 
 * slices on the command line using a tool like "mpv" like so:
 *
 * mpv --speed=10 --image-display-duration=0.1 trilinear-slice-*.ppm
 *
 * *or* if you have imagemagick you can make it into an animation like so: 
 *
 * convert trilinear-slice-*.ppm demo.gif
 *
 */
void testTrilinearuint32_terpolation() {
    //
    // Note that the size is gridSize ^ 3 and correspondingly, since we "upscale"
	// in each dimension, you will have (gridSize * scale) ^ 3 for the output 
	// image. Computers are fast, but keeping these values under 30 is advisable.
    //
    uint32_t gridSize = 10;
    uint32_t scale = 5;

    uint32_t src_num_verts = gridSize + 1;
    uint32_t target_num_verts = src_num_verts * scale;
    Color3f *grid3d = new Color3f[src_num_verts * src_num_verts * src_num_verts];
    Color3f *imageData[src_num_verts * scale]; 

    for (uint32_t i = 0; i < src_num_verts; ++i) {
      for(uint32_t x = 0 ; x < scale; x++) {
        imageData[i * scale + x] = new Color3f[src_num_verts * src_num_verts * scale * scale];
      }
      for (uint32_t j = 0; j < src_num_verts; ++j) {
        for (uint32_t k = 0; k < src_num_verts; ++k) {
          grid3d[IX(src_num_verts, i, j, k)] = Color3f(RANDFLOAT, RANDFLOAT, RANDFLOAT);
        }
      }
    }

    // uint32_terpolate grid data, we assume the grid is a unit cube
    float gx, gy, gz;
    uint32_t  gxi, gyi, gzi;
    float tx, ty, tz;

    for (uint32_t x = 0; x < target_num_verts; ++x) {
      gx = float(x) / scale;
      gxi = uint32_t(gx); tx = gx - gxi;

      for (uint32_t y = 0; y < target_num_verts; ++y) {
        gy = float(y) / scale;
        gyi = uint32_t(gy); ty = gy - gyi;

        for (uint32_t z = 0; z < target_num_verts; ++z) {
          gz = float(z) / scale;
          gzi = uint32_t(gz); tz = gz - gzi;

          const Color3f & c000 = grid3d[IX(src_num_verts, gxi, gyi, gzi)];
          const Color3f & c001 = grid3d[IX(src_num_verts, gxi, gyi, gzi + 1)];
          const Color3f & c010 = grid3d[IX(src_num_verts, gxi, gyi + 1, gzi)];
          const Color3f & c011 = grid3d[IX(src_num_verts, gxi, gyi + 1, gzi + 1)];

          const Color3f & c100 = grid3d[IX(src_num_verts, gxi + 1, gyi, gzi)];
          const Color3f & c101 = grid3d[IX(src_num_verts, gxi + 1, gyi, gzi + 1)];
          const Color3f & c110 = grid3d[IX(src_num_verts, gxi + 1, gyi + 1, gzi)];
          const Color3f & c111 = grid3d[IX(src_num_verts, gxi + 1, gyi + 1, gzi + 1)];
#if 1		
          // It's one surface than the other 
          Color3f e = bilinear(tz, ty, c000, c001, c010, c011);
          Color3f f = bilinear(tz, ty, c100, c101, c110, c111);

          // And then the percentage between them
          Color3f g = e * (1 - tx) + f * tx;

          imageData[x][bsz * y + z] = g;
#else
        Color3f g =
            (1 - tx)*(1 - ty)*(1 - tz)*c000 +
            tx*(1 - ty)*(1 - tz)*c100 +
            (1 - tx)*ty*(1 - tz)*c010 +
            tx*ty*(1 - tz)*c110 +
            (1 - tx)*(1 - ty)*tz*c001 +
            tx*(1 - ty)*tz*c101 +
            (1 - tx)*ty*tz*c011 +
            tx*ty*tz*c111;
#endif
        }
      }
    }
    delete[] grid3d;

    char fName[100] = {0};
    for (uint32_t k = 0; k < gridSize * scale; ++k) {
      spruint32_tf(fName, "trilinear-slice-%04d.ppm", k);
      saveToPPM(fName, imageData[k], bsz, bsz);
      delete imageData[k];
    }
}

int main(int argc, char **argv)
{
#ifdef WIN32
    srand(1308);
#else
    srand48(1308);
#endif
    testBilinearInterpolation();
    testTrilinearInterpolation();
    return 0;
}
