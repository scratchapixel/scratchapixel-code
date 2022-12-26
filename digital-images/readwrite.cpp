//[header]
// Digital Images
//[/header]
//[compile]
// Download the readwrite.cpp and xmas.ppm file to a folder.
// Open a shell/terminal, and run the following command where the files is saved:
//
// clang++ -o readwrite readwrite.cpp -std=c++11 -O3
//
// You can use c++ if you don't use clang++
//
// Run with: ./readwrite. Open the resulting image (ppm) in Photoshop or any program
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

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <cassert>
#include <exception>

// [comment]
// The main Image class
// [/comment]
class Image
{
public:
    
    struct Rgb
    {
        Rgb() : r(0), g(0), b(0)  {}
        Rgb(float rr) : r(rr), g(rr), b(rr) {}
        Rgb(float rr, float gg, float bb) : r(rr), g(gg), b(bb) {}
        float r, g, b;
    };
    
    Image() : w(0), h(0), pixels(nullptr) {}
    Image(const unsigned int &_w, const unsigned int &_h) :
        w(_w), h(_h), pixels(nullptr)
    {
        pixels = new Rgb[w * h];
        for (int i = 0; i < w * h; ++i) pixels[i] = 0;
    }
    ~Image() { if (pixels != nullptr) delete [] pixels; }
    unsigned int w, h;
    Rgb *pixels;
};

// [comment]
// Save an image to PPM image file
// [/comment]
void savePPM(const Image &img, const char *filename)
{
    if (img.w == 0 || img.h == 0) { fprintf(stderr, "Can't save an empty image\n"); return; }
    std::ofstream ofs;
    try {
        ofs.open(filename, std::ios::binary); // need to spec. binary mode for Windows users
        if (ofs.fail()) throw("Can't open output file");
        ofs << "P6\n" << img.w << " " << img.h << "\n255\n";
        unsigned char r, g, b;
        // loop over each pixel in the image, clamp and convert to byte format
        for (int i = 0; i < img.w * img.h; ++i) {
            r = static_cast<unsigned char>(std::min(1.f, img.pixels[i].r) * 255);
            g = static_cast<unsigned char>(std::min(1.f, img.pixels[i].g) * 255);
            b = static_cast<unsigned char>(std::min(1.f, img.pixels[i].b) * 255);
            ofs << r << g << b;
        }
        ofs.close();
    }
    catch (const char *err) {
        fprintf(stderr, "%s\n", err);
        ofs.close();
    }
}

// [comment]
// Read a PPM image file
// [/comment]
Image readPPM(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios::binary); // need to spec. binary mode for Windows users
    Image img;
    try {
        if (ifs.fail()) { throw("Can't open input file"); }
        std::string header;
        int w, h, b;
        ifs >> header;
        if (strcmp(header.c_str(), "P6") != 0) throw("Can't read input file");
        ifs >> w >> h >> b;
        img.w = w; img.h = h;
        img.pixels = new Image::Rgb[w * h]; // this is throw an exception if bad_alloc
        ifs.ignore(256, '\n'); // skip empty lines in necessary until we get to the binary data
        unsigned char pix[3];
        // read each pixel one by one and convert bytes to floats
        for (int i = 0; i < w * h; ++i) {
            ifs.read(reinterpret_cast<char *>(pix), 3);
            img.pixels[i].r = pix[0] / 255.f;
            img.pixels[i].g = pix[1] / 255.f;
            img.pixels[i].b = pix[2] / 255.f;
        }
        ifs.close();
    }
    catch (const char *err) {
        fprintf(stderr, "%s\n", err);
        ifs.close();
    }

    return img;
}

// [comment]
// Read/Write an image stored in the PPM format
// [/comment]
int main(int argc, char **argv)
{
    Image I = readPPM("./xmas.ppm");
    savePPM(I, "./out.ppm");
  
    return 0;
}