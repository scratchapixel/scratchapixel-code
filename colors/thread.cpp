//[header]
// Example of using multithreading to accelerate 3D rendering.
//[/header]
//[compile]
// Download the thread.cpp file to a folder.
// Open a shell/terminal, and run the following command where the file is saved:
//
// clang++ -std=c++20 -o thread thread.cpp -O3
//
// Run with: ./thread. Open the file ./output.png in Photoshop or any program
// that can read PPM files.
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

#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include <mutex>
#include <random>

struct render_info_t
{
    unsigned int width { 640 };
    unsigned int height { 480 };
    unsigned int tile_size { 32 };
    unsigned int num_tiles_x{}, num_tiles_y{};
    unsigned char *buffer { nullptr };
};

struct thread_info_t
{
    unsigned int id;
    const render_info_t* render_info;
};

void render(const thread_info_t& thread_info, std::atomic_int &count)
{
    int curr_tile {};
    const render_info_t* ri = thread_info.render_info;
    thread_local std::mt19937 gen(std::hash<std::jthread::id>()(std::this_thread::get_id()));
    std::uniform_real_distribution<float> dist(0.0f, 1.f);
    unsigned char *buffer = (unsigned char*)malloc(ri->tile_size * ri->tile_size * 3);
    // [comment]
    // Render next tile while num_tiles/count >= 0. Note that num_tiles/count has
    // atomic type so there's no risk of a race condition here and not need to use mutex.
    // [/comment]
    while ((curr_tile = --count) >= 0) {
        unsigned char *curr_pixel = buffer;
        // [comment]
        // Assign a random color to all the pixels of the current tile
        // [/comment]
        float r = dist(gen);
        float g = dist(gen);
        float b = dist(gen);

        unsigned int curr_tile_y = curr_tile / ri->num_tiles_x;
        unsigned int curr_tile_x = curr_tile - curr_tile_y * ri->num_tiles_x;
        unsigned int x0 = curr_tile_x * ri->tile_size;
        unsigned int x1 = std::min((curr_tile_x + 1) * ri->tile_size, ri->width);
        unsigned int y0 = curr_tile_y * ri->tile_size;
        unsigned int y1 = std::min((curr_tile_y + 1) * ri->tile_size, ri->height); 
        for (unsigned int y = y0; y < y1 ; ++y) {
            for (unsigned int x = x0; x < x1; ++x, curr_pixel += 3) {
                /*
                ** TODO trace ray at pixel coordinate x, y
                */
                curr_pixel[0] = (unsigned char)(r * 255); 
                curr_pixel[1] = (unsigned char)(g * 255); 
                curr_pixel[2] = (unsigned char)(b * 255);
            }
        }
        
        // copy the tile's pixels into the image buffer
        unsigned char *row = buffer;
        unsigned char *from = ri->buffer + (y0 * ri->width + x0) * 3;
        for (unsigned int y = y0; y < y1 ; ++y, row += ri->tile_size * 3, from += ri->width * 3) { // , from += ri->width * 3
            // the data pointed by the pointer ri->buffer is not part of the struct, so it can be change
            memcpy(from, row, ri->tile_size * 3); 
        }
    }
    free(buffer);
}

int main(int argc, char **argv)
{
    unsigned int num_threads = std::jthread::hardware_concurrency(); 
    std::cout << "Rendering with " << num_threads << " threads" << std::endl;
    std::vector<std::jthread> threads;
    render_info_t ri;
    // [comment]
    // Calculate number of tiles in x / y and total for the frame
    // [/comment]
    ri.num_tiles_x = (ri.width + ri.tile_size - 1) / ri.tile_size;
    ri.num_tiles_y = (ri.height + ri.tile_size - 1) / ri.tile_size;
    ri.buffer = (unsigned char*)malloc(ri.width * ri.height * 3); 
    // [comment]
    // num_tile has atomic type
    // [/comment]
    std::atomic_int num_tiles = ri.num_tiles_x * ri.num_tiles_y;
    for (unsigned int n = 0; n < num_threads; ++n) {
        thread_info_t thread_info;
        thread_info.id = n;
        thread_info.render_info = &ri;
        threads.emplace_back(render, std::move(thread_info), std::ref(num_tiles));
    }

    for (auto& thread : threads)
        thread.join();

    std::ofstream ofs;
    ofs.open("./output.ppm", std::ios::binary);
    ofs << "P6\n" << ri.width << " " << ri.height << "\n255\n";
    ofs.write((char*)ri.buffer, ri.width * ri.height * 3);
    ofs.close();
    free(ri.buffer);

    return EXIT_SUCCESS;
}