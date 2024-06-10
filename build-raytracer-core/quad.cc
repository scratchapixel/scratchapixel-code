#define _USE_MATH_DEFINES
#include <memory>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>

constexpr uint32_t width = 512, height = 512;
uint32_t radius = 128;
int32_t center[2] = {width/2, height/2};
uint32_t max_depth = 4;
uint8_t quad_buf[width * height];
uint32_t total_num_illum_pixels = 0;

bool ContainsWhitePixelsOnly(const uint8_t* buf, uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
	for (uint32_t j = y; j < y + h; ++j) {
		for (uint32_t i = x; i < x + w; ++i)
			if (buf[j * width + i] == 0)
				return false;
	}
	return true;
}

void DrawQuads(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
	for (uint32_t i = x; i < x + w; ++i) {
		quad_buf[y * width + i] = 128;
		quad_buf[std::min(height - 1, y + h) * width + i] = 128;
	}
	for (uint32_t j = y + 1; j < y + h; ++j) {
		quad_buf[j * width + x] = 128;
		quad_buf[j * width + std::min(width - 1, x + w)] = 128;
	}
}

void BuildQuadTree(const uint8_t* buf, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t depth) {
	std::cerr << "Depth " << depth << " " << x << " " << y << " " << w << " " << h << std::endl;
	if (depth == 0) {
		DrawQuads(x, y, w, h);
		return;
	}
	if (ContainsWhitePixelsOnly(buf, x, y, w, h)) {
		DrawQuads(x, y, w, h);
		total_num_illum_pixels += w * h;
		return;
	}
	uint32_t mid_x = w / 2, mid_y = h / 2;
	//DrawQuads(x, y, w, h);
	//if (depth == 4) return;
	BuildQuadTree(buf, x, y, mid_x, mid_y, depth - 1);
	BuildQuadTree(buf, x, y + mid_y, mid_x, h - mid_y, depth - 1);
	BuildQuadTree(buf, x + mid_x, y, w - mid_x, mid_y, depth - 1);
	BuildQuadTree(buf, x + mid_x, y + mid_y, w - mid_x, h - mid_y, depth - 1);
}

int main() {
	std::unique_ptr<uint8_t[]> buf = std::make_unique<uint8_t[]>(width * height);
	std::memset(buf.get(), 0xFF, width * height);
	std::memset(quad_buf, 0x0, width * height);
	for (int32_t j = 0; j < height; ++j) {
		for (int32_t i = 0; i < width; ++i) {
			if ((i - center[0]) * (i - center[0]) + (j - center[1]) * (j - center[1]) <= radius * radius)
				buf[j * width + i] = 0;
		}
	}
	std::cerr << "All good...\n";
	BuildQuadTree(buf.get(), 0, 0, width, height, max_depth);
	std::cerr << "Writing quad\n";
	std::ofstream ofs("./quad.ppm", std::ios::binary);
	ofs << "P5\n" << width << " " << height << "\n255\n";
	ofs.write((char*)quad_buf, width * height);
	ofs.close();
	std::cerr << "Result: " << ((100.f * total_num_illum_pixels) / (width * height)) << std::endl; 
	std::cerr << 100 * (1 - M_PI * radius * radius / (float)(width * height)) << std::endl;		
	return 0;
}
