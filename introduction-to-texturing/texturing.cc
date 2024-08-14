#define _CRT_SECURE_NO_WARNINGS

#include <cstdlib>
#include <cstdio>
#include <string.h>
#include <cassert>
#include <math.h> // For fminf and fmaxf

struct point2i { int x, y; };
struct point2f { float x, y; };
struct point3f { float x, y, z; };
struct normal3f { float x, y, z; };
struct color3f { float x, y, z; };
struct texcoord2f { float s, t; };

struct image {
	int width;
	int height;
	unsigned char* data;
};

struct shader {
	union color {
		color3f constant_value;
		struct image* image_ptr;
	} color;
};

struct mesh {
	point3f* points;
	//int* faceVertexCounts;
	int* face_vertex_indices;
	struct {
		texcoord2f* coords;
		int* indices;
	} st;
	int num_triangles;
	int num_points;
	normal3f* normals;

	struct shader* shader;
};

struct context {
	struct extent {
		int width;
		int height;
	} extent;
	float focal_length;
	struct aperture {
		float width;
		float height;
	} aperture;
	float znear, zfar;
	struct screen_coordinates {
		float r, l, t, b;
	} screen_coordinates;
	float* depth_buffer;
	color3f* color_buffer;
	float world_to_cam[16] = {
		0.371368000,  0.0626859, -0.9263670, 0, 
		0.000000000,  0.9977180,  0.0675142, 0, 
		0.928486000, -0.0250726,  0.3705200, 0, 
		0.000639866, -0.8131160, -7.4020440, 1};
};

static inline void point_mat_mult(const point3f* const p, const float* m, point3f* xp) {
    float x = m[0] * p->x + m[4] * p->y + m[8] * p->z + m[12];
    float y = m[1] * p->x + m[5] * p->y + m[9] * p->z + m[13];
    float z = m[2] * p->x + m[6] * p->y + m[10] * p->z + m[14];
    xp->x = x; 
    xp->y = y; 
    xp->z = z;
}

void set_texture(struct image* const image, const char* filename) {
	FILE* file = fopen(filename, "rb");
	char format[3];
	int bpp;
	fscanf(file, "%2s", format); // content should be P6
	fscanf(file, "%d %d", &image->width, &image->height);
	fscanf(file, "%d", &bpp);
	fgetc(file);
	image->data = (unsigned char*)malloc(image->width * image->height * 3);
	fread(image->data, 3, image->width * image->height, file);
	fclose(file);
}

static void create_mesh(struct context* const context, struct mesh* const mesh) {
#include "sphere.h"
	int num_faces = sizeof(faceVertexCounts) / sizeof(faceVertexCounts[0]);
	mesh->num_points = sizeof(points) / sizeof(points[0]);
	fprintf(stderr, "num points %d\n", mesh->num_points);
	mesh->points = (point3f*)malloc(sizeof(points));
	point3f* pts = mesh->points;
	memcpy((char*)pts, points, mesh->num_points * sizeof(point3f));
	
	for (int i = 0; i < mesh->num_points; ++i, ++pts) {
		point_mat_mult(pts, context->world_to_cam, pts);
		//fprintf(stderr, "emit -o \"part\" -pos %f %f %f;\n", pts->x, pts->y, pts->z);
	}
	//abort();

	mesh->num_triangles = 0;
	for (int i = 0; i < num_faces; ++i) {
		mesh->num_triangles += faceVertexCounts[i] - 2;
	}
	assert(mesh->num_triangles);

	mesh->face_vertex_indices = (int*)malloc(mesh->num_triangles * 3 * sizeof(int));
	mesh->st.indices = (int*)malloc(mesh->num_triangles * 3 * sizeof(int));
	
	int vert_index_array_size = sizeof(faceVertexIndices) / sizeof(faceVertexIndices[0]);
	int tex_index_array_size = sizeof(indices) / sizeof(indices[0]);
	int tex_coord_array_size = sizeof(st) / sizeof(st[0]);
	
	mesh->st.coords = (texcoord2f*)malloc(sizeof(st));
	memcpy((char*)mesh->st.coords, st, tex_coord_array_size * sizeof(texcoord2f));

	int vi[3], ti[3], index_offset = 0;
	int* pvi = mesh->face_vertex_indices;
	int* pti = mesh->st.indices;
	for (int i = 0; i < num_faces; ++i) {
		vi[0] = faceVertexIndices[index_offset]; ti[0] = indices[index_offset++];
		vi[1] = faceVertexIndices[index_offset]; ti[1] = indices[index_offset++];
		for (int j = 0; j < faceVertexCounts[i] - 2; ++j) {
			vi[2] = faceVertexIndices[index_offset]; ti[2] = indices[index_offset++];
			memcpy((char*)pvi, vi, sizeof(int) * 3);
			memcpy((char*)pti, ti, sizeof(int) * 3);
			vi[1] = vi[2];
			ti[1] = ti[2];
			pvi += 3; pti += 3;
		}
		
	}
	mesh->shader = (shader*)malloc(sizeof(shader));
	mesh->shader->color.image_ptr = (image*)malloc(sizeof(image));
	mesh->shader->color.image_ptr->data = NULL;
	set_texture(mesh->shader->color.image_ptr, "./pixar-texture3.pbm");
}

static void destroy_mesh(struct mesh* mesh) {
	free(mesh->points);
	free(mesh->face_vertex_indices);
	free(mesh->st.indices);
	free(mesh->st.coords);
	if (mesh->shader->color.image_ptr != NULL) {
		if (mesh->shader->color.image_ptr->data != NULL) {
			free(mesh->shader->color.image_ptr->data);
		}
		free(mesh->shader->color.image_ptr);
	}
	free(mesh);
}

static const point2f sample_pattern[] = {{0.25, 0.25},{0.75, 0.25},{0.25, 0.75},{0.75, 0.75}};
static const int num_samples = sizeof(sample_pattern) / sizeof(sample_pattern[0]);

static void init(struct context* context) {
	context->extent = {960, 540};
	context->focal_length = 35;
	context->aperture = {36.f, 24.f};
	context->znear = 0.1;
	context->zfar = 10;

	// image aspect ratio = 1.77, film aspect ratio = 1.5 
	// so stretch left/right coordinates
	float device_aspect_ratio = context->extent.width / (float)context->extent.height;;
	float film_aspect_ratio = context->aperture.width / (float)context->aperture.height;
	float xscale = device_aspect_ratio / film_aspect_ratio;
	float fac = 0.5f / context->focal_length  * context->znear;
	context->screen_coordinates.r = context->aperture.width * fac;
	context->screen_coordinates.t = context->aperture.height * fac;
	context->screen_coordinates.r *= xscale;
	context->screen_coordinates.l = -context->screen_coordinates.r;
	context->screen_coordinates.b = -context->screen_coordinates.t;
	fprintf(stderr, "l: %f, r: %f, t: %f, b: %f\n", 
		context->screen_coordinates.l, context->screen_coordinates.r, 
		context->screen_coordinates.t, context->screen_coordinates.b
	);
}

static void prepare_buffers(struct context* context) {
	int array_size = context->extent.width * context->extent.height;

	context->depth_buffer = (float*)malloc(sizeof(float) * num_samples * array_size);
	context->color_buffer = (color3f*)malloc(sizeof(color3f) * num_samples * array_size);

	for (int i = 0; i < array_size * num_samples; ++i) {
		context->color_buffer[i] = {0.18,0.18,0.18};
		context->depth_buffer[i] = context->zfar;
	}
}

static inline void persp_divide(point3f* p, const float& znear) {
	p->x = p->x / -p->z * znear;
    p->y = p->y / -p->z * znear;
    p->z = -p->z;
}

static inline void to_raster(const struct context::screen_coordinates& coords, const struct context::extent& extent, point3f* p) {
	p->x = 2 * p->x / (coords.r - coords.l) - (coords.r + coords.l) / (coords.r - coords.l);
	p->y = 2 * p->y / (coords.t - coords.b) - (coords.t + coords.b) / (coords.t - coords.b);

	p->x = (p->x + 1) / 2 * extent.width;
	p->y = (1 - p->y) / 2 * extent.height;
	//p->z = 1 / p->z;
}

static inline void tri_bbox(const point3f* p0, const point3f* p1, const point3f* p2, float* const bbox) {
	bbox[0] = fminf(fminf(p0->x, p1->x), p2->x); // Min x-coordinate
    bbox[1] = fminf(fminf(p0->y, p1->y), p2->y); // Min y-coordinate
    bbox[2] = fmaxf(fmaxf(p0->x, p1->x), p2->x); // Max x-coordinate
    bbox[3] = fmaxf(fmaxf(p0->y, p1->y), p2->y); // Max y-coordinate
}

static inline int min(int lo, int v) { return (lo < v) ? lo : v; }
static inline int max(int hi, int v) { return (hi > v) ? hi : v; }
static inline float mix(float a, float b, float t) { return a * (1 - t) + b * t; }
static inline float edge(const point3f* const a, const point3f* const b, const point3f* const test) {
	return (test->x - a->x) * (b->y - a->y) - (test->y - a->y) * (b->x - a->x);
}

static void shade(const struct shader* shader, texcoord2f st, color3f* ci) {
	if (shader->color.image_ptr != NULL) {
		const image* const image = shader->color.image_ptr;
		point2i texel;
		texel.x = (int)fminf(st.s * image->width, image->width - 1);
		texel.y = (int)fminf((1 - st.t) * image->height, image->height - 1);
		unsigned char texel_color[3];
		memcpy(texel_color, image->data + (texel.y * image->width + texel.x) * 3, 3);
		ci->x = texel_color[0] / 255.f;
		ci->y = texel_color[1] / 255.f;
		ci->z = texel_color[2] / 255.f;
		return;
	}
	ci->x = shader->color.constant_value.x;
	ci->y = shader->color.constant_value.y;
	ci->z = shader->color.constant_value.z;
}

static inline void rasterize(int x0, int y0, int x1, int y1, 
							 const point3f* const p0, const point3f* const p1, const point3f* const p2, 
							 const texcoord2f* const st0, const texcoord2f* const st1, const texcoord2f* const st2,
							 const mesh* const mesh,
							 struct context* context) {
	float area = edge(p0, p1, p2);
	float rcp_num_samples = 1.f / num_samples;
	point3f pixel, sample;
	pixel.y = y0;
	for (int j = y0, row = y0 * context->extent.width; j <= y1; ++j, pixel.y += 1, row += context->extent.width) {
		pixel.x = x0;
		for (int i = x0, index = row + x0; i <= x1; ++i, pixel.x += 1, ++index) {
			for (int k = 0; k < num_samples; ++k) {
				sample.x = pixel.x + sample_pattern[k].x;
				sample.y = pixel.y + sample_pattern[k].y;
				
				float w0 = edge(p1, p2, &sample);
				float w1 = edge(p2, p0, &sample);
				float w2 = edge(p0, p1, &sample);
				
				if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
					w0 /= area, w1 /= area, w2 /= area;
					float one_over_z = w0 / p0->z + w1 / p1->z + w2 / p2->z;
					float z = 1 / one_over_z;

					if (z < context->depth_buffer[index * num_samples + k]) {
						context->depth_buffer[index * num_samples + k] = z;

						texcoord2f st;
						st.s = st0->s * w0 + st1->s * w1 + st2->s * w2;
						st.t = st0->t * w0 + st1->t * w1 + st2->t * w2;
						st.s *= z;
						st.t *= z;
						shade(mesh->shader, st, &context->color_buffer[index * num_samples + k]);
					}
				}
			}		
		}
	}
}

void render(struct context* context, int num_meshes, const mesh** const meshes) {
	float bbox[4];
	int x0, x1, y0, y1;
	for (int i = 0; i < num_meshes; ++i) {
		const mesh* const mesh = meshes[i];
		const int* vi = mesh->face_vertex_indices;
		const int* sti = mesh->st.indices;
		for (int j = 0; j < mesh->num_triangles; ++j, vi += 3, sti += 3) {
			point3f p0 = mesh->points[vi[0]];
			point3f p1 = mesh->points[vi[1]];
			point3f p2 = mesh->points[vi[2]];
			persp_divide(&p0, context->znear);
			persp_divide(&p1, context->znear);
			persp_divide(&p2, context->znear);
			to_raster(context->screen_coordinates, context->extent, &p0);
			to_raster(context->screen_coordinates, context->extent, &p1);
			to_raster(context->screen_coordinates, context->extent, &p2);
			tri_bbox(&p0, &p1, &p2, bbox);
			if (bbox[0] > context->extent.width - 1 || bbox[2] < 0 || bbox[1] > context->extent.height - 1 || bbox[3] < 0)
				continue;
			x0 = max(0, (int)bbox[0]);
			y0 = max(0, (int)bbox[1]);
			x1 = min(context->extent.width - 1, (int)bbox[2]);
			y1 = min(context->extent.width - 1, (int)bbox[3]);
			
			texcoord2f st0 = mesh->st.coords[sti[0]];
			texcoord2f st1 = mesh->st.coords[sti[1]];
			texcoord2f st2 = mesh->st.coords[sti[2]];
			st0.s /= p0.z, st0.t /= p0.z;
			st1.s /= p1.z, st1.t /= p1.z;
			st2.s /= p2.z, st2.t /= p2.z;

			rasterize(x0, y0, x1, y1, &p0, &p1, &p2, &st0, &st1, &st2, mesh, context);
		}
	}
}

void cleanup(struct context* context) {
	free(context->depth_buffer);
	free(context->color_buffer);
}

static inline float remap(float val, float lo, float hi) { return (val - lo) / (hi - lo); }

int main() {
	context context;
	init(&context);
	prepare_buffers(&context);

	int num_meshes = 2;
	mesh** meshes = (mesh**)malloc(sizeof(mesh*) * num_meshes);
	for (int i = 0; i < num_meshes; ++i) {
		meshes[i] = (mesh*)malloc(sizeof(mesh));
	}
	
	create_mesh(&context, meshes[0]);

	render(&context, 1, (const mesh** const)meshes);
	
	float min = 0.1;
	float max = 10;
	float *fb = (float*)malloc(4 * context.extent.width * context.extent.height * sizeof(float));
	for (int i = 0, pi = 0; i < context.extent.width * context.extent.height * 4; i += 4, pi = i / 4) {		
		float z0 = remap(context.depth_buffer[i    ], min, max);
		float z1 = remap(context.depth_buffer[i + 1], min, max);
		float z2 = remap(context.depth_buffer[i + 2], min, max);
		float z3 = remap(context.depth_buffer[i + 3], min, max);
		int row = pi / context.extent.width;
		int col = pi % context.extent.width;
		fb[(row * 2    ) * context.extent.width * 2 + col * 2    ] = z0;
		fb[(row * 2    ) * context.extent.width * 2 + col * 2 + 1] = z1;
		fb[(row * 2 + 1) * context.extent.width * 2 + col * 2    ] = z2;
		fb[(row * 2 + 1) * context.extent.width * 2 + col * 2 + 1] = z3;
	}
	
	FILE *file = fopen("./depth.ppm", "wb");
	fprintf(file, "P6\n%d %d\n255\n", context.extent.width * 2, context.extent.height * 2);
	for (int i = 0; i < context.extent.width * context.extent.height * 4; ++i) {
		unsigned char rgb[3] = {
			(unsigned char)(fb[i] * 255),
			(unsigned char)(fb[i] * 255),
			(unsigned char)(fb[i] * 255),
		};
		fwrite(rgb, 3, 1, file);
	}
	fclose(file);
	free(fb);

	file = fopen("./color.ppm", "wb");
	fprintf(file, "P6\n%d %d\n255\n", context.extent.width, context.extent.height);
	for (int i = 0; i < context.extent.width * context.extent.height * 4; i += 4) {
		color3f accum = {0,0,0};
		for (int j = 0; j < 4; ++j) {
			accum.x += context.color_buffer[i + j].x;
			accum.y += context.color_buffer[i + j].y;
			accum.z += context.color_buffer[i + j].z;
		}
		unsigned char rgb[3] = {
			(unsigned char)(accum.x / 4.f * 255),
			(unsigned char)(accum.y / 4.f * 255),
			(unsigned char)(accum.z / 4.f * 255),
		};
		fwrite(rgb, 3, 1, file);
	}
	fclose(file);

	// cleanup
	cleanup(&context);
	destroy_mesh(meshes[0]);
	//destroy_mesh(meshes[1]);
	
	free(meshes);

	return 0;
}