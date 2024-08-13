
#include <cstdlib>
#include <string.h>

struct point3f { float x, y, z; };
struct normal3f { float x, y, z; };
struct texCoord2f { float s, t; };

struct tri_mesh {
	point3f* points;
	//int* faceVertexCounts;
	int* faceVertexIndices;
	struct st {
		texCoord2f* coords;
		int* indices;
	};
	int num_triangles;
	int num_points;
	normal3f* normals;
};

void create_mesh(struct tri_mesh* mesh) {
#include "sphere.h"
	int num_faces = sizeof(faceVertexCounts) / sizeof(faceVertexCounts[0]);
	mesh->num_points = sizeof(points) / sizeof(points[0]);
	mesh->points = (point3f*)malloc(mesh->num_points * sizeof(point3f));
	memcpy((char*)mesh->points, points, mesh->num_points * sizeof(point3f));

	int num_triangles = 0;
	for (int i = 0; i < num_faces; ++i) {
		num_triangles = faceVertexCounts[i] - 2;
	}

	mesh->faceVertexIndices = (int*)malloc(num_triangles * 3 * sizeof(int));
	int vi[3], index_offset = 0, ntris = 0;
	for (int i = 0; i < num_faces; ++i) {
		vi[0] = faceVertexIndices[index_offset++];
		vi[1] = faceVertexIndices[index_offset++];
		for (int j = 0; j < faceVertexCounts[i] - 2; ++j) {
			vi[2] = faceVertexIndices[index_offset++];
			memcpy((char*)mesh->faceVertexIndices + ntris * 3, vi, sizeof(int) * 3);
			vi[1] = vi[2];
			++ntris;
		}
	}
}

void destroy_mesh(struct tri_mesh* mesh) {
	free(mesh->points);
	free(mesh->faceVertexIndices);
	free(mesh);
}

int main() {
	int num_meshes = 2;
	tri_mesh** meshes = (tri_mesh**)malloc(sizeof(tri_mesh*) * num_meshes);
	for (int i = 0; i < num_meshes; ++i) {
		meshes[i] = (tri_mesh*)malloc(sizeof(tri_mesh));
	}

	create_mesh(meshes[0]);
	
	
	// cleanup
	destroy_mesh(meshes[0]);
	//destroy_mesh(meshes[1]);
	
	free(meshes);

	return 0;
}