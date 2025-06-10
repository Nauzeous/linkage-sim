#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


typedef Vector2 Vec2;
typedef Rectangle Rect;
typedef Color Colour;

#define JOINT_RADIUS 10.0f
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 800
#define TRACER_LENGTH 45
#define FRAMERATE 120.0f
#define MAX_NODES 20
#define ANGLE_VEL 0.5f/(2.0f*PI*FRAMERATE)
#define PIVOT_NODE 0 // the index in arrays, immutable
#define FIXED_NODE 1
#define ROTATING_NODE 2



typedef struct{
	uint8_t source_node;
	uint8_t target_node;
} Line;

typedef enum{
	Known,
	Unknown,
	Fixed,
	Rotating
}Node_state;

typedef struct{
	Vec2 pos; 
	Node_state state;
} Node;

typedef struct{
	Node nodes[MAX_NODES];
	float adj_matrix[MAX_NODES][MAX_NODES];
	uint8_t free_nodes[MAX_NODES]; // use this like a stack
	uint8_t free_node_count; 
} Graph;

typedef struct{
	int num_nodes;
	Vec2* positions;
	float* connections;
	uint8_t* pnode1;
	uint8_t* connection1;
	uint8_t* pnode2;
	uint8_t* connection2;
} Linkage;



typedef struct{
	// these names might be a bit confusing

	Line line_hovered;
	Line line_selected;

	uint8_t joint_hovered;
	uint8_t joint_selected;

	bool hovering_joint;
	bool selected_joint;

	bool hovering_line;
	bool selected_line;
} State;