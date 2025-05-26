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
#define ANGLE_VEL -.03f
#define TRACER_LENGTH 45
#define FRAMERATE 120
#define MAX_NODES 20



typedef struct{
	bool exists;
	Vec2 solution1;
	Vec2 solution2;
} Intersection;

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
	// if the node is a rotating node, the pos is in the form (r,theta)
	Vec2 pos; 
	Node_state state;
} Node;

typedef struct{
	Node* nodes;
	double adj_matrix[MAX_NODES][MAX_NODES];
	int num_nodes;
} Graph;

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