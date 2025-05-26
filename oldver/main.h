#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


typedef Vector2 Vec2;
typedef Rectangle Rect;
typedef Color Colour;

#define JOINT_RADIUS 10.0f
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 800
#define MAX_NODES 100
#define MAX_LINES 150
#define MAX_CONNECTIONS 10 // max number of connections to a single node
#define ANGLE_VEL -.03f
#define TRACER_LENGTH 45
#define FRAMERATE 120





typedef struct{
	bool exists;
	Vec2 solution1;
	Vec2 solution2;
} Intersection;

typedef struct Line Line;
typedef struct Node Node;

typedef enum{
	Known,
	Unknown,
	Fixed,
	Rotating
}Node_state;

struct Node{
	// if the node is a rotating node, the pos is in the form (r,theta)
	Vec2 pos; 
	Line** connected; // array of pointers to lines
	int num_connected; 
	Node_state state;
};



struct Line{
	float length;
	Node* source;
	Node* target;
};

typedef struct{
	Node* nodes; // array of nodes
	int num_nodes;
	Line* lines; // array of lines
	int num_lines;
} Graph;

typedef struct{
	// these names might be a bit confusing
	bool hovering_joint;
	bool selected_joint;
	bool hovering_line;
	bool selected_line;

	Node* joint_hovered;
	Node* joint_selected;
	Line* line_hovered;
	Line* line_selected;
} State;