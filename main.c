#include "main.h"



float clamp(float x, float a, float b){
	if (x > b)return b;
	if (x < a)return a;
	return x;
}

bool hovering_over_line(Vec2 mouse_pos, Graph* graph,Line line){
	// dot projection formula from 
	Vec2 p1 = graph->nodes[line.source_node].pos;
	Vec2 p2 = graph->nodes[line.target_node].pos;
	Vec2 atomouse = Vector2Add(mouse_pos,p1);
	Vec2 ab = Vector2Subtract(p2,p1);
	// distance on line from 0 to 1 that is closest to mouse
	float h = clamp(Vector2DotProduct(atomouse,ab)/Vector2DotProduct(ab,ab),0.0,1.0);
	return Vector2Length(Vector2Subtract(atomouse,Vector2Scale(ab,h)))<10.0f?true:false;
}

bool vec2_equal(Vec2 a, Vec2 b){ 
	// dont need to worry about floating point rounding here
	return (a.x == b.x && a.y == b.y);
}

bool lines_equal(Line a, Line b){
	return  (a.source_node == b.source_node && a.target_node == b.target_node) ||
		    (a.source_node == b.target_node && a.target_node == b.source_node);
}


bool hovering_over_joint(Vec2 mouse_pos,Vec2 joint){
	return Vector2Length(Vector2Subtract(joint,mouse_pos))<JOINT_RADIUS*3.0f?true:false;
}

void draw_view(Graph* graph,State* state,Vec2 mouse_pos){
	for (int i = 0; i <MAX_NODES;i++){
		for (int j = i+1;j<MAX_NODES;j++){
			if (graph->adj_matrix[i][j] == 0.0f){
				continue;
			}
			Colour line_col;
			if (lines_equal((Line){i,j},state->line_hovered)){
				line_col = RED;
			} else {
				line_col = BLACK;
			}
			DrawLineEx(graph->nodes[i].pos,graph->nodes[j].pos,5.0f,line_col);
			
		}
	}

	if (state->selected_joint){
		DrawLineEx(graph->nodes[state->joint_selected].pos,mouse_pos,5.0f,RED);
	}

	for(int i = 0;i<MAX_NODES;i++){
		Colour joint_col = i==state->joint_hovered?RED:BLACK;
		DrawCircleV(graph->nodes[i].pos,10.0f,joint_col);
	}
}

void draw_sim(Graph* graph,Vec2 tracer[]){
	for (int i = 0; i < MAX_NODES;i++){
		for (int j = i+1;j<MAX_NODES;j++){
			if (graph->adj_matrix[i][j] == 0.0){
				continue;
			}
			Vec2 p1 = graph->nodes[i].pos;
			Vec2 p2 = graph->nodes[j].pos;
			DrawLineEx(p1,p2,5.0f,BLACK);
		}
	}

	for(int i = 0;i<TRACER_LENGTH;i++){
		DrawCircleV(tracer[i],5.0f,RED);
	}

	for (int i = 0;i<MAX_NODES;i++){
		DrawCircleV(graph->nodes[i].pos,10.0f,BLACK);
	}
}


/* 
i got this from
https://www.petercollingridge.co.uk/tutorials/computational-geometry/circle-circle-intersections/
*/
void circle_intersection(Vec2 p1, Vec2 p2, Vec2* prev_sol, float r1, float r2){
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;

	float d = sqrt(dx*dx + dy*dy);
	if (d > r1 + r2 || d < abs(r2-r1)){
		return;
	}

	dx /= d;
	dy /= d;

	const float a = (r1*r1 - r2*r2 + d*d)/(2*d);
	const float px = p1.x + a*dx;
	const float py = p1.y + a*dy;
	const float h = sqrt(r1*r1 - a*a);

	Vec2 solution1 = {px+h*dy,
					  py-h*dx};

	Vec2 solution2 = {px-h*dy,
					  py+h*dx};

	Vec2 solution1_diff = Vector2Subtract(*prev_sol,solution1);
	Vec2 solution2_diff = Vector2Subtract(*prev_sol,solution2);

	float manhattan_dist_sol1 = abs(solution1_diff.x+solution1_diff.y);
	float manhattan_dist_sol2 = abs(solution2_diff.x+solution2_diff.y);

	if (manhattan_dist_sol1 > manhattan_dist_sol2){
		*prev_sol = solution2;
	} else {
		*prev_sol = solution1;
	}
}

bool line_exists(Graph* graph,int source, int target){
	return !(graph->adj_matrix[source][target] == 0.0f);
}


void add_node(Graph* graph,Vec2 pos,Node_state state){
	Node new_node;
	new_node.pos = pos;
	new_node.state = state;
	graph->free_node_count--;
	uint8_t node_id = graph->free_nodes[graph->free_node_count];
	graph->nodes[node_id]=new_node;
}

void add_line(Graph* graph,int a,int b){
	double dist = Vector2Distance(graph->nodes[a].pos,graph->nodes[b].pos);
	graph->adj_matrix[a][b] = dist;
	graph->adj_matrix[b][a] = dist;
}

void remove_node(Graph* graph,int node_id){
	for(int j = 0;j<MAX_NODES;j++){
		graph->adj_matrix[j][node_id]=0.0f;
		graph->adj_matrix[node_id][j]=0.0f;
	}
	graph->free_nodes[graph->free_node_count--] = node_id;
}

State* init_state(){
	State* state = malloc(sizeof(State));
	state->hovering_joint=false;
	state->selected_joint=false;
	state->hovering_line=false;
	state->selected_line=false;
	state->joint_hovered=0;
	state->joint_selected=0;
	state->line_hovered=(Line){0,0};
	state->line_selected=(Line){0,0};
	return state;
}


Graph* init_graph(){
	Graph* graph = malloc(sizeof(Graph));
	*graph = (Graph){0};

	add_node(graph,(Vec2){400,400},Fixed); 
	add_node(graph,(Vec2){300,400},Fixed);
	add_node(graph,(Vec2){400,370},Rotating);
	add_line(graph,0,2);
	return graph;
}

bool can_be_determined(Graph* graph,int node_id){
	int constraints=0;
	for (int i = 0;i<MAX_NODES;i++){
		if (graph->adj_matrix[node_id][i] != 0.0f && graph->nodes[i].state != Unknown){
			constraints++;
		}
	}
	return (constraints > 1);
}

Linkage* get_node_evaluation_order(Graph* graph) {

	Linkage* linkage = malloc(sizeof(Linkage));



	int num_nodes = MAX_NODES - graph->free_node_count;
	uint8_t* queue = malloc(num_nodes);
	uint8_t* known_neighbours = calloc(num_nodes,1);
	uint8_t* known_neighbour1 = malloc(num_nodes);
	uint8_t* known_neighbour2 = malloc(num_nodes);

	// start with 2 fixed nodes and 1 rotating node
	int frontptr = 3;
	queue[0] = 0;
	queue[1] = 1;
	queue[2] = 2;

	while (frontptr<=num_nodes){
		bool added_node = false;
		int curr_node = frontptr-1;
		for(int i = 0;i<MAX_NODES;i++){
			if (graph->adj_matrix[curr_node][i] == 0.0f)
				continue;
			if (graph->nodes[i].state == Unknown)
				known_neighbours[i]++;

			if (known_neighbours[i]>1){
				added_node = true;
				queue[frontptr]=i;
				frontptr++;
			}
		}
		if (!added_node && frontptr<num_nodes){
			printf("linkage is underconstrained");
			return NULL;
		}
	}
}

void reset_states(Graph* graph){
	for(int i =0;i<MAX_NODES;i++){
		if (graph->nodes[i].state == Known){
			graph->nodes[i].state=Unknown;
		}
	}
}

void update_node_positions(Linkage* linkage){
	// get new position for the rotating node
	Vec2* rotating_node = linkage->positions+ROTATING_NODE;
	Vec2 pivot_node = linkage->positions[PIVOT_NODE];

	const float cos_theta = cos(ANGLE_VEL);
	const float sin_theta = sin(ANGLE_VEL);

	Vec2 diff = Vector2Subtract(*rotating_node,pivot_node);
	rotating_node->x = pivot_node.x + (diff.x * cos_theta - diff.y * sin_theta);
	rotating_node->y = pivot_node.y + (diff.x * sin_theta + diff.y * cos_theta);


	for(int i = 3;i<linkage->num_nodes;i++){
		uint8_t pnode1 = linkage->pnode1[i];
		uint8_t pnode2 = linkage->pnode2[i];
		uint8_t conn1 = linkage->connection1[i];
		uint8_t conn2 = linkage->connection2[i];

		Vec2 p1 = linkage->positions[pnode1];
		Vec2 p2 = linkage->positions[pnode2];
		float r1 = linkage->connections[conn1];
		float r2 = linkage->connections[conn2];

		circle_intersection(p1,p2,linkage->positions+i,r1,r2);

	}
}


void run_sim(Graph* graph){
	Linkage* linkage = create_linkage(graph);

	Vec2 tracer[TRACER_LENGTH];
	for(int i = 0;i<TRACER_LENGTH-1;i++){
		tracer[i]=graph->nodes[MAX_NODES-1].pos;
	}



	while (!WindowShouldClose()){
		BeginDrawing();

		update_node_positions(linkage);

		for(int i =0;i<TRACER_LENGTH-1;i++){
			tracer[i]=tracer[i+1];
		}
		tracer[TRACER_LENGTH-1] = linkage->nodes[linkage->num_nodes-1].pos;
		reset_states(graph);
		ClearBackground(WHITE);
		draw_sim(graph,tracer);
		EndDrawing();
	}
}

void sanitise_graph(Graph* graph){ // remove unconnected nodes
	int size = MAX_NODES*sizeof(Node);
	Node* connected_nodes = malloc(size);
	int frontptr = 0;
	for(int i = 0;i<MAX_NODES;i++){
		Node curr_node = graph->nodes[i];
		if (curr_node.num_connected > 0){
			connected_nodes[frontptr++]=curr_node;
		}
	}
	memcpy(graph->nodes,connected_nodes,size);
	MAX_NODES = frontptr;
	free(connected_nodes);
}


void setup_mode(Graph* graph){
	if (graph == NULL){
		graph = init_graph();
	}
	Vec2 mouse_pos;
	Image play_button_img = LoadImage("assets/buttons.png");
	Texture2D play_button = LoadTextureFromImage(play_button_img);
	UnloadImage(play_button_img);
	Rect play_button_rect = {SCREEN_WIDTH-50-80,50,80,80};
	State* state = init_state();
	while(!WindowShouldClose()){

		
		BeginDrawing();
		//DrawRectangleRec(run_button_rect,RED);
		mouse_pos = GetMousePosition();
		ClearBackground(WHITE);
		DrawTexturePro(
			play_button,(Rect){20.0f,0.0f,play_button.height,play_button.height},
			play_button_rect,
			(Vec2){0,0},0.0f,WHITE
			);




		draw_view(graph,state,mouse_pos);
		EndDrawing();
		state->hovering_joint = false;
		state->hovering_line = false;
		state->joint_hovered = NULL;
		state->line_hovered = NULL;

		for (int i = 0;i<MAX_NODES;i++){
			if (hovering_over_joint(mouse_pos,graph->nodes[i].pos)){
				state->joint_hovered = graph->nodes+i;
				state->hovering_joint = true;
				break;
			}
		}

		for (int i = 0;i<graph->num_lines;i++){
			if (hovering_over_line(mouse_pos,graph,graph->lines[i])){
				state->line_hovered = graph->lines+i;
				state->hovering_line = true;
				break;
			}
		}

		// helps when joints and lines overlap
		if (state->hovering_joint){
			state->hovering_line=false;
		}


		if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)){
			// a joint is selected

			if (CheckCollisionPointRec(mouse_pos,play_button_rect)){
				return;
			}

			if (state->selected_joint){
				if (!state->hovering_joint){
					add_node(graph,mouse_pos,Unknown);
					add_line(graph,state->joint_selected,graph->nodes+MAX_NODES-1);
					state->selected_joint = false;
				}

				if (state->hovering_joint && state->joint_hovered != state->joint_selected){
					add_line(graph,state->joint_selected,state->joint_hovered);
					state->selected_joint = false;
				}
			// no joint is selected but hovering over a joint
			} else if (state->hovering_joint){
				state->joint_selected = state->joint_hovered;
				state->selected_joint = true;
			} else{
				add_node(graph,mouse_pos,Unknown);
			}
		}

		if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)){

			if (state->hovering_joint){
				if (state->joint_hovered > graph->nodes+1){
					remove_node(graph,state->joint_hovered);
				}
			} else if (state->hovering_line){
				remove_connection(graph,state->line_hovered);
			}
		}
	}
}

void update_graph_copy(Graph* graph,Graph* graph_copy){
	graph_copy->num_nodes=MAX_NODES;
	graph_copy->num_lines=graph->num_lines;
	memcpy(graph_copy->nodes,graph->nodes,MAX_NODES*sizeof(Node));
	memcpy(graph_copy->lines,graph->lines,graph->num_lines*sizeof(Line));
}

void jansen_linkage(){
	return;
}

int main(){
	InitWindow(SCREEN_WIDTH,SCREEN_HEIGHT,"strandbeest");
	bool in_setup = true;
	SetTargetFPS(FRAMERATE);
	Graph* graph = init_graph();
	Graph* graph_copy = init_graph();

	while (!WindowShouldClose()){
		if (in_setup){
			setup_mode(graph);
			in_setup = false;
			update_graph_copy(graph,graph_copy);
			sanitise_graph(graph_copy);

		} else {
			run_sim(graph_copy);
		}
	}
	
	CloseWindow();
	return 1;
}
