#include "incl2.h"



float clamp(float x, float a, float b){
	if (x > b)return b;
	if (x < a)return a;
	return x;
}




Intersection circle_solutions(Vector2 p1, Vector2 p2, float r1, float r2) {
    double n = p1.x - p2.x;
    double m = p2.y - p1.y;
    double u = r1*r1 + p2.x*p2.x + p2.y*p2.y - p1.x*p1.x - p1.y*p1.y - r2*r2;
    Intersection result = {0};  // Defaults to no solution
    
    // Check if circles are identical or one is inside the other
    double distance = sqrt(n*n + m*m);
    if (distance > r1 + r2) {
        // Circles are too far apart
        result.exists = false;
        return result;
    }
    if (distance < fabs(r1 - r2)) {
        // One circle is inside the other
        result.exists = false;
        return result;
    }
    if (distance == 0 && r1 == r2) {
        // Identical circles - infinite solutions
        result.exists = false;  // No specific solution points
        return result;
    }
    
    // Handle horizontal alignment
    if (fabs(m) < 0.0001f) {
        // Centers horizontally aligned
        double h = (r1*r1 - r2*r2 + distance*distance) / (2*distance);
        double x_mid = p1.x - n * h / distance;
        double offset = sqrt(r1*r1 - h*h);
        
        result.exists = true;
        result.solution1 = (Vector2){ x_mid, p1.y + offset };
        result.solution2 = (Vector2){ x_mid, p1.y - offset };
        return result;
    }
    
    // Handle vertical alignment
    if (fabs(n) < 0.0001f) {
        // Centers vertically aligned
        double h = (r1*r1 - r2*r2 + distance*distance) / (2*distance);
        double y_mid = p1.y - m * h / distance;
        double offset = sqrt(r1*r1 - h*h);
        
        result.exists = true;
        result.solution1 = (Vector2){ p1.x + offset, y_mid };
        result.solution2 = (Vector2){ p1.x - offset, y_mid };
        return result;
    }
    
    // General case - solve quadratic equation
   	double a = m*m + n*n;
    double b = -2.0f * p1.x * m*m + 2.0f * n * (u/2.0f - p1.y * m);
    double c = m*m * p1.x*p1.x + (u/2.0f - p1.y * m) * (u/2.0f - p1.y * m) - m*m * r1*r1;
    
    double discriminant = b*b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        // No intersection (this shouldn't happen due to earlier checks)
        result.exists = false;
        return result;
    }
    
    float sqrtD = sqrtf(discriminant);
    float x1 = (-b + sqrtD) / (2.0f * a);
    float x2 = (-b - sqrtD) / (2.0f * a);
    float y1 = (n * x1 + u / 2.0f) / m;
    float y2 = (n * x2 + u / 2.0f) / m;
    
    result.exists = true;
    result.solution1 = (Vector2){ x1, y1 };
    result.solution2 = (Vector2){ x2, y2 };
    return result;
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
	for (int i = 0; i < graph->num_nodes;i++){
		for (int j = i+1;j<graph->num_nodes;j++){
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

	for(int i = 0;i<graph->num_nodes;i++){
		Colour joint_col = i==state->joint_hovered?RED:BLACK;
		DrawCircleV(graph->nodes[i].pos,10.0f,joint_col);
	}
}

void draw_sim(Graph* graph,Vec2 tracer[]){
	for (int i = 0; i < graph->num_nodes;i++){
		for (int j = i+1;j<graph->num_nodes;j++){
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

	for (int i = 0;i<graph->num_nodes;i++){
		DrawCircleV(graph->nodes[i].pos,10.0f,BLACK);
	}
}


void get_position(int node_id, Graph* graph) {
	Node* node = graph->nodes+node_id;
    switch (node->state) {
        case Fixed:
            return;
        case Rotating:
            Vec2 relative_pos = Vector2Subtract(node->pos, graph->nodes[0].pos);
            relative_pos = Vector2Rotate(relative_pos, ANGLE_VEL);
            Vec2 new_pos = Vector2Add(relative_pos, graph->nodes[0].pos);
            node->pos = new_pos;
            return;
        case Known:
        case Unknown:
            break; // Continue with calculation
    }

    Node pnode1,pnode2;
    double r1,r2;

    bool pnode1_found=false;
    
    // Find 2 connections to known nodes
    for(int i = 0; i < graph->num_nodes; i++) {
    	double curr_conn =
    	if (graph->adj_matrix[node_id][i] != 0.0){
    		r1==0.0?r1:r2 = curr_conn;
    	}
    
    if (!conn1 || !conn2 || !pnode1 || !pnode2) {
        return; // Not enough constraints to determine position
    }
    
    // Now use the correct centers and lengths
    float r1 = conn1->length;
    float r2 = conn2->length;
    
    // Pass the known positions as centers
    Intersection sol = circle_solutions(pnode1->pos, pnode2->pos, r1, r2);
    
    if (!sol.exists) {
        printf("no solution, machine would break here \n");
        return;
    }
    
    // If node already has a position, choose the closest solution
    double dist_from_sol1 = Vector2DistanceSqr(sol.solution1, node->pos);
    double dist_from_sol2 = Vector2DistanceSqr(sol.solution2, node->pos);
    
    node->pos = (dist_from_sol1<dist_from_sol2-1.0)?sol.solution1:sol.solution2;
    node->state = Known;
}

bool line_exists(Graph* graph,Line l){
	for(int i = 0;i<graph->num_lines;i++){
		if ((graph->lines[i].source == l.target && graph->lines[i].target == l.source) &&
		   (graph->lines[i].source == l.source && graph->lines[i].target == l.target))return true;
	}
	return false;
}


void add_node(Graph* graph,Vec2 pos,Node_state state){
	Node new_node;
	new_node.pos = pos;
	new_node.state = state;
	graph->adj_matrix[graph->num_nodes++]
}

void add_line(Graph* graph,int a,int b){
	graph->adj_matrix[a][b] = Vector2Distance(graph->nodes[a],graph->nodes[b]);
}

void remove_connection(Graph* graph,Line* line){	



}
void remove_node(Graph* graph,Node* node){
	for(int i = 0;i<graph->num_lines;i++){
		if (graph->lines[i].source == node || graph->lines[i].target == node){
			remove_connection(graph,graph->lines+i);
			i--;
		}
	}
	graph->num_nodes--;
	*node=graph->nodes[graph->num_nodes];
}

State* init_state(){
	State* state = malloc(sizeof(State));
	state->hovering_joint=false;
	state->selected_joint=false;
	state->hovering_line=false;
	state->selected_line=false;
	state->joint_hovered=NULL;
	state->joint_selected=NULL;
	state->line_hovered=NULL;
	state->line_selected=NULL;
	return state;
}


Graph* init_graph(){
	Graph* graph = malloc(sizeof(Graph));

	graph->lines = malloc(sizeof(Line)*MAX_LINES);
	graph->nodes = malloc(sizeof(Node)*MAX_NODES);
	graph->num_nodes = 0;
	graph->num_lines = 0;
	add_node(graph,(Vec2){400,400},Fixed); 
	add_node(graph,(Vec2){300,400},Fixed);
	add_node(graph,(Vec2){400,370},Rotating);
	add_line(graph,graph->nodes,graph->nodes+2);
	return graph;
}

bool can_be_determined(Graph* graph,Node* node){
	Node* connected_node;
	int constraints=0;
	for (int i = 0;i<node->num_connected;i++){
		Line* curr_line = node->connected[i];
		if (curr_line->source == node && curr_line->target->state != Unknown)constraints++;
		else if (curr_line->target== node && curr_line->source->state!=Unknown)constraints++;

	}
	return (constraints > 1);
}

Node** get_node_evaluation_order(Graph* graph) {
	Node* a=graph->nodes;

    Node** queue = malloc(sizeof(Node*) * graph->num_nodes);

    for(int i=0;i<8;i++){
    	queue[i]=a+i;
    }
    return queue;


    int queue_front = 0;
    int queue_back = 0;
    
    // Add all fixed and rotating nodes to the queue first
    for (int i = 0; i < graph->num_nodes; i++) {
        if (graph->nodes[i].state == Fixed || graph->nodes[i].state == Rotating){ 
            queue[queue_back++] = graph->nodes+i;
		} else {
            graph->nodes[i].state = Unknown;
        }
    }
    
    while (queue_front < queue_back){
        Node* current = queue[queue_front++];
        for (int i = 0; i < current->num_connected; i++) {
            Line* line = current->connected[i];
            Node* other_node;
            if (line->source==current){
            	other_node = line->target;
            } else if (line->target==current){
            	other_node = line->source;
            } else {
            	printf("ermm \n"); // this should not be reachable
            	// if the line is connected to the node then either the target or the source is the node
            }
            if (other_node->state == Unknown && can_be_determined(graph,other_node)){
            	queue[queue_back++]=other_node;
            	other_node->state=Known;
            }
        }
    }
    
    if (queue_back != graph->num_nodes){
    	return NULL;
    }
    
    // If we couldn't determine all nodes, it might be an over-constrained or under-constrained system
    
    return queue;
}

void reset_states(Graph* graph){
	for(int i =0;i<graph->num_nodes;i++){
		if (graph->nodes[i].state == Known){
			graph->nodes[i].state=Unknown;
		}
	}
}


void run_sim(Graph* graph){
	Node** order = get_node_evaluation_order(graph);

	Vec2 tracer[TRACER_LENGTH];
	for(int i = 0;i<TRACER_LENGTH-1;i++){
		tracer[i]=graph->nodes[graph->num_nodes-1].pos;
	}

	while (!WindowShouldClose()){
		BeginDrawing();
		for(int i=0;i<graph->num_nodes;i++){
			get_position(order[i],graph);
		}
		for(int i =0;i<TRACER_LENGTH-1;i++){
			tracer[i]=tracer[i+1];
		}
		tracer[TRACER_LENGTH-1] = graph->nodes[graph->num_nodes-1].pos;
		reset_states(graph);
		ClearBackground(WHITE);
		draw_sim(graph,tracer);
		EndDrawing();
	}
}

void sanitise_graph(Graph* graph){ // remove unconnected nodes
	int size = graph->num_nodes*sizeof(Node);
	Node* connected_nodes = malloc(size);
	int frontptr = 0;
	for(int i = 0;i<graph->num_nodes;i++){
		Node curr_node = graph->nodes[i];
		if (curr_node.num_connected > 0){
			connected_nodes[frontptr++]=curr_node;
		}
	}
	memcpy(graph->nodes,connected_nodes,size);
	graph->num_nodes = frontptr;
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

		for (int i = 0;i<graph->num_nodes;i++){
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
					add_line(graph,state->joint_selected,graph->nodes+graph->num_nodes-1);
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
	graph_copy->num_nodes=graph->num_nodes;
	graph_copy->num_lines=graph->num_lines;
	memcpy(graph_copy->nodes,graph->nodes,graph->num_nodes*sizeof(Node));
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
