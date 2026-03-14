// FloodFill.h - Maze solver using FloodFill algorithm
#ifndef FLOODFILL_H
#define FLOODFILL_H

#include "GlobalConfig.h"
#include "Sensors.h"
#include "Motors.h"
#include "RobotMotion.h"

// --- Data structures ---
typedef struct coord { int x; int y; } coord;
typedef struct neighbor { coord pos; Heading heading; int streak; } neighbor;
typedef neighbor item_type;

struct node { item_type data; struct node* next; };
struct _queue { struct node* head; struct node* tail; int size; };
typedef struct _queue* queue;

// --- Maze state ---
unsigned char target = STARTING_TARGET;
coord currentXY = {STARTING_X, STARTING_Y};
Heading currentHeading = STARTING_HEADING;

int verticalWalls[MAZE_WIDTH+1][MAZE_HEIGHT]   = {{0}};
int horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT+1] = {{0}};
int floodArray[MAZE_WIDTH][MAZE_HEIGHT];

// --- Declarations ---
void halt_with_error(const char* msg);
queue queue_create();
int queue_is_empty(queue q);
void queue_push(queue q, item_type elem);
item_type queue_pop(queue q);
void queue_clear(queue q);

int  API_wallFront();
int  API_wallLeft();
int  API_wallRight();
void API_moveForward();
void API_turnRight();
void API_turnLeft();
void API_turn180();

void updateFloodArray(coord c, int val);
int isAccessible(coord c1, coord c2);
void generateNeighbor(queue q, neighbor n, Heading h, int streak);
void floodFill();
void updateWalls();
Action turnLeft();
Action turnRight();
Action nextAction();
void checkDestination();
void generateInitialWalls();
Action solver();
void printMaze();

void debugPrintFF(Action act);
void debugPrintEncoder();
void sendLine(uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3);

// --- API wrappers ---
int API_wallFront(){ return wallFront(); }
int API_wallRight(){ return wallRight(); }
int API_wallLeft(){  return wallLeft(); }

void API_moveForward(){ BT.println("[FF] FORWARD"); forward_one_cell(); }
void API_turnRight(){  BT.println("[FF] RIGHT");   turn_90_right(); }
void API_turnLeft(){   BT.println("[FF] LEFT");    turn_90_left(); }
void API_turn180(){    BT.println("[FF] 180");     turn_180_exact(); }

// --- Debug / telemetry ---
void sendLine(uint16_t d0,uint16_t d1,uint16_t d2,uint16_t d3){
  char buf[64];
  int n=snprintf(buf,sizeof(buf),
      "SL:%u SR:%u FL:%u FR:%u", d0,d1,d2,d3);
  BT.write(buf,n);
  BT.write('\n');
}

void debugPrintFF(Action act) {
    switch(act){
        case FORWARD: BT.println("[FF] DECISION = FORWARD"); break;
        case LEFT:    BT.println("[FF] DECISION = LEFT"); break;
        case RIGHT:   BT.println("[FF] DECISION = RIGHT"); break;
        case IDLE:    BT.println("[FF] DECISION = IDLE (180 or stuck)"); break;
    }
}

void debugPrintEncoder() {
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        "[ENC] L:%ld  R:%ld",
        countLeft, countRight
    );
    BT.write(buf, n);
    BT.write('\n');
}

// --- Queue ---
void halt_with_error(const char* msg) { 
  BT.print("ERROR: "); 
  BT.println(msg); 
  motorStop(); 
  while(1); 
}

queue queue_create() {
    queue q = (queue) malloc(sizeof(struct _queue));
    if (q == NULL) { halt_with_error("Out of memory"); }
    q->head = NULL; q->tail = NULL; q->size = 0; return q;
}

int queue_is_empty(queue q) { 
    if (q == NULL) { halt_with_error("NULL queue"); } 
    return q->head == NULL; 
}

void queue_push(queue q, item_type elem) {
    struct node* n = (struct node*) malloc(sizeof(struct node));
    if (n == NULL) { halt_with_error("Out of memory"); }
    n->data = elem; n->next = NULL;
    if (q->head == NULL) { q->head = q->tail = n; } 
    else { q->tail->next = n; q->tail = n; }
    q->size += 1;
}

item_type queue_pop(queue q) {
    if (queue_is_empty(q)) { halt_with_error("Queue empty"); }
    struct node* head = q->head;
    if (q->head == q->tail) { q->head = NULL; q->tail = NULL; } 
    else { q->head = q->head->next; }
    q->size -= 1; item_type data = head->data; free(head); return data;
}

void queue_clear(queue q) { while (!queue_is_empty(q)) { queue_pop(q); } }

// --- FloodFill core ---
void updateFloodArray(coord c, int val) { floodArray[c.x][c.y] = val; }

// Check if two adjacent cells have no wall between them
int isAccessible(coord c1, coord c2) {
    if (c1.x == c2.x) {
        if (c1.y > c2.y) return !horizontalWalls[c1.x][c1.y];
        else             return !horizontalWalls[c1.x][c2.y];
    }
    if (c1.y == c2.y) {
        if (c1.x > c2.x) return !verticalWalls[c1.x][c1.y];
        else             return !verticalWalls[c2.x][c1.y];
    }
    return 0;
}

void generateNeighbor(queue q, neighbor n, Heading h, int streak) {
    coord newCoord = n.pos;
    switch(h) {
        case NORTH: newCoord.y++; break;
        case WEST:  newCoord.x--; break;
        case SOUTH: newCoord.y--; break;
        case EAST:  newCoord.x++; break;
    }
    if (newCoord.x < 0 || newCoord.x >= MAZE_WIDTH || newCoord.y < 0 || newCoord.y >= MAZE_HEIGHT) return;
    if (!isAccessible(n.pos, newCoord)) return;

    int floodVal    = floodArray[n.pos.x][n.pos.y];
    int newFloodVal = floodArray[newCoord.x][newCoord.y];
    int score       = TILE_SCORE 
                      + ((n.heading == h) ? 0 : TURN_SCORE) 
                      - ((n.heading == h) ? STREAK_SCORE : 0);

    if (newFloodVal == NOT_YET_SET || newFloodVal > floodVal + score) {
        updateFloodArray(newCoord, floodVal + score);
        neighbor nxt = { newCoord, h, (n.heading == h) ? streak + 1 : 1 };
        queue_push(q, nxt);
    }
}

void floodFill() {
    for (int i=0; i < MAZE_WIDTH; i++)
      for (int j=0; j < MAZE_HEIGHT; j++)
        floodArray[i][j] = NOT_YET_SET;

    queue q = queue_create();

    if (target) {
        // Flood from goal region
        for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++)
          for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++) {
            coord g = {x, y};
            updateFloodArray(g, 0); 
            neighbor n = { g, NORTH, 0 };
            queue_push(q, n);
          }
    } else {
        // Flood from start (returning home)
        coord s = {STARTING_X, STARTING_Y};
        updateFloodArray(s, 0);
        neighbor n = { s, NORTH, 0 };
        queue_push(q, n);
    }

    while (!queue_is_empty(q)) {
        neighbor current = queue_pop(q);
        generateNeighbor(q, current, NORTH, current.streak);
        generateNeighbor(q, current, WEST,  current.streak);
        generateNeighbor(q, current, SOUTH, current.streak);
        generateNeighbor(q, current, EAST,  current.streak);
    }
    queue_clear(q); free(q);
}

// Read current walls from sensors and store in wall arrays
void updateWalls() {
    if (API_wallFront()) {
        switch (currentHeading) {
            case NORTH: horizontalWalls[currentXY.x][currentXY.y+1] = 1; break;
            case WEST:  verticalWalls[currentXY.x][currentXY.y]     = 1; break;
            case SOUTH: horizontalWalls[currentXY.x][currentXY.y]   = 1; break;
            case EAST:  verticalWalls[currentXY.x+1][currentXY.y]   = 1; break;
        }
    }
    if (API_wallLeft()) {
        switch (currentHeading) {
            case NORTH: verticalWalls[currentXY.x][currentXY.y]     = 1; break;
            case WEST:  horizontalWalls[currentXY.x][currentXY.y]   = 1; break;
            case SOUTH: verticalWalls[currentXY.x+1][currentXY.y]   = 1; break;
            case EAST:  horizontalWalls[currentXY.x][currentXY.y+1] = 1; break;
        }
    }
    if (API_wallRight()) {
        switch (currentHeading) {
            case NORTH: verticalWalls[currentXY.x+1][currentXY.y]   = 1; break;
            case WEST:  horizontalWalls[currentXY.x][currentXY.y+1] = 1; break;
            case SOUTH: verticalWalls[currentXY.x][currentXY.y]     = 1; break;
            case EAST:  horizontalWalls[currentXY.x][currentXY.y]   = 1; break;
        }
    }
}

Action turnLeft() { 
    API_turnLeft(); 
    currentHeading = (Heading)((currentHeading + 1) % 4); 
    return LEFT; 
}

Action turnRight() { 
    API_turnRight(); 
    currentHeading = (Heading)((currentHeading == NORTH) ? EAST : currentHeading - 1); 
    return RIGHT; 
}

// Decide next move based on flood values of neighbors
Action nextAction() {
    int currentFlood = floodArray[currentXY.x][currentXY.y];

    int northFlood = (currentXY.y + 1 < MAZE_HEIGHT) ? floodArray[currentXY.x][currentXY.y+1] : OUT_OF_BOUNDS;
    int westFlood  = (currentXY.x - 1 >= 0)          ? floodArray[currentXY.x-1][currentXY.y] : OUT_OF_BOUNDS;
    int southFlood = (currentXY.y - 1 >= 0)          ? floodArray[currentXY.x][currentXY.y-1] : OUT_OF_BOUNDS;
    int eastFlood  = (currentXY.x + 1 < MAZE_WIDTH)  ? floodArray[currentXY.x+1][currentXY.y] : OUT_OF_BOUNDS;

    int     minFlood   = currentFlood;
    Heading newHeading = currentHeading;

    if (northFlood != OUT_OF_BOUNDS && isAccessible(currentXY, (coord){currentXY.x,   currentXY.y+1}) && northFlood < minFlood) { 
        minFlood = northFlood; newHeading = NORTH; 
    }
    if (westFlood  != OUT_OF_BOUNDS && isAccessible(currentXY, (coord){currentXY.x-1, currentXY.y})   && westFlood  < minFlood) { 
        minFlood = westFlood;  newHeading = WEST;  
    }
    if (southFlood != OUT_OF_BOUNDS && isAccessible(currentXY, (coord){currentXY.x,   currentXY.y-1}) && southFlood < minFlood) { 
        minFlood = southFlood; newHeading = SOUTH; 
    }
    if (eastFlood  != OUT_OF_BOUNDS && isAccessible(currentXY, (coord){currentXY.x+1, currentXY.y})   && eastFlood  < minFlood) { 
        minFlood = eastFlood;  newHeading = EAST;  
    }

    if (newHeading == currentHeading) {
        API_moveForward();
        switch (currentHeading) {
            case NORTH: currentXY.y++; break;
            case WEST:  currentXY.x--; break;
            case SOUTH: currentXY.y--; break;
            case EAST:  currentXY.x++; break;
        }
        return FORWARD;
    }

    if (currentHeading == (newHeading+3)%4) return turnLeft();
    else if (currentHeading == (newHeading+1)%4) return turnRight();
    else { 
        API_turn180(); 
        currentHeading = (Heading)((currentHeading + 2) % 4); 
        return IDLE; 
    }
}

void checkDestination() {
    if (target) {
        if (currentXY.x >= LOWER_X_GOAL && currentXY.x <= UPPER_X_GOAL &&
            currentXY.y >= LOWER_Y_GOAL && currentXY.y <= UPPER_Y_GOAL) {
            target = 0;
            BT.println("[FF] Reached GOAL -> heading back to START");
        }
    } else {
        if (currentXY.x == STARTING_X && currentXY.y == STARTING_Y) {
            target = 1;
            BT.println("[FF] Back at START. Stopping.");
            motorStop();
            while(1);
        }
    }
}

// Set outer boundary walls
void generateInitialWalls() {
    for (int i=0; i < MAZE_WIDTH; i++) {
        horizontalWalls[i][0]            = 1;
        horizontalWalls[i][MAZE_HEIGHT]  = 1;
    }
    for (int i=0; i < MAZE_HEIGHT; i++) {
        verticalWalls[0][i]             = 1;
        verticalWalls[MAZE_WIDTH][i]    = 1;
    }
}

// Main solver loop: check destination -> read walls -> flood -> move
Action solver() {
  checkDestination();
  updateWalls();
  floodFill();
  return nextAction();
}

// Print maze to Bluetooth (ASCII art)
void printMaze() {
    BT.print("Pos: (");
    BT.print(currentXY.x);
    BT.print(", ");
    BT.print(currentXY.y);
    BT.println(")");

    for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            BT.print("+");
            if (horizontalWalls[x][y + 1]) BT.print("---");
            else                           BT.print("   ");
        }
        BT.println("+");

        for (int x = 0; x < MAZE_WIDTH; x++) {
            if (verticalWalls[x][y]) BT.print("|");
            else                     BT.print(" ");

            if (currentXY.x == x && currentXY.y == y) {
                switch (currentHeading) {
                    case NORTH: BT.print(" ^ "); break;
                    case WEST:  BT.print(" < "); break;
                    case SOUTH: BT.print(" v "); break;
                    case EAST:  BT.print(" > "); break;
                }
            } else {
                BT.print("   ");
            }
        }
        BT.println("|");
    }

    for (int x = 0; x < MAZE_WIDTH; x++) {
        BT.print("+---");
    }
    BT.println("+");
    BT.println("==============================================");
}

#endif // FLOODFILL_H
