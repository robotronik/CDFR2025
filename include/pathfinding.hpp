#pragma once
#include "fonction.h"
#include "constante.h"
#include <queue>
#include <stack>



struct CompareInstruction{
    bool operator()(const instruction_t& a, const instruction_t& b){
        return a.cout > b.cout;
    }
};

typedef std::priority_queue<instruction_t,std::vector<instruction_t>, CompareInstruction> heap;
typedef std::stack<instruction_t> Astack;

instruction_t stack_pop(Astack* stack, instruction_t* parent);


float distance2(position_t* p1, position_t* p2);

int getOrientation(int x, int y);

void RobotAdverse_obstacle(tableState* itable, float pastemps, int numpas );

void RobotActualise(robot_t* robot, int x , int y, int time);

instruction_t Euristique(instruction_t* current, position_t* objectif,robot_t* adversaire, int next_teta, int MouvDist);

int RobotGetZone(position_t pos, int N);

void CalculeNext(heap* Calculer,robot_t* adversaire, position_t* objectif, instruction_t* current);

instruction_t* A_star(tableState* itable, position_t objectif); 