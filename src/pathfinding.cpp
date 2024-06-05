#include "pathfinding.hpp"

int getOrientation(int x, int y){
    return (int)((x/ abs(x) == 1 ? 0 : 1*M_PI) -  atan(x / y)/DEG_TO_RAD);
}

float distance2(position_t* p1, position_t* p2){
    return sqrt(pow(p1->x-p2->x,2)+pow(p1->y-p2->y,2));
}

instruction_t stack_pop(Astack* stack, instruction_t* parent){
    while(!stack->empty() && (&(stack->top()) != parent)){
        stack->pop();
    }
    if(stack->empty()){
        perror("Stack vide\n");
    }
    else{
        instruction_t previous = stack->top();
        stack->pop();
        return previous;
    }
}

void build_path(Astack* stack,Astack* solution, instruction_t* last ){
    while(last->parent!=NULL){
        solution->push(stack_pop(stack,last));
        last = &(solution->top());
    }

}

void RobotAdverse_obstacle(tableState* itable, float pastemps, int numpas ){
    robot_t* adversaire = &(itable->adversaire);
    obstacle_t *obstacle = &(adversaire->obstacle);

    obstacle->forme = RECTANGLE;
    obstacle->demie_largeur = LARGEUR_ROBOT*SECURITE/2;
    obstacle->demie_longueur = sqrt(pow(adversaire->vit_x,2) + pow(adversaire->vit_y,2))*pastemps* SECURITE /2;
    obstacle->teta = (adversaire->pos.teta);
    obstacle->centre_x = adversaire->pos.x + adversaire->vit_x*pastemps*SECURITE/2;
    obstacle->centre_y = adversaire->pos.y + adversaire->vit_y*pastemps*SECURITE/2;
    return;
}

void RobotActualise(robot_t* robot, int x, int y , int time){
    robot->vit_x = (x - robot->pos.x)/(time-robot->pos.time);
    robot->vit_y = (y - robot->pos.y)/(time-robot->pos.time);
    robot->pos.teta = getOrientation(robot->vit_x,robot->vit_y);
    robot->pos.x= x;
    robot->pos.y= y;
    robot->pos.time= time;
    return;
}

int RobotGetZone(position_t pos, int N){
    int zone = pos.x * 2 + N*2* pos.y;
    return zone;
}

instruction_t Euristique(instruction_t* current, position_t* objectif,robot_t* adversaire ,int next_teta, int MouvDist){
    instruction_t next;
    next.parent = current;
    next.pos.teta = next_teta;
    next.pos.x = (int)(MouvDist*cos(next.pos.teta));
    next.pos.y = (int)(MouvDist*sin(next.pos.teta));

    int tetaMouv = next_teta- current->pos.teta;

    if(!tetaMouv){
        next.type = DROIT;
    }else{
        next.type = ROTATION;
    }

    if(current->type == ROTATION && next.type == DROIT)
        next.cout = 10*MouvDist + distance2(&(next.pos),objectif) + RISQUE / distance2(&(next.pos),&(adversaire->pos));
    else if (current->type == DROIT && next.type == DROIT)
    {
        next.cout = 1*MouvDist + distance2(&(next.pos),objectif) + RISQUE / distance2(&(next.pos),&(adversaire->pos));
    }else{
        next.cout = 100*tetaMouv + distance2(&(next.pos),objectif) + RISQUE / distance2(&(next.pos),&(adversaire->pos));
    }
    
    return next;
}


instruction_t* A_star(tableState* itable, position_t objectif){
    heap Calculer;
    Astack traiter;
    instruction_t current;
    current.parent = NULL;
    current.pos = itable->robot.pos;
    current.cout = 0;

    while(current.pos.teta != objectif.teta && current.pos.x != objectif.x && current.pos.y != objectif.y){
        CalculeNext(&Calculer,&(itable->adversaire),&objectif, &current);
        current = Calculer.top();
    }
    Astack solution;
    build_path(&traiter,&solution,&current);
}


void CalculeNext(heap* Calculer, robot_t* adversaire, position_t* objectif, instruction_t* current){
    int tetaIdeal = getOrientation(objectif->x-current->pos.x,objectif->y-current->pos.y);
    int MouvDist,next_teta;

    MouvDist = 0;
    for (int i = 0; i < NOMBRE_TEST_Angle; i++)
    {
        next_teta = tetaIdeal + i* ( 360/(NOMBRE_TEST_Angle));
        Calculer->push(Euristique(current,objectif,adversaire, next_teta,MouvDist));
    }

    next_teta = 0;
    for (int i = 1; i <= NOMBRE_TEST_DROIT; i++)
    {
        MouvDist = (int)(distance2(objectif,&(current->pos))>200 ? 200 / i : distance2(objectif,&(current->pos)) )/i ;
        Calculer->push(Euristique(current,objectif, adversaire,next_teta, MouvDist));
        Calculer->push(Euristique(current,objectif,adversaire,next_teta,-MouvDist));
    }
}