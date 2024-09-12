#include "deplacement.h"

<<<<<<< HEAD
int deplacementPathFinding(int collide, Asser* robot, int x, int y){
    LOG_SCOPE("PATH FINDING");
    int ireturn = 0;
    static bool initStat = true;
    static go_to_State_t currentState = GOTO_INIT;
    go_to_State_t nextState = currentState;
    int deplacementreturn;

    // Ludovic Bouchard - Variables non d�f�nies, n�c�ssaire pour compiler...
    asser_direction_side direction;
    int teta, rotation;

    switch (currentState)
    {
    case GOTO_INIT :
        if(initStat) LOG_STATE("GOTO_INIT");
        nextState = GOTO_LOOKAT;
        break;
    case GOTO_LOOKAT :
        if(initStat){ 
            LOG_STATE("GOTO_LOOKAT");
            if(direction == MOVE_FORWARD){
                robot->setLookForward(x,y,ROTATION_DIRECT);
            } 
            else{
                robot->setLookBackward(x,y,ROTATION_DIRECT);
            }
        }
        if(robot->getError(ANGULAR_ERROR)==0){
            nextState = GOTO_MOVE;
        }
        break;
    case GOTO_MOVE :
        if(initStat) LOG_STATE("GOTO_MOVE");
        deplacementreturn = deplacementLinearPoint(collide,robot,x,y);
        if(deplacementreturn>0){
            nextState = GOTO_TURN;
        }
        else if(deplacementreturn<0){
            nextState = GOTO_INIT;
            ireturn = deplacementreturn;
        }
        break;
    case GOTO_TURN :
        if(initStat){ LOG_STATE("GOTO_TURN");
            robot->angularSetpoint(teta,rotation);
        }
        if(!robot->getError(ANGULAR_ERROR)){
            nextState = GOTO_INIT;
            ireturn = 1;
        }
        break;
    
    default:
        if(initStat) LOG_ERROR("default");
        nextState = GOTO_INIT;
        break;
    }

    initStat = false;
    if(nextState != currentState){
        initStat = true;
    }
    currentState = nextState;
    return ireturn;

}

=======
>>>>>>> 5cab2c8 (save)

int deplacementLinearPoint(int collide, Asser* robot, int x, int y){
    LOG_SCOPE("MOVE");
    static unsigned long startTime;
    static int memx  = 0;
    static int memy = 0;
    static bool initStat = true; 
    static deplcement_State_t step = DEPLACEMENT_INIT;
    deplcement_State_t nextstep = step;
    int iret = 0;
    int distance;


    switch (step)
    {
    case DEPLACEMENT_INIT:
        if(initStat) LOG_STATE("DEPLACEMENT_INIT");
        memx = x;
        memy = y;

        nextstep = DEPLACEMENT_WAITFIRSTMOVE;
        robot->linearSetpoint(memx,memy);

        break;
    case DEPLACEMENT_WAITFIRSTMOVE:
        if(initStat) LOG_STATE("DEPLACEMENT_WAITFIRSTMOVE");
        robot->getBrakingDistance(distance);
        if(distance != 0){
            if(collide < DISTANCESTOP){
                LOG_INFO("distance colide : ",collide);
                nextstep = DEPLACEMENT_WAIT;
                startTime = millis() + 3000; //TIME waiting
            }
            else{
                nextstep = DEPLACEMENT_MOVE;
                robot->linearSetpoint(memx,memy);
            }
        }
        break;

    case DEPLACEMENT_MOVE:
        if(initStat) LOG_STATE("DEPLACEMENT_MOVE");
        if(!robot->getError(LINEAR_ERROR)){
            nextstep = DEPLACEMENT_INIT;
            iret = 1; //GOOD END
        }
        if(collide < DISTANCESTOP){
            LOG_INFO("distance colide : ",collide);
            nextstep = DEPLACEMENT_STOP;
            robot->stop();
            robot->brakeMotor(true);
        }
        break;

    case DEPLACEMENT_STOP:
        if(initStat) LOG_STATE("DEPLACEMENT_STOP");
        robot->getBrakingDistance(distance);
        if(distance==0){
            startTime = millis() + 3000;
            nextstep = DEPLACEMENT_WAIT;
        }
        break;

    case DEPLACEMENT_WAIT:
        if(initStat) LOG_STATE("DEPLACEMENT_WAIT");
        if(startTime < millis()){
            nextstep = DEPLACEMENT_INIT;
            iret = -1; //BAD END
            robot->brakeMotor(false);
            robot->enableMotor(true);
            robot->linearSetpoint(memx,memy);
        }
        if(collide > DISTANCERESTART){
            LOG_INFO("distance colide : ",collide);
            nextstep = DEPLACEMENT_MOVE;
            robot->brakeMotor(false);
            robot->enableMotor(true);
            robot->linearSetpoint(memx,memy);
        }
        break;
    
    default:
        LOG_ERROR("DEFAULT");
        nextstep = DEPLACEMENT_INIT;
        break;
    }

    initStat = false;
    if(nextstep != step){
        initStat = true;
    }
    step = nextstep;
    return iret;
}



int deplacementgoToPoint(int collide, Asser* robot, int x, int y, int teta, asser_direction_side direction,asser_rotation_side rotationLookAt,asser_rotation_side rotation){
    LOG_SCOPE("go to");
    int ireturn = 0;
    static bool initStat = true;
    static go_to_State_t currentState = GOTO_INIT;
    go_to_State_t nextState = currentState;
    int deplacementreturn;


    switch (currentState)
    {
    case GOTO_INIT :
        if(initStat) LOG_STATE("GOTO_INIT");
        nextState = GOTO_LOOKAT;
        break;
    case GOTO_LOOKAT :
        if(initStat){ 
            LOG_STATE("GOTO_LOOKAT");
            if(direction == MOVE_FORWARD){
                robot->setLookForward(x,y,ROTATION_DIRECT);
            } 
            else{
                robot->setLookBackward(x,y,ROTATION_DIRECT);
            }
        }
        if(robot->getError(ANGULAR_ERROR)==0){
            nextState = GOTO_MOVE;
        }
        break;
    case GOTO_MOVE :
        if(initStat) LOG_STATE("GOTO_MOVE");
        deplacementreturn = deplacementLinearPoint(collide,robot,x,y);
        if(deplacementreturn>0){
            nextState = GOTO_TURN;
        }
        else if(deplacementreturn<0){
            nextState = GOTO_INIT;
            ireturn = deplacementreturn;
        }
        break;
    case GOTO_TURN :
        if(initStat){ LOG_STATE("GOTO_TURN");
            robot->angularSetpoint(teta,rotation);
        }
        if(!robot->getError(ANGULAR_ERROR)){
            nextState = GOTO_INIT;
            ireturn = 1;
        }
        break;
    
    default:
        if(initStat) LOG_ERROR("default");
        nextState = GOTO_INIT;
        break;
    }

    initStat = false;
    if(nextState != currentState){
        initStat = true;
    }
    currentState = nextState;
    return ireturn;

}

int deplacementgoToPointNoTurn(int collide, Asser* robot, int x, int y, asser_direction_side direction,asser_rotation_side rotationLookAt){
    LOG_SCOPE("go to");
    int ireturn = 0;
    static bool initStat = true;
    static go_to_State_t currentState = GOTO_INIT;
    go_to_State_t nextState = currentState;
    int deplacementreturn;


    switch (currentState)
    {
    case GOTO_INIT :
        if(initStat) LOG_STATE("GOTO_INIT");
        nextState = GOTO_LOOKAT;
        break;
    case GOTO_LOOKAT :
        if(initStat){ 
            LOG_STATE("GOTO_LOOKAT");
            if(direction == MOVE_FORWARD){
                robot->setLookForward(x,y,ROTATION_DIRECT);
            } 
            else{
                robot->setLookBackward(x,y,ROTATION_DIRECT);
            }
        }
        if(robot->getError(ANGULAR_ERROR)==0){
            nextState = GOTO_MOVE;
        }
        break;
    case GOTO_MOVE :
        if(initStat) LOG_STATE("GOTO_MOVE");
        deplacementreturn = deplacementLinearPoint(collide,robot,x,y);
        if(deplacementreturn>0){
            nextState = GOTO_INIT;
            ireturn = 1;
        }
        else if(deplacementreturn<0){
            nextState = GOTO_INIT;
            ireturn = -1;
        }
        break;
    default:
        if(initStat) LOG_ERROR("default");
        nextState = GOTO_INIT;
        break;
    }

    initStat = false;
    if(nextState != currentState){
        initStat = true;
    }
    currentState = nextState;
    return ireturn;
}

/*
int DeplacementPathFinding(int collide, Asser* robot, int x, int y, int teta, asser_direction_side direction,asser_rotation_side rotationLookAt,asser_rotation_side rotation){ LOG_SCOPE("PATHFINDING");
    int ireturn = 0;
    static bool initStat = true;
    static go_to_State_PF_t currentState = GOTO_INIT_PF;
    go_to_State_PF_t nextState = currentState;
    int deplacementreturn;


    switch (currentState)
    {
    case GOTO_INIT_PF :
        if(initStat) LOG_STATE("GOTO_INIT_PF");
        //Astack solution;
        //solution = 
        nextState = GOTO_CHOOSE_POINT_PF;
        break;

    case GOTO_CHOOSE_POINT_PF :
    if(initStat) LOG_STATE("GOTO_CHOOSE_POINT_PF");
        
        if (solution.empty()){
            nextState = GOTO_TURN_PF;
        }
        x = solution.top().pos.x;
        y = solution.top().pos.y;
        solution.pop;
        nextState = GOTO_LOOKAT_PF;
        
        break;

    case GOTO_LOOKAT_PF :
        if(initStat){ 
            LOG_STATE("GOTO_LOOKAT_PF");
            if(direction == MOVE_FORWARD){
                robot->setLookForward(x,y,ROTATION_DIRECT);
            } 
            else{
                robot->setLookBackward(x,y,ROTATION_DIRECT);
            }
        }
        if(robot->getError(ANGULAR_ERROR)==0){
            nextState = GOTO_MOVE_PF;
        }
        break;
    case GOTO_MOVE_PF :
        if(initStat) LOG_STATE("GOTO_MOVE_PF");
        deplacementreturn = deplacementLinearPoint(collide,robot,x,y);
        if (deplacementImpossible()){
            nextState = GOTO_INIT_PF;
        }
        if(deplacementreturn>0){
            nextState = GOTO_CHOOSE_POINT_PF;
        }
        else if(deplacementreturn<0){
            nextState = GOTO_INIT_PF;
            ireturn = deplacementreturn;
        }
        break;
    case GOTO_TURN_PF :
        if(initStat){ LOG_STATE("GOTO_TURN_PF");
            robot->angularSetpoint(teta,rotation);
        }
        if(!robot->getError(ANGULAR_ERROR)){
            nextState = GOTO_INIT_PF;
            ireturn = 1;
        }
        break;
    
    default:
        if(initStat) LOG_ERROR("default");
        nextState = GOTO_INIT_PF;
        break;
    }

    initStat = false;
    if(nextState != currentState){
        initStat = true;
    }
    currentState = nextState;
    return ireturn;
}
*/