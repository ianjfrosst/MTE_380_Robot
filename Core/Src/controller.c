#include "controller.h"
#include "stdbool.h"
#include "math.h"

SenseDist_t* LCoordToSenseDist(LCoord coord){ 
    SenseDist_t dist; 
    // it may overshoot, but since actively sensing/correcting, should be able to stop overshooting 
    dist.F_dist = coord/100*GRID_SQ_MM; 
    dist.L_dist = (coord%100)*GRID_SQ_MM;
    return &dist; 
}

LCoord SenseDistToLCoord(SenseDist_t* dist){
    LCoord loc; 
    loc = dist->F_dist/GRID_SQ_MM*100; 
    loc += dist->L_dist/GRID_SQ_MM; 
    return loc;
}

LCoord SenseDistToLCoord(uint64_t front, uint64_t left){
    LCoord loc; 
    loc = front/GRID_SQ_MM*100; 
    loc += left/GRID_SQ_MM; 
    return loc;
}

bool PosesEqual(Pose_t* pose1, Pose_t* pose2){
    return (pose1->map_ind == pose2->map_ind && pose1->coord == pose2->coord);
}

bool SenseDistsEqual(SenseDist_t* sd1, SenseDist_t* sd2){
    return (sd1->L_dist == sd2->L_dist && sd1->F_dist == sd2->F_dist);
}

PoseError_t* PoseError(Pose_t* ref_pose, Pose_t* curr_pose, bool ignore_map){
    if(ref_pose->map_ind != curr_pose->map_ind && !ignore_map){
        return POSE_MAP_ERROR; 
    }
    // Error: will assume it is is center of square. Okay? --> rounds to closest square 

    PoseError_t error; 
    int16_t coord_error = ref_pose->coord - curr_pose->coord; 

    error.F_err = ref_pose->coord/100 - curr_pose->coord/100; 
    error.L_err = ref_pose->coord%100 - curr_pose->corrd%100; 

    return &coord_error; 
}

void SetupMaps(){
    for(uint8_t i = 0; i < kGridSize; i++){ // y, traverses rows 
       for(uint8_t j = 0; j < kGridSize; j++){ // x, traverses col
           MAP[1][i][j] = i*100 + j; 
           MAP[2][i][j] = (kGridSize - 1 - i)*100 + j;
           MAP[3][i][j] = (kGridSize - 1 - i)*100 + (kGridSize - 1 - j);
           MAP[0][i][j] = i*100 + (kGridSize - 1 - j);
        } 
    }
}

void PrintMaps(){
    for (uint8_t k = 0; k < kNumMaps; k++) {	
        for (uint8_t i = 0; i < kGridSize; i++) { // y, traverses rows 
	        for (uint8_t j = 0; j < kGridSize; j++){// x, traverses col
	            printf ("%04d\t", MAP[k][i][j]);
	        }
	        printf ("\n");
	    }
        printf ("\n\n\n");
    }
}

void ControlLoop(){
    uint8_t next_pose_ind = 0; 
    uint8_t curr_map = 0; 
    LCoord curr_loc;
    Pose_t curr_pose;
    PlannedPose_t next_pose; 

    IMUAngle IMU_ref = ReadIMU(); //***to get current offset being started with  
    //bool not_turned = false; 
    Angle angle_dev = 0; 
    PoseError_t error; 
    Angle turn_angle = 0;  

    next_pose = TRAVEL_PATH[next_pose_ind]; 
    curr_loc = SenseDistToLCoord(ReadToF(front), ReadToF(left)); //***STAND IN FOR TOF FUNCTIONS ;
    curr_pose = {curr_map, curr_loc};

    if(!PosesEqual(&next_pose.pose, &curr_pose)){
        //***FLASH LIGHTS?? - STARTING IN WRONG SPOT 
        return;
    }else{
        next_pose_ind+=1; 
    }

    while(next_pose_ind < kNumSteps){
        next_pose = TRAVEL_PATH[next_pose_ind];
        curr_loc = SenseDistToLCoord(ReadToF(front), ReadToF(left)); //***STAND IN FOR TOF FUNCTIONS ;
        curr_pose = {curr_map, curr_loc};

        //***** IF PREV POSE = CURR POSE FOR MORE THAN X READS --> WERE STUCK! MORE POWER! MOVE! 

        if(PosesEqual(&next_pose.after_turn, &NULL_POSE)){

            error = *(PoseError(&next_pose.pose, &curr_pose, false)); 

            if(error.F_err == POSE_MAP_ERROR.F_err && error.L_err == POSE_MAP_ERROR.L_err)){
                ; // Do something? When would this happen? 
            }

            if(PosesEqual(&next_pose.pose, &curr_pose)){
                next_pose_ind += 1; 
            }else{
                
                //***** ADD CHECK THAT WE ARE >1 SQ FROM WALL

                if(error.L_err != 0){
                    turn_angle = arctan(1/error.L_err); 
                    turn_angle = turn_angle < 0 ? -90 - turn_angle : 90 - turn_angle; 
                    TURN(turn_angle); // ***assuming angle > 0 = Turn CW, <0 = CCW 
                    //***Move forward incr for a set time approx = 0.5 or 1 sq 
                }

                angle_dev = (IMU_ref - ReadIMU()) % (360*ANGLE_TO_IMU_ANGLE); //***call IMU reading 
                if(abs(angle_dev) > 1){ 
                    TURN(-1*angle_dev); // ***
                    curr_loc = SenseDistToLCoord(ReadToF(front), ReadToF(left)); //***STAND IN FOR TOF FUNCTIONS ;
                    curr_pose = {curr_map, curr_loc};
                    //***** ADD CHECK THAT WE ARE >1 SQ FROM WALL
                }
                // it will move forward as required, and angle dev would show the 
                // turned value on the next accurate reading and turn it back to straight 
                // BUT angled during correction = ToF sensors are WRONG 


                if(error.F_err > 0){
                   ; //***MOTORS MOVE FORWARD  
                }else{ // overshot (ie. err <0) or correct row, L<>R off which is why PoseEqual failed
                    next_pose_ind+=1; 
                } 

            }
        }else{ // need to turn. Either at or not at turn row yet 
            
            angle_dev = (IMU_ref - ReadIMU()) % (360*ANGLE_TO_IMU_ANGLE); //***call IMU reading 
            
            // Correct any prior offset ex. if correction from previous step and update curr_loc
            if(abs(angle_dev) > 1){ 
                TURN(-1*angle_dev); // ***
                curr_loc = SenseDistToLCoord(ReadToF(front), ReadToF(left)); //***STAND IN FOR TOF FUNCTIONS ;
                curr_pose = {curr_map, curr_loc};
            }
            error = *(PoseError(&next_pose.pose, &curr_pose, false));

            if(error.F_err > 1){
                ;// ***Motor forward
            }else{  // either overshot (error.F_err < 0) or in perfect place 

                // OPTION 1: Turn 90, let above code correct from there 
                TURN(90); ///*** CW Turns only 
                angle_dev = (IMU_ref - ReadIMU()) % (360*ANGLE_TO_IMU_ANGLE); //***call IMU reading  
                
                while(angle_dev < 85){
                    TURN(10);
                    angle_dev = (IMU_ref - ReadIMU()) % (360*ANGLE_TO_IMU_ANGLE); //***call IMU reading  
                }

                curr_map = next_pose.after_turn.map_ind ; 
                IMU_ref += 90*ANGLE_TO_IMU_ANGLE; //RESET IMU -- SHOULD NOT TRY TO UNTURN 90 
                next_pose_ind += 1;

                // OPTION 2: Turn until next map with next pose more similar than current                
                #if 0
                    turn_curr_pose = {next_pose.after_turn.map_ind, curr_loc};
                    turn_error = PoseError(next_pose.after_turn, turn_curr_pose);

                    not_turned = turn_error > curr_error; 
                    if(!turned){ 
                        // turn in small incr on the spot
                    }else{
                        curr_map = next_pose.after_turn.map_ind; 
                        // turn_cycle = false; 
                        next_pose_ind += 1;
                    }
                    // while(!not_turned){
                    //     // turn in small increments in spot 
                    //     curr_error = PoseError(next_pose.pose, curr_pose, false);
                    //     turn_error = PoseError(next_pose.after_turn, curr_pose, true);
                    //     not_turned = turn_error < curr_error; 
                    // }
                #endif 
                
            }
        }
    }

/*
Need to check for PoseError returning SENSE_ERROR 
L-R error: Need to move by -1*error.L_dist ex. 1600 -> 1601. Error = -1, need to move +1
F error is -ve: Need to turn 90 and move back to position :/ -- AVOIDDDDD 
Control loop should only correct L-R Error. Turns should be preemptive? (undershoot better than overshoot) 
WE TURN IN SPOT. Maybe turn not in spot but slightly forward, then change when we turn to undershoot by one mini-square? 

Movement: 
- needs a default forward movement (speed)
- need a default correction factor for movement ex. if need to go towards left, multiply error by factor (Power scale). -100 to +100 = -90 to +90deg 
- could it also call loc's movement func? 
*/

int main(){


    LCoord front; 
    LCoord left; 

    while(true){
        front = ;  // call to ToF front;
        left = ;   // call to ToF left; 
        printf("Loc: %d \n", SenseDistToLCoord(front, left)); 
    }

    // SetupMaps();
    // PrintMaps(); 

    // Sense function setup 
    // Move function setup 
    // Sense - if in wrong starting position - exit / flash a light 

    // ControlLoop(); 

    return 0; 
}