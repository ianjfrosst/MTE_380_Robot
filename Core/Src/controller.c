#include "controller.h"
#include "stdbool.h"


// Assumed FUnctions:
// Sense(curr_map_ind) -> LCoord 
// Move(dist_x, dist_y)  
// s_c(map, dist_x, dist_y) // sensor → coord
// c_s(x,y) = coord // sensor (will know to check which is > 1000 to apply proper inverse)
// Error: will assume it is is center of square. Okay? 

SenseDist_t LCoordToSenseDist(LCoord coord){ 
    SenseDist_t dist; 
    // it may overshoot, but since actively sensing/correcting, should be able to stop overshooting 
    dist.F_dist = coord/100*GRID_SQ_MM; 
    dist.L_dist = (coord%100)*GRID_SQ_MM;
    return dist; 
}

LCoord SenseDistToLCoord(SenseDist_t front, SenseDist_t left){
    LCoord loc; 
    loc = front/GRID_SQ_MM*100; 
    loc += left/GRID_SQ_MM; 
    return loc;
}

bool PosesEqual(Pose_t pose1, Pose_t pose2){
    return (pose1.map_ind == pose2.map_ind && pose1.coord == pose2.coord);
}

bool SenseDistsEqual(SenseDist_t sd1, SenseDist_t sd2){
    return (sd1.L_dist == sd2.L_dist && sd1.F_dist == sd2.F_dist);
}

LCoord PoseError(Pose_t ref_pose, Pose_t curr_pose, bool ignore_map){
    if(ref_pose.map_ind != curr_pose.map_ind && !ignore_map){
        return SENSE_ERROR; 
    }
    LCoord coord_error = ref_pose - curr_pose; 
    return coord_error; 
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
    uint8_t next_pose_ind = 1; 
    uint8_t curr_map = 0; 
    LCoord curr_loc;
    Pose_t curr_pose;
    PlannedPose_t next_pose; 

    // bool turn_cycle = false;
    LCoord curr_error; 
    LCoord turn_error; 
    bool not_turned = false; 

    while(next_pose_ind < kNumSteps){
        // if(!turn_cycle)
        next_pose = TRAVEL_PATH[next_pose_ind];
        curr_loc = Sense(curr_map); 
        curr_pose = {curr_map, curr_loc};

        if(PosesEqual(next_pose.after_turn, NULL_POSE){
            if(PosesEqual(next_pose, curr_pose)){
                next_pose_ind += 1; 
            }else{
                LCoord error = PoseError(next_pose.pose, curr_pose, false); 
                SenseDist_t move = LCoordToSenseDist(error); 
                move.L_dist*=(-1); 
                // forward_p = 
                // MotorL = (error/)
                // move robot based on that error. 
                // call to loc's Move function
            }
        }else{ // need to turn. Either at or not at turn row yet 
            // turn_cycle = true; 
            int16_t row_diff = (next_pose.pose.coord - curr_pose.pose.coord); 
            if(row_diff > 100){
                // Motor forward
                // call to loc's Move function
            }else{  // either overshot (row_diff < 0) or in perfect place                 
                curr_error = PoseError(next_pose.pose, curr_pose, false);

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

    SetupMaps();

    LCoord front; 
    LCoord left; 

    while(true){
        front = ;  // call to ToF front;
        left = ;   // call to ToF left; 
        printf("Loc: %d \n", SenseDistToLCoord(front, left)); 
    }

    // PrintMaps(); 

    // Sense function setup 
    // Move function setup 
    // Sense - if in wrong starting position - exit / flash a light 

    // ControlLoop(); 

    return 0; 
}