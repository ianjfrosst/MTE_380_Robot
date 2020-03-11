

#define NULL_LCOORD 9999 // max Lcoord = 1717
#define NULL_MAP 200 // max map ind = 3 
#define NULL_SENSOR_DIST -999 // max map ind = 3 
#define NULL_POSE_ERR -99

#include <stdint.h> 

uint8_t GRID_SQ_MM  = 102; // 304.8/3 = 101.6 
uint16_t ANGLE_TO_IMU_ANGLE = ; // Factor to convert real angle to IMU angle 

typedef uint16_t LCoord; 
typedef uint16_t Angle;
typedef uint16_t IMUAngle;

typedef struct {
    int16_t L_dist; // left sensor
    int16_t F_dist; // front sensor
}SenseDist_t; 

SenseDist_t SENSE_ERROR = {NULL_SENSOR_DIST, NULL_SENSOR_DIST};

typedef struct{
    uint8_t map_ind; 
    LCoord coord; 
}Pose_t;

typedef struct{
    Pose_t pose; 
    Pose_t after_turn; 
}PlannedPose_t; 

typedef struct{
    int8_t F_err; 
    int8_t L_err; 
}PoseError_t; 

PoseError_t POSE_MAP_ERROR = {NULL_POSE_ERR, NULL_POSE_ERR}; 

const uint8_t kGridSize = 18; 
const uint8_t kNumMaps = 4; 
LCoord MAP[kNumMaps][kGridSize][kGridSize]; 

const uint8_t kNumSteps = 100; 
PlannedPose_t TRAVEL_PATH[kNumSteps] = {
// {
//     {
//         .pose = {
//             .map_ind = 0,
//             .coord = 123123
//         },
//         .after_turn = {
//             .map_ind = 1,
//             .coord = 112324
//          }
//     },
// }
    {{0, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 901}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 801}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 701}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 601}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 501}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 401}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 301}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 201}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 101}, {1, 1601}}, 
    {{1, 1501}, {NULL_MAP, NULL_LCOORD}, 
    {{1, 1401}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1301}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1201}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1101}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 901}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 801}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 701}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 601}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 501}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 401}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 301}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 201}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 101}, {2, 1601}}, 
    {{2, 1501}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1401}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1301}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1201}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1101}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 901}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 801}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 701}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 601}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 501}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 401}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 301}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 201}, {NULL_MAP, NULL_LCOORD}},
    {{2, 101}, {3, 1601}},
    {{3, 1501}, {NULL_MAP, NULL_LCOORD}, 
    {{3, 1401}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1301}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1201}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1101}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 901}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 801}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 701}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 601}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 501}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 401}, {0, 1604}},
    {{0, 1504}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1404}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1304}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 904}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 804}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 704}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 604}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 504}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 404}, {1, 1304}},
    {{1, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 904}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 804}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 704}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 604}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 504}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 404}, {2, 1304}},
    {{2, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 904}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 804}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 704}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 604}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 504}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 404}, {3, 1304}},
    {{3, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 904}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 804}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 704}, {0, 1307}}, 
    {{0, 1207}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1107}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1007}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 907}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 807}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 707}, {1, 1007}}, 
    {{1, 907}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 807}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 707}, {2, 1007}}, 
    {{2, 907}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 807}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 707}, {NULL_MAP, NULL_LCOORD}}, 
}

SenseDist_t LCoordToSenseDist(LCoord coord);
LCoord SenseDistToLCoord(SenseDist_t* dist);
LCoord SenseDistToLCoord(uint64_t front, uint64_t left);
SenseDist_t* LCoordToSenseDist(LCoord coord); 

bool PosesEqual(Pose_t* pose1, Pose_t* pose2); 
bool SenseDistsEqual(SenseDist_t* sd1, SenseDist_t* sd2); 
PoseError_t* PoseError(Pose_t* ref_pose, Pose_t* curr_pose, bool ignore_map);

void SetupMaps(); 
void PrintMaps(); 

void ControlLoop(); 