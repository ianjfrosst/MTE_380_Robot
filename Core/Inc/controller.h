

#define NULL_LCOORD 9999 // max Lcoord = 1717
#define NULL_MAP 200 // max map ind = 3 
#define NULL_SENSOR_DIST -999 // max map ind = 3 

#include <stdint.h> 

uint8_t GRID_SQ_MM  = 102 // 304.8/3 = 101.6 

typedef uint16_t LCoord; 

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

uint8_t kGridSize = 18; 
uint8_t kNumMaps = 4; 
LCoord MAP[kNumMaps][kGridSize][kGridSize]; 

uint8_t kNumSteps = 100; 
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
    {{0, 0901}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0801}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0701}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0601}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0501}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0401}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0301}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0201}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0101}, {1, 1601}}, 
    {{1, 1501}, {NULL_MAP, NULL_LCOORD}, 
    {{1, 1401}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1301}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1201}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1101}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0901}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0801}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0701}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0601}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0501}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0401}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0301}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0201}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0101}, {2, 1601}}, 
    {{2, 1501}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1401}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1301}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1201}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1101}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0901}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0801}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0701}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0601}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0501}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0401}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0301}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0201}, {NULL_MAP, NULL_LCOORD}},
    {{2, 0101}, {3, 1601}},
    {{3, 1501}, {NULL_MAP, NULL_LCOORD}, 
    {{3, 1401}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1301}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1201}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1101}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1001}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0901}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0801}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0701}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0601}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0501}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0401}, {0, 1604}},
    {{0, 1504}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1404}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1304}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0904}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0804}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0704}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0604}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0504}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0404}, {1, 1304}},
    {{1, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0904}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0804}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0704}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0604}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0504}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0404}, {2, 1304}},
    {{2, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0904}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0804}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0704}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0604}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0504}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0404}, {3, 1304}},
    {{3, 1204}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1104}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 1004}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0904}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0804}, {NULL_MAP, NULL_LCOORD}}, 
    {{3, 0704}, {0, 1307}}, 
    {{0, 1207}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1107}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 1007}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0907}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0807}, {NULL_MAP, NULL_LCOORD}}, 
    {{0, 0707}, {1, 1007}}, 
    {{1, 0907}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0807}, {NULL_MAP, NULL_LCOORD}}, 
    {{1, 0707}, {2, 1007}}, 
    {{2, 0907}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0807}, {NULL_MAP, NULL_LCOORD}}, 
    {{2, 0707}, {NULL_MAP, NULL_LCOORD}}, 
}