#ifndef RADAR_STRUCT_H
#define RADAR_STRUCT_H

#include <cstdint>
#include <array>
#include <vector>

#define MAX_PEAK_FRAME 512
#define MAX_OBJECT_NUM 100
#define RADAR_COUNT 6

typedef enum {
    LeftRearRadar = 0,
    LeftFrontRadar = 1,
    RightFrontRadar = 2,
    FrontCenterRadar = 3,
    RightRearRadar = 4,   // 0x601 (点云)
    RearCenterRadar = 5   // 0x641 (目标)
} SensorID;

struct CanData {
    uint32_t ID;
    uint8_t DataLenth;
    uint8_t Channel;
    double timeStamp;
    uint8_t Data[64];
};

// 点云簇结构 (用于 0x601 解析)
struct RadarProcessed {
    float Range, Angle, Speed, EzAngle, RCS, SNR;
    float Dx, Dy, Dh;
};

// 目标结构 (用于 0x641 解析)
struct RadarObject {
    float DistLong;    // 纵向距离 x
    float DistLat;     // 横向距离 y
    float VrelLong;    // 纵向相对速度
    float VrelLat;     // 横向相对速度
    float Class;       // 目标分类
    float RCS;
    float Orientation;
    float DistHigh;
    uint8_t ID;
};

struct PackageData {
    // 存放 0x601 解析出的点云
    std::array<RadarProcessed, MAX_PEAK_FRAME> PeakFar[RADAR_COUNT];
    uint16_t PeakFarCount[RADAR_COUNT] = {0};
    
    // 存放 0x641 解析出的目标
    std::array<RadarObject, MAX_OBJECT_NUM> ObjectList[RADAR_COUNT];
    uint16_t ObjectNum[RADAR_COUNT] = {0};

    uint8_t UpdateChannel[RADAR_COUNT] = {0};
};

#endif