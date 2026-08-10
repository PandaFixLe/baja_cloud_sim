#ifndef RADAR_PARSER_H
#define RADAR_PARSER_H

#include "radar_struct.h"

class RadarDataParser {
public:
    PackageData radarPack;
    void processCanData(const CanData& canData, uint8_t radarID);

private:
    // 用于 0x600/0x601 (右后 RR) 的解析函数
    void parseRRClusterFrame(const CanData& canData);
    
    // 用于 0x640/0x641 (后中 RC) 的解析函数
    void parseRCObjectFrame(const CanData& canData);
    
    // 保留旧函数用于兼容（虽然当前不用，但声明了免得报错）
    void parseHeaderFrame(const CanData& canData, uint8_t radarID);
};

#endif