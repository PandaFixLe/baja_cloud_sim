#include "radar_can_parser/radar_parser.h"
#include <cmath>
#include <iostream>

#define PI 3.1415926

extern double CAN_GetFormatData(int startByte, int startBit, int length,
                                double resolution, double offset,
                                const uint8_t* data, int is_moto);

void RadarDataParser::processCanData(const CanData& canData, uint8_t radarID) {
    // Header 帧 -> 重置计数
    if ((canData.ID & 0x0F) == 0x00) {
        if (radarID == RightRearRadar) {
            radarPack.PeakFarCount[radarID] = 0;
        } else if (radarID == RearCenterRadar) {
            radarPack.ObjectNum[radarID] = 0;
        }
        return;
    }

    // 0x601 (RR) -> 点云
    if (radarID == RightRearRadar) {
        parseRRClusterFrame(canData);
        return;
    }

    // 0x641 (RC) -> 目标
    if (radarID == RearCenterRadar) {
        parseRCObjectFrame(canData);
        return;
    }
}

void RadarDataParser::parseRRClusterFrame(const CanData& canData) {
    const int rr_cluster_abs_bits[8][6] = {
        {7, 11, 24, 46, 55, 48},
        {76, 81, 88, 110, 119, 112},
        {132, 137, 144, 166, 175, 168},
        {196, 201, 208, 230, 239, 232},
        {260, 265, 272, 294, 303, 296},
        {324, 329, 336, 358, 367, 360},
        {372, 377, 384, 406, 415, 408},
        {436, 441, 448, 470, 479, 472}
    };

    for (int i = 0; i < 8; ++i) {
        int idx = radarPack.PeakFarCount[RightRearRadar];
        if (idx >= MAX_PEAK_FRAME) return;

        RadarProcessed& pt = radarPack.PeakFar[RightRearRadar][idx];
        const int* abs = rr_cluster_abs_bits[i];

        auto get_val = [&](int abs_bit, int len, double factor, double offset) {
            int byte = abs_bit / 8;
            int bit = abs_bit % 8;
            return CAN_GetFormatData(byte, bit, len, factor, offset, canData.Data, 0);
        };

        pt.Range   = get_val(abs[0], 12, 0.0625, 0);
        pt.Speed   = get_val(abs[1], 11, 0.1, -120);
        pt.RCS     = get_val(abs[2], 9, 0.2, -51.2);
        pt.Angle   = get_val(abs[3], 10, 0.004, -1.6);
        pt.EzAngle = get_val(abs[4], 7, 0.005, -0.315);
        pt.SNR     = get_val(abs[5], 7, 0.5, 0);

        // 后向雷达角度补偿
        float final_angle = pt.Angle + PI;
        pt.Dx = pt.Range * sin(final_angle);
        pt.Dy = pt.Range * cos(final_angle);
        pt.Dh = pt.Range * sin(pt.EzAngle);

        radarPack.PeakFarCount[RightRearRadar]++;
    }
}

void RadarDataParser::parseRCObjectFrame(const CanData& canData) {
    const int obj00_abs[9] = {
        7, 11, 39, 43, 98, 152, 175, 472, 456
    };
    const int obj01_abs[9] = {
        223, 227, 255, 259, 314, 368, 391, 488, 472
    };

    auto parse_single_obj = [&](const int* abs) {
        int idx = radarPack.ObjectNum[RearCenterRadar];
        if (idx >= MAX_OBJECT_NUM) return;

        RadarObject& obj = radarPack.ObjectList[RearCenterRadar][idx];
        auto get_val = [&](int abs_bit, int len, double factor, double offset) {
            int byte = abs_bit / 8;
            int bit = abs_bit % 8;
            return CAN_GetFormatData(byte, bit, len, factor, offset, canData.Data, 0);
        };

        obj.DistLong = get_val(abs[0], 12, 0.10752688, -179.6);
        obj.VrelLong = get_val(abs[1], 11, 0.125, -128);
        obj.DistLat  = get_val(abs[2], 12, 0.0875, -179.1125);
        obj.VrelLat  = get_val(abs[3], 11, 0.125, -128);
        obj.Class    = get_val(abs[4], 3, 1, 0);
        obj.RCS      = get_val(abs[5], 9, 0.2, -51.2);
        obj.Orientation = get_val(abs[6], 10, 0.01, -5.11);
        obj.DistHigh = get_val(abs[7], 8, 0.1, -12.5);
        obj.ID       = (uint8_t)get_val(abs[8], 8, 1, 0);

        radarPack.ObjectNum[RearCenterRadar]++;
    };

    parse_single_obj(obj00_abs);
    parse_single_obj(obj01_abs);
}