#include <cstdint>

// is_moto: 0 = Intel (小端), 1 = Motorola (大端)
double CAN_GetFormatData(int startByte, int startBit, int length,
                         double resolution, double offset,
                         const uint8_t* data, int is_moto) {
    if (length <= 0 || startByte < 0) return 0.0;

    uint64_t canData = 0;
    int bitShiftRight = startBit % 8;
    int bitLeft = 8 - bitShiftRight;
    int currentByte = startByte;

    if (is_moto) { // Motorola (大端) - 本DBC未使用
        while (length > bitLeft) {
            canData |= (uint64_t)(data[currentByte] >> bitShiftRight) << (length - bitLeft);
            length -= bitLeft;
            bitLeft = 8;
            bitShiftRight = 0;
            currentByte--;
        }
        canData |= (uint64_t)((data[currentByte] >> bitShiftRight) & ((1 << length) - 1));
    } else { // Intel (小端) - DBC全部是@0+，使用此分支
        while (length > bitLeft) {
            canData |= (uint64_t)(data[currentByte] >> bitShiftRight) << (length - bitLeft);
            length -= bitLeft;
            bitLeft = 8;
            bitShiftRight = 0;
            currentByte++;
        }
        canData |= (uint64_t)((data[currentByte] >> bitShiftRight) & ((1 << length) - 1));
    }

    // 【核心修复】DBC定义的是分辨率（乘法因子），所以用乘法
    return (double)canData * resolution + offset;
}