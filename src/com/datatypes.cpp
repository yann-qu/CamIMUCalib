#include "com/datatypes.h"
#include <string.h>

McuData generatePanMcuData(float yaw, float pitch) {
    McuData result;
    PanData data = (PanData){'s', MCU_PAN_TYPE, yaw, pitch, 'e'};
    memcpy(&result, &data, sizeof(McuData));
    return result;
}
McuData generateConfigMcuData(uint8_t state, uint8_t anti_top,
                              uint8_t enemy_color) {
    McuData result;
    ConfigData data;
    data.start_flag = 's';
    data.type = MCU_CONFIG_TYPE;
    data.state = state;
    data.anti_top = anti_top;
    data.enemy_color = enemy_color;
    data.end_flag = 'e';
    memcpy(&result, &data, sizeof(McuData));
    return result;
}
McuData generateEnergyMcuData(int delta_x, int delta_y) {
    McuData result;
    EnergyData data = (EnergyData){'s', MCU_ENERGY_TYPE, delta_x, delta_y, 'e'};
    memcpy(&result, &data, sizeof(McuData));
    return result;
}
McuData generateSpeedMcuData(float speed) {
    McuData result;
    SpeedData data;
    data.start_flag = 's';
    data.type = MCU_SPEED_TYPE;
    data.speed = speed;
    data.end_flag = 'e';
    memcpy(&result, &data, sizeof(McuData));
    return result;
}
void readPanMcuData(McuData* mcudata, float* yaw, float* pitch) {
    PanData data;
    memcpy(&data, mcudata, sizeof(McuData));
    *yaw = data.yaw;
    *pitch = data.pitch;
}
void readConfigMcuData(McuData* mcudata, uint8_t* state, uint8_t* anti_top,
                       uint8_t* enemy_color) {
    ConfigData data;
    memcpy(&data, mcudata, sizeof(McuData));
    *state = data.state;
    *anti_top = data.anti_top;
    *enemy_color = data.enemy_color;
}
void readEnergyMcuData(McuData* mcudata, int* delta_x, int* delta_y) {
    EnergyData data;
    memcpy(&data, mcudata, sizeof(McuData));
    *delta_x = data.delta_x;
    *delta_y = data.delta_y;
}
void readSpeedMcuData(McuData* mcudata, float* speed) {
    SpeedData data;
    memcpy(&data, mcudata, sizeof(McuData));
    *speed = data.speed;
}
/**
 * @brief 提取IMU发送的数据
 * @param mcuData mcu发送的数据结构体
 * @param x yaw degree
 * @param y pitch degree
 * @param z roll degree
 */
void readIMUMcuData(McuData* mcuData, float* yaw, float* pitch, float* roll) {
    pc_com data;
    memcpy(&data, mcuData, sizeof(McuData));
    *yaw = data.x;
    *pitch = data.y;
    *roll = data.z;
}