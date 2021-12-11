#ifndef H_RMSERIAL
#define H_RMSERIAL
#include "com/datatypes.h"
//#include "config/rmconfig.h"
#include "com/serial/serial.h"
#include "glog/logging.h"
#include <thread>
#include <mutex>

extern std::string uart_port;

struct McuConfig {
    uint8_t state;        // 当前状态，自瞄-大符-小符
    uint8_t anti_top;     // 是否为反陀螺模式
    uint8_t enemy_color;  // 敌方颜色
    float curr_yaw;       // 当前云台yaw角速度
    float curr_pitch;     // 当前云台pitch角速度
    float bullet_speed;   // 当前弹速
    int delta_x;          // 能量机关x轴补偿量
    int delta_y;          // 能量机关y轴补偿量
    float yaw, pitch, roll; // 接收IMU数据
};

class RmSerial {
public:
    serial::Serial* active_port;
    std::thread* receive_task;
    ~RmSerial();
    bool init_success;
    bool thread_running;
    bool init();
    bool isConnected() { return active_port->isOpen(); }
    bool send_data(const SendData& data);
    bool send_data(uint8_t* data, size_t size);
    void start_thread();
    void stop_thread();
    void manual_receive();
};

extern std::mutex receive_mtx;
extern McuConfig receive_config_data;
#endif