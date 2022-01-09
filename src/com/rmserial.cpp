#include "com/rmserial.h"

// receive data
std::mutex receive_mtx;
McuConfig  receive_config_data;

RmSerial::~RmSerial() {
    stop_thread();
}
bool RmSerial::send_data(uint8_t* data, size_t size) {
    return active_port->write(data, size) == size;
}
bool RmSerial::send_data(const SendData& data) {
    return send_data(( uint8_t* )(&data), sizeof(SendData));
}

void proccess_data(uint8_t* s, uint8_t* e) {
    if (e - s != sizeof(McuData)) { // 判断数据大小是否正确
        LOG(ERROR) << "Invalid MCU Data (Invalid Size:" << e - s << ")";
        return;
    }
    McuData mcu_data;
    memcpy(&mcu_data, s, sizeof(McuData)); // 把s中数据拷贝到mcu_data。s最后一位之前已经置0
    if (mcu_data.start_flag != 's') {      // 判断开始位是否正确
        LOG(ERROR) << "Invalid MCU Data (Invalid Data)";
        return;
    }
    receive_mtx.lock();
    switch (mcu_data.type) {
        case MCU_PAN_TYPE:
            readPanMcuData(&mcu_data, &receive_config_data.curr_yaw, &receive_config_data.curr_pitch);
            // 未移植rmconfig
            // if (config.show_mcu_info)
            //     LOG(INFO) << "Recieve Mcu Pan: "
            //               << "Yaw: " << receive_config_data.curr_yaw << " Pitch: " << receive_config_data.curr_pitch;
            break;
        case MCU_CONFIG_TYPE:
            readConfigMcuData(&mcu_data, &receive_config_data.state, &receive_config_data.anti_top, &receive_config_data.enemy_color);
            // 未移植rmconfig
            // if (config.show_mcu_info)
            //     LOG(INFO) << "Recieve Mcu Config: "
            //               << "State: " << receive_config_data.state << " Anti top: " << ( int )receive_config_data.anti_top << " Enemy color:" << ( int )receive_config_data.enemy_color;
            break;
        case MCU_ENERGY_TYPE:
            readEnergyMcuData(&mcu_data, &receive_config_data.delta_x, &receive_config_data.delta_y);
            // 未移植rmconfig
            // if (config.show_mcu_info)
            //     LOG(INFO) << "Recieve Mcu Energy: "
            //               << "Delta x: " << receive_config_data.delta_x << " Delta y: " << receive_config_data.delta_y;
            break;
        case MCU_SPEED_TYPE:
            readSpeedMcuData(&mcu_data, &receive_config_data.bullet_speed);
            // 未移植rmconfig
            // if (config.show_mcu_info) LOG(INFO) << "Recieve Mcu Bullet Speed: " << receive_config_data.bullet_speed;
            break;
        case MCU_IMU_TYPE:
            readIMUMcuData(&mcu_data, &receive_config_data.yaw, &receive_config_data.pitch, &receive_config_data.roll);
            break;
        default:
            break;
    }
    receive_mtx.unlock();
}

void recieve_data_once(RmSerial* rm_serial) {
    static uint8_t  buff[100];
    uint8_t*        buffer_tail    = buff;
    serial::Serial* port           = rm_serial->active_port;
    size_t          wait_in_buffer = port->available(); // 检查number of characters in the buffer.
    if (wait_in_buffer) {                               // 可用
        port->read(buffer_tail, wait_in_buffer);        // Read a given amount of bytes from the serial port into a given buffer.
        buffer_tail += wait_in_buffer;                  // 地址后移
        if (buff[0] != 's') {                           // 判断起始位是否正确。如果不正确就不要刚才读的数据
            buffer_tail = buff;
            return;
        }
        if (buffer_tail - buff < sizeof(McuData)) { // 数据读取的位数不够
            return;
        } else if (buffer_tail - buff == sizeof(McuData)) { // 数据读取的位数足够
            if (buffer_tail[-1] == 'e') {                   // 判断数据是否完整读取，结束位是否正确
                *buffer_tail = 0;                           // 数组最后一位置0
                proccess_data(buff, buffer_tail);           // 处理buff中的数据
                buffer_tail = buff;
            } else {
                buffer_tail = buff;
            }
            return;
        } else {
            buffer_tail = buff;
            return;
        }
    }
}

// 接受数据线程
void recieve_data(RmSerial* rm_serial) {
    LOG(INFO) << "recieve thread started!";
    while (rm_serial->thread_running) {
        recieve_data_once(rm_serial); // 接受一次数据
    }
}
void RmSerial::manual_receive() {
    recieve_data_once(this);
}
void RmSerial::start_thread() { // 开启线程
    if (init_success) {
        thread_running = true;
        receive_task   = new std::thread(recieve_data, this);
        // task.detach();
    }
}
void RmSerial::stop_thread() { // 结束线程
    if (init_success) {
        thread_running = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool RmSerial::init() {
    LOG(INFO) << "Serial Send Size:" << sizeof(SendData);
    LOG(INFO) << "Serial Recieve Size:" << sizeof(McuData);
    try {
        active_port  = new serial::Serial(uart_port, 115200, serial::Timeout::simpleTimeout(1000));
        init_success = true;
    } catch (...) {
        init_success = false;
    }

    // 未移植rmconfig
    //初始化数据接受结构体
    // receive_config_data.anti_top     = config.ANTI_TOP;
    // receive_config_data.bullet_speed = config.BULLET_SPEED;
    // receive_config_data.delta_x      = config.MCU_DELTA_X;
    // receive_config_data.delta_y      = config.MCU_DELTA_Y;
    // receive_config_data.enemy_color  = config.ENEMY_COLOR;
    // receive_config_data.state        = config.RUNMODE;
    //开启数据接受线程
    start_thread();
    if (active_port != nullptr && active_port->isOpen()) {
        LOG(INFO) << "Successfully initialized port " << uart_port;
        return true;
    } else {
        LOG(ERROR) << "failed to initialize port " << uart_port;
        return false;
    }
}
