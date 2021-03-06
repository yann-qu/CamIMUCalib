/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());  // 把读取到的数据再发送回去
 *    }
 *  }
 * </pre>
 */

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else

#include <unistd.h>

#endif

#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
    usleep(milliseconds * 1000); // 100 ms
#endif
}

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
        serial::PortInfo device = *iter++;

        printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
               device.hardware_id.c_str());
    }
}

void print_usage()
{
    cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int run(int argc, char **argv)
{
    // 第一个参数argv[0]一定是程序的名称，并且包含了程序所在的完整路径，
    // 所以确切的说需要我们输入的main函数的参数个数应该是argc-1个
    if (argc < 2) {
        print_usage();
        return 0;
    }

    // Argument 1 is the serial port or enumerate flag
    string port(argv[1]);

    if (port == "-e") {
        // -e参数会列出所有的可用端口，然后结束程序
        enumerate_ports();
        return 0;
    }
        // 如果不是-e参数，则参数应大于等于3
    else if (argc < 3) {
        print_usage();
        return 1;
    }

    // Argument 2 is the baudrate
    unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
    sscanf_s(argv[2], "%lu", &baud);
#else
    sscanf(argv[2], "%lu", &baud); // 从第2个参数中提取波特率
#endif

    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

    cout << "Is the serial port open?";
    if (my_serial.isOpen())
        cout << " Yes." << endl;
    else
        cout << " No." << endl;

    // Get the Test string
    int count = 0;
    string test_string;
    if (argc == 4) {
        test_string = argv[3];    // 提取测试字符串
    } else {
        test_string = "Testing."; // 没有传入测试字符串时有默认值
    }

    // Test the timeout, there should be 1 second between prints
    cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
    while (count < 10) {
        size_t bytes_wrote = my_serial.write(test_string);           // Write a string to the serial port.

        //? +1的作用？
        string result = my_serial.read(test_string.length() + 1);// Read a given amount of bytes from the serial port into a given buffer.

        cout << "Iteration: " << count << ", Bytes written: ";
        cout << bytes_wrote << ", Bytes read: ";
        cout << result.length() << ", String read: " << result << endl;

        count += 1;
    }

    // Test the timeout at 250ms
    my_serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
    count = 0;
    cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
    while (count < 10) {
        size_t bytes_wrote = my_serial.write(test_string);

        string result = my_serial.read(test_string.length() + 1);

        cout << "Iteration: " << count << ", Bytes written: ";
        cout << bytes_wrote << ", Bytes read: ";
        cout << result.length() << ", String read: " << result << endl;

        count += 1;
    }

    // Test the timeout at 250ms, but asking exactly for what was written
    count = 0;
    cout << "Timeout == 250ms, asking for exactly what was written." << endl;
    while (count < 10) {
        size_t bytes_wrote = my_serial.write(test_string);

        string result = my_serial.read(test_string.length());

        cout << "Iteration: " << count << ", Bytes written: ";
        cout << bytes_wrote << ", Bytes read: ";
        cout << result.length() << ", String read: " << result << endl;

        count += 1;
    }

    // Test the timeout at 250ms, but asking for 1 less than what was written
    count = 0;
    cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
    while (count < 10) {
        size_t bytes_wrote = my_serial.write(test_string);

        string result = my_serial.read(test_string.length() - 1);

        cout << "Iteration: " << count << ", Bytes written: ";
        cout << bytes_wrote << ", Bytes read: ";
        cout << result.length() << ", String read: " << result << endl;

        count += 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    try {
        return run(argc, argv);
    } catch (exception &e) {
        cerr << "Unhandled Exception: " << e.what() << endl;
    }
}
