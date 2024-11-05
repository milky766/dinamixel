#include <iostream>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <termios.h>
#include <fcntl.h>
#include "dynamixel_sdk.h"

#define PROTOCOL_VERSION 2.0
#define DEVICENAME "/dev/ttyUSB0" // ポート名
#define BAUDRATE 57600             // ボーレート
#define DXL_ID 1

// 制御テーブルアドレス
#define ADDR_OPERATING_MODE 11   // 動作モード
#define ADDR_TORQUE_ENABLE 64    // トルクの有効/無効
#define ADDR_CURRENT_LIMIT 38    // 電流制限
#define ADDR_GOAL_CURRENT 102    // 目標電流
#define ADDR_PRESENT_CURRENT 126 // 現在の電流
#define ADDR_PRESENT_POSITION 132 // 現在の角度

#define OPERATING_MODE_CURRENT 0 // 電流制御モード
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define P_GAIN 1.0               // Pゲイン
#define D_GAIN 0.1               // Dゲイン
#define MAX_CURRENT 20           // 最大電流（20 mA）
#define TARGET_POSITION 1024     // 目標角度（エンコーダ値で90度相当）
#define DURATION 3.0             // 制御の持続時間（3秒）

using namespace dynamixel;

struct DataRecord {
    double time;
    int16_t current;
    int32_t position;
};

// 現在時刻を取得し、YYYYMMDDHHMMSS形式の文字列を返す関数
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm buf;
    localtime_r(&in_time_t, &buf);

    std::ostringstream oss;
    oss << std::put_time(&buf, "%Y%m%d%H%M%S");
    return oss.str();
}

// キーボード入力を非ブロッキングで監視する関数
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main() {
    PortHandler *portHandler = PortHandler::getPortHandler(DEVICENAME);
    PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        std::cerr << "Failed to open port!" << std::endl;
        return 1;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "Failed to set baudrate!" << std::endl;
        return 1;
    }

    uint8_t error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (error != 0) {
        std::cerr << packetHandler->getRxPacketError(error) << std::endl;
    } else {
        std::cout << "Operating mode set to Current Control Mode." << std::endl;
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (error != 0) {
        std::cerr << packetHandler->getRxPacketError(error) << std::endl;
    } else {
        std::cout << "Torque enabled." << std::endl;
    }

    // 電流制限を設定（20 mA）
    int16_t current_limit = MAX_CURRENT; // 最大電流を20mAに設定
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, current_limit, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 1;
    }

    // データ記録用
    std::vector<DataRecord> data_log;
    auto start_time = std::chrono::steady_clock::now(); // 計測開始時間
    int32_t initial_position = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&initial_position, &error);

    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 1;
    }

    int32_t previous_position = initial_position;
    double previous_time = 0.0;

    while (true) {
        // キーボード入力があればループを抜ける
        if (kbhit()) {
            std::cout << "Key pressed! Stopping the motor." << std::endl;
            break;
        }

        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();

        // 3秒経過したらループを抜ける
        if (elapsed_time >= DURATION) {
            std::cout << "3 seconds elapsed. Stopping the motor." << std::endl;
            break;
        }

        // 目標角度との差を計算
        int32_t target_position = initial_position + static_cast<int32_t>(TARGET_POSITION * (elapsed_time / DURATION));
        if (target_position > initial_position + TARGET_POSITION) {
            target_position = initial_position + TARGET_POSITION; // 90度を超えないように
        }

        int32_t present_position = 0;
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&present_position, &error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            break;
        }

        // 角度の差と速度（角度の変化率）を計算
        int32_t position_error = target_position - present_position;
        double velocity = (present_position - previous_position) / (elapsed_time - previous_time);

        // PD制御による電流指令を計算
        int16_t goal_current = static_cast<int16_t>(P_GAIN * position_error - D_GAIN * velocity);
        if (goal_current > MAX_CURRENT) goal_current = MAX_CURRENT;
        if (goal_current < -MAX_CURRENT) goal_current = -MAX_CURRENT;

        // 電流指令を送信
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_current, &error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            break;
        }

        // 現在の電流を取得
        int16_t present_current = 0;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT, (uint16_t*)&present_current, &error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            break;
        }

        // データを記録
        data_log.push_back({elapsed_time, present_current, present_position});

        // 次のループに備えて更新
        previous_position = present_position;
        previous_time = elapsed_time;

        usleep(10000); // 10ms待機（100Hz）
    }

    // トルクを無効化してモータを停止
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &error);
    portHandler->closePort();

    // データをCSVファイルに保存
    std::string filename = "./current_data/" + getCurrentTimestamp() + "_data.csv";
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "Time (s),Current (mA),Position\n";
        for (const auto &record : data_log) {
            file << record.time << "," << record.current << "," << record.position << "\n";
        }
        file.close();
        std::cout << "Data saved to " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }

    return 0;
}
