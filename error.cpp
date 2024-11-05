#include <iostream>
#include <vector>
#include <chrono>        // 時間計測のためのインクルード
#include <unistd.h>      // usleep関数のためのインクルード
#include <iomanip>       // 日時フォーマットのためのインクルード
#include <fstream>       // ファイル出力のためのインクルード
#include <ctime>         // 現在時刻の取得
#include "dynamixel_sdk.h" // Dynamixel SDKのヘッダファイル

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

#define OPERATING_MODE_CURRENT 0 // 電流制御モードを0に設定
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

// 90度に相当するエンコーダの値
#define TARGET_POSITION 1024   // 90度相当
#define DURATION 3.0           // 3秒間

using namespace dynamixel;

// データ記録用の構造体
struct DataRecord {
    double time;         // 時間 (秒)
    int16_t current;     // 電流 (mA)
    int32_t position;    // 角度 (位置データ)
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

    // 電流制限を設定（10 mA）
    int16_t current_limit = 10; // mA
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, current_limit, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 1;
    }

    // 目標電流を設定（3 mA）
    int16_t goal_current = 3; // mA
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_current, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 1;
    }

    // データ記録用
    std::vector<DataRecord> data_log;
    auto start_time = std::chrono::steady_clock::now(); // 計測開始時間

    // 3秒間の制御ループ（線形に角度を変化）
    int32_t initial_position = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&initial_position, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 1;
    }

    for (int i = 0; i < 300; i++) { // 3秒間、100Hzのループ
        // 経過時間の計測
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();

        // 目標位置の計算 (線形に0度から90度まで変化)
        int32_t target_position = initial_position + static_cast<int32_t>(TARGET_POSITION * (elapsed_time / DURATION));
        if (target_position > initial_position + TARGET_POSITION) {
            target_position = initial_position + TARGET_POSITION; // 90度を超えないように
        }

        // 目標位置に基づいた電流設定
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, target_position, &error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            break;
        } else if (error != 0) {
            std::cerr << packetHandler->getRxPacketError(error) << std::endl;
            break;
        }

        // 現在の電流を取得
        int16_t present_current = 0;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT, (uint16_t*)&present_current, &error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            break;
        } else if (error != 0) {
            std::cerr << packetHandler->getRxPacketError(error) << std::endl;
            break;
        }

        // 現在の角度（位置）を取得
        int32_t present_position = 0;
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&present_position, &error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            break;
        } else if (error != 0) {
            std::cerr << packetHandler->getRxPacketError(error) << std::endl;
            break;
        }

        // データを記録
        data_log.push_back({elapsed_time, present_current, present_position});

        // 10ms待機（100Hz）
        usleep(10000);
    }

    // トルクを無効化してモータを停止
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    } else if (error != 0) {
        std::cerr << packetHandler->getRxPacketError(error) << std::endl;
    } else {
        std::cout << "Torque disabled. Motor stopped." << std::endl;
    }

    portHandler->closePort();

    // タイムスタンプを取得し、ファイル名を生成
    std::string timestamp = getCurrentTimestamp();
    std::string filename = "./current_data/" + timestamp + "_data.csv";

    // CSVファイルにデータを保存
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
