#include "dynamixel_sdk.h"  // Uses Dynamixel SDK library
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <string>
#include <iostream>
#include <filesystem>
#include <thread>
#include <atomic>
#include <limits>
#include <cmath>

// 制御用のアドレスなど
#define ADDR_OPERATING_MODE           11                    
#define ADDR_GOAL_CURRENT             102                   
#define ADDR_PRESENT_POSITION         132                   
#define ADDR_PRESENT_CURRENT          126                   
#define ADDR_TORQUE_ENABLE            64
#define ADDR_TORQUE_LIMIT             40 // Torque Limitのアドレス（公式Control Tableを確認）

#define PROTOCOL_VERSION              2.0                   
#define DXL_ID1                       1                     
#define DXL_ID2                       2                     
#define BAUDRATE                      57600
#define DEVICENAME                    "/dev/ttyUSB0"        

#define TORQUE_ENABLE                 1                     
#define TORQUE_DISABLE                0                     
#define CURRENT_CONTROL_MODE          0

//エラーを表示


std::atomic<bool> stop_flag(false);  // モーター停止フラグ

// エラーコードをビットごとに解析して表示する関数
void printDxlError(uint8_t error) {
    if (error == 0) return;
    std::cerr << "エラー内容: ";
    if (error & 0x01) std::cerr << "Input Voltage Error ";
    if (error & 0x02) std::cerr << "Angle Limit Error ";
    if (error & 0x04) std::cerr << "Overheating Error ";
    if (error & 0x08) std::cerr << "Range Error ";
    if (error & 0x10) std::cerr << "Checksum Error ";
    if (error & 0x20) std::cerr << "Overload Error ";
    if (error & 0x40) std::cerr << "Instruction Error ";
    
    std::cerr << std::endl;
}

// モーターの位置と電流を取得する関数
void getMotorData(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int id, int32_t& position, int16_t& current) {
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    // 現在位置の取得
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, (uint32_t*)&position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " の位置取得に失敗しました: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Position): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
    }

    // 現在電流の取得
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_CURRENT, (uint16_t*)&current, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " の電流取得に失敗しました: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Current): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
    }
}

// モーターの設定を行う関数
bool setupMotor(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    std::cout << "Setting up motor ID: " << id << std::endl;

    // 1. トルクを無効化
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " のTorque Disableに失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Torque Disable): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
        return false;
    }

    // 2. オペレーティングモードの設定を電流制御モードに変更
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " のオペレーティングモード設定に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Operating Mode): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
        return false;
    }

    // 3. Goal Currentを0に設定
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_CURRENT, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " のGoal Current設定に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Goal Current): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
        return false;
    }

    // 4. Torque Limitの設定（例: 最大電流500に変更）
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_TORQUE_LIMIT, 500, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " のTorque Limit設定に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Torque Limit): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
        return false;
    }

    // 5. トルクの有効化
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Motor " << id << " のトルク有効化に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return false;
    }
    if (dxl_error != 0) {
        std::cerr << "Motor " << id << " RxPacketError (Torque Enable): " << static_cast<int>(dxl_error) << std::endl;
        printDxlError(dxl_error);
        return false;
    }

    return true;
}

// キーボード入力を監視するスレッド
void monitorInput() {
    std::cout << "Press Enter to stop the motors...\n";
    std::cin.get();  // Enterキーを押すのを待つ
    stop_flag = true;
}

// 目標位置の線形軌道生成関数
int32_t calculateTargetPosition(int32_t start_pos, int32_t goal_pos, double t, double duration) {
    if (t >= duration) {
        return goal_pos;
    }
    double ratio = t / duration;
    return static_cast<int32_t>(start_pos + ratio * (goal_pos - start_pos));
}

int main() {
    // ログファイルの設定
    std::string user_input;
    std::cout << "Enter a name for the data log (e.g., run1): ";
    std::cin >> user_input;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // 入力バッファのクリア

    std::string directory = "angle_current";
    std::string filename = "angle_current_" + user_input + ".csv";
    std::filesystem::create_directory(directory); 
    std::ofstream file(directory + "/" + filename);
    file << "Time(s),Position1,Current1,Position2,Current2\n";

    // Dynamixelの初期化
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        std::cerr << "Failed to open port!\n";
        return 0;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "Failed to set baudrate!\n";
        portHandler->closePort();
        return 0;
    }

    // モータのセットアップ（電流制御モード）
    if (!setupMotor(packetHandler, portHandler, DXL_ID1) ||
        !setupMotor(packetHandler, portHandler, DXL_ID2)) {
        std::cerr << "Failed to initialize motors.\n";
        portHandler->closePort();
        return 0;
    }

    // 初期位置の取得
    int32_t start_position1, start_position2;
    int16_t current1, current2;
    getMotorData(packetHandler, portHandler, DXL_ID1, start_position1, current1);
    getMotorData(packetHandler, portHandler, DXL_ID2, start_position2, current2);

    // 目標位置の設定
    int32_t goal_position1 = start_position1 + static_cast<int32_t>((4096.0 / 360.0) * 90);  // 90度動かす
    int32_t goal_position2 = start_position2 - static_cast<int32_t>((4096.0 / 360.0) * 90);  // 反対方向に90度動かす

    auto start_time = std::chrono::system_clock::now();
    std::thread inputThread(monitorInput);

    double duration = 1.0; // 1秒で動作を完了させる
    double dt = 0.01; // 制御ループの周期（10ms）

    // PID制御のパラメータ（初期値を低めに設定）
    double Kp = 5.0; // 比例ゲイン
    double Kd = 0.5; // 微分ゲイン

    // 前回の誤差を保存する変数
    double previous_error1 = 0.0, previous_error2 = 0.0;

    // 電流の最大値（XM430-W350の場合、範囲は -2048 ~ +2047）
    const int16_t MAX_CURRENT = 500;
    const int16_t MIN_CURRENT = 0;

    while (true) {
        if (stop_flag) {
            std::cout << "Stop flag detected. Exiting loop.\n";
            break;
        }

        auto now = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0;

        if (elapsed > duration) {
            break; // 1秒経過したらループを抜ける
        }

        // 目標位置の計算
        int32_t target_position1 = calculateTargetPosition(start_position1, goal_position1, elapsed, duration);
        int32_t target_position2 = calculateTargetPosition(start_position2, goal_position2, elapsed, duration);

        // 現在の位置を取得
        int32_t present_position1, present_position2;
        getMotorData(packetHandler, portHandler, DXL_ID1, present_position1, current1);
        getMotorData(packetHandler, portHandler, DXL_ID2, present_position2, current2);

        // 位置誤差の計算
        double error1 = static_cast<double>(target_position1 - present_position1);
        double error2 = static_cast<double>(target_position2 - present_position2);

        // PD制御計算
        double derivative1 = (error1 - previous_error1) / dt;
        double derivative2 = (error2 - previous_error2) / dt;

        double output_current1 = Kp * error1 + Kd * derivative1;
        double output_current2 = Kp * error2 + Kd * derivative2;

        previous_error1 = error1;
        previous_error2 = error2;

        // 電流の制限
        int16_t goal_current1 = static_cast<int16_t>(
            std::max(
                std::min(output_current1, static_cast<double>(MAX_CURRENT)),
                static_cast<double>(MIN_CURRENT)
            )
        );
        int16_t goal_current2 = static_cast<int16_t>(
            std::max(
                std::min(output_current2, static_cast<double>(MAX_CURRENT)),
                static_cast<double>(MIN_CURRENT)
            )
        );

        // ゴール電流を送信
        int dxl_comm_result;
        uint8_t dxl_error_code;

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_CURRENT, goal_current1, &dxl_error_code);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Motor1 のゴール電流送信に失敗しました: " 
                      << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        }
        if (dxl_error_code != 0) {
            std::cerr << "Motor1 RxPacketError: " << static_cast<int>(dxl_error_code) << std::endl;
            printDxlError(dxl_error_code);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_CURRENT, goal_current2, &dxl_error_code);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Motor2 のゴール電流送信に失敗しました: " 
                      << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        }
        if (dxl_error_code != 0) {
            std::cerr << "Motor2 RxPacketError: " << static_cast<int>(dxl_error_code) << std::endl;
            printDxlError(dxl_error_code);
        }

        // データの記録
        file << elapsed << "," << present_position1 << "," << current1 << "," 
             << present_position2 << "," << current2 << "\n";
        file.flush();

        // 制御ループの周期待機
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    // 目標電流をゼロに設定してモータを停止
    int dxl_comm_result_stop;
    uint8_t dxl_error_code_stop;

    dxl_comm_result_stop = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_CURRENT, 0, &dxl_error_code_stop);
    if (dxl_comm_result_stop != COMM_SUCCESS) {
        std::cerr << "Motor1 のゴール電流停止送信に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result_stop) << std::endl;
    }
    if (dxl_error_code_stop != 0) {
        std::cerr << "Motor1 RxPacketError (Stop): " << static_cast<int>(dxl_error_code_stop) << std::endl;
        printDxlError(dxl_error_code_stop);
    }

    dxl_comm_result_stop = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_CURRENT, 0, &dxl_error_code_stop);
    if (dxl_comm_result_stop != COMM_SUCCESS) {
        std::cerr << "Motor2 のゴール電流停止送信に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result_stop) << std::endl;
    }
    if (dxl_error_code_stop != 0) {
        std::cerr << "Motor2 RxPacketError (Stop): " << static_cast<int>(dxl_error_code_stop) << std::endl;
        printDxlError(dxl_error_code_stop);
    }

    // トルクの無効化と後片付け
    dxl_comm_result_stop = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_code_stop);
    if (dxl_comm_result_stop != COMM_SUCCESS) {
        std::cerr << "Motor1 のトルク無効化に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result_stop) << std::endl;
    }
    if (dxl_error_code_stop != 0) {
        std::cerr << "Motor1 RxPacketError (Torque Disable): " << static_cast<int>(dxl_error_code_stop) << std::endl;
        printDxlError(dxl_error_code_stop);
    }

    dxl_comm_result_stop = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_code_stop);
    if (dxl_comm_result_stop != COMM_SUCCESS) {
        std::cerr << "Motor2 のトルク無効化に失敗しました: " 
                  << packetHandler->getTxRxResult(dxl_comm_result_stop) << std::endl;
    }
    if (dxl_error_code_stop != 0) {
        std::cerr << "Motor2 RxPacketError (Torque Disable): " << static_cast<int>(dxl_error_code_stop) << std::endl;
        printDxlError(dxl_error_code_stop);
    }

    stop_flag = true;
    inputThread.join();
    file.close();
    portHandler->closePort();
    return 0;
}
