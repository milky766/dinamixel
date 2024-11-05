#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Control table address
#define ADDR_OPERATING_MODE           11                    // Control Modeのアドレス
#define ADDR_GOAL_CURRENT             102                   // 目標電流のアドレス //decimal = 10(26.9mAぐらいがちょうどいいかも)
#define ADDR_PRESENT_CURRENT          126                   // 現在の電流のアドレス
#define ADDR_TORQUE_ENABLE             64

// Protocol version
#define PROTOCOL_VERSION              2.0                   // Default protocol version of XM430

// Default setting
#define DXL_ID                       1                     // Dynamixel ID
#define BAUDRATE                      57600
#define DEVICENAME                    "/dev/ttyUSB0"        // Port name

#define TORQUE_ENABLE                 1                     // Value for enabling the torque
#define TORQUE_DISABLE                0                     // Value for disabling the torque
#define CURRENT_CONTROL_MODE          0                     // 電流制御モード

// 非同期でキーボード入力があるかチェックする関数
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    // 端末設定を一時変更して非同期入力を確認
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


// 端末の入力モードを設定する関数
void setTerminalMode(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

int main()
{
    // Initialize PortHandler instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort())
        printf("Succeeded to open the port!\n");
    else {
        printf("Failed to open the port!\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
        printf("Succeeded to change the baudrate!\n");
    else {
        printf("Failed to change the baudrate!\n");
        return 0;
    }

    // Set operating mode to current control mode
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, &dxl_error); // x->yはxが指すオブジェクトのメンバyをアクセスする
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    } else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    } else {
        printf("Dynamixel has been successfully set to current control mode\n");
    }

    // Enable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, 64, TORQUE_ENABLE, &dxl_error);

    // Set goal current (current value is in 0.1mA units, so 100 means 10mA)
    int16_t goal_current = 10;  // example current value
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_current, &dxl_error);
    

 // キーボード入力を非同期に監視するための端末設定を有効化
    setTerminalMode(true);
    printf("Press any key to stop the motor...\n");

    // モーターを動作させ続けるループ
    while (true) {
        if (kbhit()) {  // キーボード入力があれば停止
            printf("Key pressed! Stopping the motor.\n");
            packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
            break;
        }
        usleep(10000);  // 10ms待機
    }

    // 端末設定を元に戻す
    setTerminalMode(false);

    // Close port
    portHandler->closePort();
    
    return 0;
}
