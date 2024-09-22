/**
 * @file (New)RosComm.cpp
 * @author ZOU Hetai (hzouah@connect.ust.hk) RM2023
 * @author Lawrence Ruan, Fallengold (zguobd@connect.ust.hk) RM2024
 * @version 0.2
 * @date 2024-6-13
 * @copyright Copyright (c) 2024
 * @brief  Lightning ROS Communication Half-duplex 2M UART Driver, featuring CH434P chips
 * ==========================================================================
 * @note   CUBEMX Configuration:
 * [UART:  Baudrate 2M, No hardware control, TX and RX mode,
 *         No parity, 2 stop bit, 8 bit word length,
 *         16 bit oversampling]
 *
 * [RXDMA: Circular DMA mode, bytes alignment,
 *         Memory increment mode enable, Peripheral increment mode disable]
 *
 * [TXDMA: Normal DMA mode, bytes alignment,
 *         Memory increment mode enable, Peripheral increment mode disable]
 * ===========================================================================
 * @note   Recommended Application configuration:
 * [ROSNUM: 2(Sentry); 1(Other robots)]
 * [ROS_MAXPACKAGE_SIZE: 64]
 * [ROS_RX_BUF_SIZE: 1024]
 * [ROS_TX_BUF_SIZE: 512]
 * ===========================================================================
 * @note   See the test backlog here:
 * https://app.gitbook.com/o/Ge5THhzazmpoHzdmQGwe/s/Fxaib9jlhGy3w6HfD88w/algorithm/usb-to-ttl-driver
 */

#pragma once

#include "NewRosCommConfig.h"

#if USE_NEW_ROS_COMM

#include "FreeRTOS.h"
#include "MISOCircularBuffer.hpp"
#include "NewRosCommProtocol.hpp"
#include "atomic.h"
#include "main.h"
#include "task.h"
#include "timers.h"

#ifndef NEW_ROS_NUM
#define NEW_ROS_NUM 1
#endif

#ifndef NEW_ROS_MAX_PACKAGE_SIZE
#define NEW_ROS_MAX_PACKAGE_SIZE 64
#endif

#ifndef NEW_ROS_RX_BUF_SIZE
#define NEW_ROS_RX_BUF_SIZE 1024
#endif

// Note, the rx buffer size is recommended to be as same with the tx buffer of the rm_serial_driver
#ifndef NEW_ROS_TX_BUF_SIZE
#define NEW_ROS_TX_BUF_SIZE 512
#endif

#ifndef NEW_ROS_RX_TIMEOUT
#define NEW_ROS_RX_TIMEOUT 50
#endif

#ifndef NEW_ROS_MAX_FRAME_CALLBACK_NUM
#define NEW_ROS_MAX_FRAME_CALLBACK_NUM 4
#endif

#define NEW_ROS_TX_TASK_STACK_SIZE 256
#define NEW_ROS_RX_TASK_STACK_SIZE 512

namespace Core
{

namespace Communication
{

namespace RosComm
{

using FrameCallback = void (*)(uint8_t *, uint16_t len, UART_HandleTypeDef *handle);
// Structure to store Process Function element
struct FrameCallbackElement
{
    uint8_t id;
    FrameCallback func;
};

class RosManager
{
   public:
    /**
     *  @brief Init ROSManager
     *  @param handle pointer of UART handle
     **/
    void init(UART_HandleTypeDef *handle);

    /**
     * @brief Transmit the message
     * @param rosheader: the header of the message, without 16CRC
     * @param data: The payload data to be packed in the message
     */
    void transmit(FrameHeader &rosheader, uint8_t *data);

    /***
     * @brief Bind user callback function to a particular frame / message on reception
     * @param id in NewRosCommProtocol, CommunicationType
     * @param callback The frame callback function
     **/
    void registerFrameCallback(const uint8_t &id, FrameCallback callback);

    /**
     * @brief Check if the connection is established
     */
    bool isConnected() { return connected; }

    /**
     * @brief Check if the ROSManager is initialized
     */
    bool isInitialized() { return rosCommInitialized; }

    /**
     * @brief Get the UART handle
     */
    UART_HandleTypeDef *getUARTHandle() const { return huart; }

    static RosManager managers[NEW_ROS_NUM];

   private:
    static void rxTimerFunc(TimerHandle_t xTimer);

    static void txCpltCallback(UART_HandleTypeDef *handle);

    static void rxErrorCallback(UART_HandleTypeDef *handle);

    static void rxCallback(UART_HandleTypeDef *handle, uint16_t size);

    FrameCallbackElement callbackTable[NEW_ROS_MAX_FRAME_CALLBACK_NUM];
    uint8_t frameCallbackCounter = 0;

    static RosManager *getManager(UART_HandleTypeDef *handle);

    UART_HandleTypeDef *huart;

    bool connected = false;

    uint16_t disConnectCounter = 0;

    // MISO Circular Buffer, implemented in the MISOCircularBuffer.hpp
    // Multiple Input: User threads
    // Single Output: DMA
    MISOCiruclarBuffer<NEW_ROS_TX_BUF_SIZE> txBuffer;
    int16_t rxSize   = 0;
    int16_t rxWIndex = -1;  // write index
    int16_t rxRIndex = -1;  // read index

    // SISO Circular Buffer
    uint8_t rosRxCircularBuff[NEW_ROS_RX_BUF_SIZE];

    StaticTask_t txTaskTCB;
    StackType_t uxTxTaskStack[NEW_ROS_TX_TASK_STACK_SIZE];
    static void txTask(void *pvParameters);
    TaskHandle_t txTaskHandle;

    StaticTimer_t rxTimeoutTimerBuffer;
    TimerHandle_t rxTimeoutTimer;

    StaticTask_t rxTaskTCB;
    StackType_t uxRxTaskStack[NEW_ROS_RX_TASK_STACK_SIZE];
    static void rxTask(void *pvParameters);
    TaskHandle_t rxTaskHandle;

    // Check how many bytes are pending in the rx buffer, waiting for decode
    uint16_t rxBufferPendingBytes();

    // Package status
    // For every call on updateAnGetNextFrame, the function will try to decode a package, and return the status of the package
    enum EPackageStatus
    {
        ePackageComplete = 0,
        ePackageHeaderIncomplete,
        ePackagePayLoadIncomplete,
        ePackageHeaderCRCError,
        ePackageCRCError,
        ePackageStateCount
    };

    volatile bool rosCommInitialized = false;

    // Update and get the next frame in the rx circular buffer
    EPackageStatus updateAndGetNextFrame();

    // Buffer for one decoded frame
    // On successful reception and decoding of a frame, the frame will be stored in this buffer and return to the user
    union Frame
    {
        struct
        {
            FrameHeader header;
            uint16_t crc;
            uint8_t payLoadCRC16[NEW_ROS_MAX_PACKAGE_SIZE + 2];
        } __attribute__((packed)) frame = {{0, 0, 0}, 0, {0}};

        uint8_t array[sizeof(FrameHeader) + NEW_ROS_MAX_PACKAGE_SIZE + 4];
    } rxFrameBuffer;
#if USE_DEBUG
    uint32_t packageStateCnt[ePackageStateCount] = {0};  // Debug
    float completePackageReceiveFreq             = 0;
    float crcErrorRate                           = 0;
    uint32_t rxErrorCounter                      = 0;
    uint32_t txAbortPackageCounter               = 0;

    enum ETxError
    {
        eMISOBufferEmpty       = 0,
        eMISOBufferMutipleRead = 1,
        eHALNotOK              = 2,
        eTxErrorCount
    };

    uint32_t txErrorsCounter[eTxErrorCount] = {0};

#endif
};

}  // end of namespace RosComm
}  // end of namespace Communication
}  // end of namespace Core

#endif  //