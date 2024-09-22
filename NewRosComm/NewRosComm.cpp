#include "NewRosComm.hpp"

#include "CRC.hpp"
#include "FreeRTOS.h"
#include "NewRosCommProtocol.hpp"
#include "string.h"

#if USE_NEW_ROS_COMM


namespace RosComm
{

RosManager RosManager::managers[NEW_ROS_NUM];

// Helper Function
RosManager *RosManager::getManager(UART_HandleTypeDef *handle)
{
    for (int i = 0; i < NEW_ROS_NUM; ++i)
    {
        if (handle == RosManager::managers[i].getUARTHandle())
            return &RosManager::managers[i];
    }
    assertFailed((uint8_t *)__FILE__, __LINE__);
    // Wrong parameter
    return nullptr;
}

void RosManager::txTask(void *pvParameters)
{
    RosManager *pManager = (RosManager *)pvParameters;
    uint8_t *pData;
    uint16_t txSize = 0;

    while (true)
    {
        // Is there any pending message
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (not(txSize = pManager->txBuffer.getNextAvailableReadSizeNoCircular()))
        {
#if USE_DEBUG
            pManager->txErrorsCounter[eMISOBufferEmpty]++;
#endif
            continue;
        }
        ATOMIC_ENTER_CRITICAL();
        {
            if ((pData = pManager->txBuffer.enterRead(txSize)) == nullptr)
            {
                pManager->txBuffer.exitRead();
#if USE_DEBUG
                pManager->txErrorsCounter[eMISOBufferMutipleRead]++;
#endif
                continue;
            }
            if (HAL_UART_Transmit_DMA(pManager->huart, pData, txSize) == HAL_OK)
            {
                __HAL_DMA_DISABLE_IT(pManager->huart->hdmatx, DMA_IT_HT);
            }
            else  // Error
            {
                HAL_UART_AbortTransmit(pManager->huart);
                pManager->txBuffer.exitRead();
#if USE_DEBUG
                pManager->txErrorsCounter[eHALNotOK]++;
#endif
            }
        }
        ATOMIC_EXIT_CRITICAL();
    }
}

void RosManager::rxTask(void *pvParameters)
{
    RosManager *pManager = (RosManager *)pvParameters;
    EPackageStatus packageState;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        packageState = ePackageComplete;
        while ((pManager->rxSize = pManager->rxBufferPendingBytes()) && packageState != ePackageHeaderIncomplete &&
               packageState != ePackagePayLoadIncomplete)
        {
            packageState = pManager->updateAndGetNextFrame();
            if (packageState == ePackageComplete)
            {
                pManager->connected         = true;
                pManager->disConnectCounter = 0;
                uint8_t protocolID          = pManager->rxFrameBuffer.frame.header.protocolID;
                uint16_t len                = pManager->rxFrameBuffer.frame.header.dataLen;
                for (uint8_t j = 0; j < pManager->frameCallbackCounter; j++)
                {
                    if (pManager->callbackTable[j].id != protocolID)
                        continue;
                    pManager->callbackTable[j].func(pManager->rxFrameBuffer.frame.payLoadCRC16, len, pManager->huart);
                }
            }
// Debug
#if USE_DEBUG
            pManager->packageStateCnt[packageState]++;
            pManager->completePackageReceiveFreq = pManager->packageStateCnt[ePackageComplete] / (xTaskGetTickCount() / 1000.0f);
            pManager->crcErrorRate = (float)(pManager->packageStateCnt[ePackageCRCError] + pManager->packageStateCnt[ePackageHeaderCRCError]) /
                                     (pManager->packageStateCnt[ePackageComplete]);
#endif
        }
    }
}

void RosManager::rxTimerFunc(TimerHandle_t xTimer)
{
    RosManager *pManager = (RosManager *)(pvTimerGetTimerID(xTimer));
    bool lastConnected   = pManager->connected;
    if (++pManager->disConnectCounter > NEW_ROS_RX_TIMEOUT)
    {
        pManager->connected         = false;
        pManager->disConnectCounter = NEW_ROS_RX_TIMEOUT;
    }

    if (lastConnected && not pManager->connected)
    {
        ATOMIC_ENTER_CRITICAL();
        {
            HAL_UART_AbortReceive(pManager->huart);
            HAL_UARTEx_ReceiveToIdle_DMA(pManager->huart, pManager->rosRxCircularBuff, NEW_ROS_RX_BUF_SIZE);
            pManager->rxRIndex = pManager->rxWIndex = -1;
        }
        ATOMIC_EXIT_CRITICAL();
    }
}

uint16_t RosManager::rxBufferPendingBytes()
{
    uint16_t bytes;
    ATOMIC_ENTER_CRITICAL();
    {
        bytes = (this->rxWIndex - this->rxRIndex + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;
    }
    ATOMIC_EXIT_CRITICAL();
    return bytes;
}

RosManager::EPackageStatus RosManager::updateAndGetNextFrame()
{
    uint16_t size                               = this->rxSize;
    static constexpr uint16_t CRC16_SIZE        = 2;
    static constexpr uint16_t HEADER_CRC16_SIZE = sizeof(FrameHeader) + CRC16_SIZE;

    // Find the start byte
    uint16_t rPtr = this->rxRIndex;
    while (size > HEADER_CRC16_SIZE)
    {
        rPtr = (rPtr + 1) % NEW_ROS_RX_BUF_SIZE;
        if (rosRxCircularBuff[rPtr] == RosComm::START_BYTE)
        {
            break;
        }
        size--;
    }

    // Verify the completeness of the potential header
    if (size <= HEADER_CRC16_SIZE)
    {
        this->rxRIndex = (rPtr - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;  // START_BYTES - 1
        return ePackageHeaderIncomplete;
    }

    // Copy the buffer into the header
    if (rPtr + HEADER_CRC16_SIZE <= NEW_ROS_RX_BUF_SIZE)
    {
        memcpy(rxFrameBuffer.array, &rosRxCircularBuff[rPtr], HEADER_CRC16_SIZE);
    }
    else
    {
        uint16_t firstPartSize = NEW_ROS_RX_BUF_SIZE - rPtr;
        memcpy(rxFrameBuffer.array, &rosRxCircularBuff[rPtr], firstPartSize);
        memcpy(rxFrameBuffer.array + firstPartSize, rosRxCircularBuff, HEADER_CRC16_SIZE - firstPartSize);
    }
    size -= HEADER_CRC16_SIZE;
    rPtr = (rPtr + HEADER_CRC16_SIZE) % NEW_ROS_RX_BUF_SIZE;

    // Verify the Header CRC
    if (!Crc::verifyCRC16CheckSum(rxFrameBuffer.array, HEADER_CRC16_SIZE))
    {
        this->rxRIndex = (rPtr - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;
        return ePackageHeaderCRCError;
    }

    // Read the payload length from the header
    uint16_t payLoadLen = rxFrameBuffer.frame.header.dataLen;
    if (payLoadLen > NEW_ROS_MAX_PACKAGE_SIZE)
    {
        this->rxRIndex = (rPtr - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;
        return ePackageHeaderCRCError;
    }

    // Verify the completeness of the potential payload
    if (size < payLoadLen + CRC16_SIZE)
    {
        this->rxRIndex = (rPtr - HEADER_CRC16_SIZE - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;  // Start bytes
        return ePackagePayLoadIncomplete;
    }

    // Copy the payload and 16CRC into the buffer
    if (rPtr + payLoadLen + CRC16_SIZE <= NEW_ROS_RX_BUF_SIZE)
    {
        memcpy(rxFrameBuffer.frame.payLoadCRC16, &rosRxCircularBuff[rPtr], payLoadLen + CRC16_SIZE);
    }
    else
    {
        uint16_t firstPartSize = NEW_ROS_RX_BUF_SIZE - rPtr;
        memcpy(rxFrameBuffer.frame.payLoadCRC16, &rosRxCircularBuff[rPtr], firstPartSize);
        memcpy(rxFrameBuffer.frame.payLoadCRC16 + firstPartSize, rosRxCircularBuff, payLoadLen + CRC16_SIZE - firstPartSize);
    }
    rPtr = (rPtr + payLoadLen + CRC16_SIZE) % NEW_ROS_RX_BUF_SIZE;

    // Verify the payload CRC
    if (!Crc::verifyCRC16CheckSum(rxFrameBuffer.array, HEADER_CRC16_SIZE + payLoadLen + CRC16_SIZE))
    {
        this->rxRIndex = (rPtr - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;
        return ePackageCRCError;
    }

    // CHEERS! The package is complete
    this->rxRIndex = (rPtr - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;
    return ePackageComplete;
}

void RosManager::rxCallback(UART_HandleTypeDef *handle, uint16_t size)
{
    // traceISR_ENTER();
    RosManager *pManager                = getManager(handle);
    uint16_t wPtr                       = (size - 1 + NEW_ROS_RX_BUF_SIZE) % NEW_ROS_RX_BUF_SIZE;
    pManager->rxWIndex                  = wPtr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(pManager->rxTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISRa(xHigherPriorityTaskWoken);
    // traceISR_EXIT();
}

void RosManager::txCpltCallback(UART_HandleTypeDef *handle)
{
    // traceISR_ENTER();
    RosManager *pManager = getManager(handle);
    pManager->txBuffer.exitReadFromISR();
    if (pManager->txBuffer.getNextAvailableReadSizeNoCircular())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(pManager->txTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    // traceISR_EXIT();
}

void RosManager::rxErrorCallback(UART_HandleTypeDef *handle)
{
    RosManager *pManager     = getManager(handle);
    BaseType_t interruptMask = pdFALSE;
    interruptMask            = taskENTER_CRITICAL_FROM_ISR();
    {
        HAL_UART_AbortReceive(handle);
        HAL_UARTEx_ReceiveToIdle_DMA(handle, pManager->rosRxCircularBuff, NEW_ROS_RX_BUF_SIZE);
        pManager->rxRIndex = pManager->rxWIndex = -1;
#if USE_DEBUG
        pManager->rxErrorCounter++;
#endif
    }
    taskEXIT_CRITICAL_FROM_ISR(interruptMask);
}

void RosManager::registerFrameCallback(const uint8_t &id, FrameCallback callback)
{
    configASSERT(frameCallbackCounter < NEW_ROS_MAX_FRAME_CALLBACK_NUM);
    for (uint8_t i = 0; i < frameCallbackCounter; i++)
    {
        configASSERT(callbackTable[i].id != id);
    }
    callbackTable[frameCallbackCounter].id   = id;
    callbackTable[frameCallbackCounter].func = callback;
    frameCallbackCounter++;
}

void RosManager::init(UART_HandleTypeDef *handle)
{
    if (rosCommInitialized)
    {
        return;
    }

    huart = handle;
    // UART configuration assertion check
    configASSERT(huart->Init.BaudRate == 2000000);                   // Baudrate 2M
    configASSERT(huart->Init.HwFlowCtl == UART_HWCONTROL_NONE);      // No hardware control
    configASSERT(huart->Init.Mode == UART_MODE_TX_RX);               // TX and RX mode
    configASSERT(huart->Init.Parity == UART_PARITY_NONE);            // No parity
    configASSERT(huart->Init.StopBits == UART_STOPBITS_2);           // 2 stop bit
    configASSERT(huart->Init.WordLength == UART_WORDLENGTH_8B);      // 8 bit word length
    configASSERT(huart->Init.OverSampling == UART_OVERSAMPLING_16);  // 16 bit oversampling
#if defined(STM32G473xx)
    configASSERT(huart->Init.OneBitSampling == UART_ONE_BIT_SAMPLE_ENABLED);  // One bit sample disable
#endif

    // RX DMA configuration assertion check
    configASSERT(huart->hdmarx->Init.Mode == DMA_CIRCULAR);                     // Circular DMA mode
    configASSERT(huart->hdmarx->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE);  // 8 bit data alignment
    configASSERT(huart->hdmarx->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE);
    configASSERT(huart->hdmarx->Init.MemInc == DMA_MINC_ENABLE);      // Memory increment mode enable
    configASSERT(huart->hdmarx->Init.PeriphInc == DMA_PINC_DISABLE);  // Peripheral increment mode disable

    // TX DMA configuration assertion check
    configASSERT(huart->hdmatx->Init.Mode == DMA_NORMAL);                       // Normal DMA mode
    configASSERT(huart->hdmatx->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE);  // 8 bit data alignment
    configASSERT(huart->hdmatx->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE);
    configASSERT(huart->hdmatx->Init.MemInc == DMA_MINC_ENABLE);      // Memory increment mode enable
    configASSERT(huart->hdmatx->Init.PeriphInc == DMA_PINC_DISABLE);  // Peripheral increment mode disable

    configASSERT(HAL_UART_RegisterCallback(huart, HAL_UART_TX_COMPLETE_CB_ID, txCpltCallback) == HAL_OK);
    configASSERT(HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, rxErrorCallback) == HAL_OK);
    configASSERT(HAL_UART_RegisterRxEventCallback(huart, rxCallback) == HAL_OK);

    txTaskHandle   = xTaskCreateStatic(txTask, "ros tx task", NEW_ROS_TX_TASK_STACK_SIZE, this, 15, uxTxTaskStack, &txTaskTCB);
    rxTimeoutTimer = xTimerCreateStatic("ros rx timeout timer", pdMS_TO_TICKS(1), true, this, rxTimerFunc, &rxTimeoutTimerBuffer);

    rxTaskHandle = xTaskCreateStatic(rxTask, "ros rx task", NEW_ROS_RX_TASK_STACK_SIZE, this, 15, uxRxTaskStack, &rxTaskTCB);

    configASSERT(HAL_UARTEx_ReceiveToIdle_DMA(huart, rosRxCircularBuff, NEW_ROS_RX_BUF_SIZE) == HAL_OK);
    rosCommInitialized = true;
}

void RosManager::transmit(FrameHeader &header, uint8_t *data)
{
    configASSERT(isInitialized());

    uint16_t totalSize                          = sizeof(FrameHeader) + header.dataLen + 4;  // header + crc + data + crc
    static constexpr uint16_t HEADER_CRC16_SIZE = sizeof(FrameHeader) + 2;                   // header + crc

    uint8_t package[NEW_ROS_MAX_PACKAGE_SIZE] = {0};
    memcpy(package, &header, sizeof(FrameHeader));
    Crc::appendCRC16CheckSum(package, HEADER_CRC16_SIZE);
    memcpy(package + HEADER_CRC16_SIZE, data, header.dataLen);
    Crc::appendCRC16CheckSum(package, totalSize);

    // The tx buffer is full
    if (not txBuffer.write(package, totalSize))
    {
#if USE_DEBUG
        txAbortPackageCounter++;
#endif
        return;
    }

    // The buffer is not read by DMA
    if (txBuffer.getReadNestedNum() == 0 && txBuffer.getNextAvailableReadSize() > 0)
        xTaskNotifyGive(txTaskHandle);
}
}  // namespace RosComm


#endif  // endif for USE_ROS_COMM