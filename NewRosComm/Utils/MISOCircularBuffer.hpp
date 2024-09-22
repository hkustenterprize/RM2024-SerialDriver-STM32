#include "FreeRTOS.h"
#include "atomic.h"
#include "string.h"
#include "task.h"


namespace RosComm
{

template <uint16_t tpSize>
class MISOCiruclarBuffer
{
   public:
    MISOCiruclarBuffer() = default;

    constexpr static uint16_t MISO_BUFFER_SIZE = tpSize;

    /**
     * @brief Before writing data into the buffer, the caller should call this function
     * This process will update the tail read index and increment the nested number by 1
     * @param len: The length of the buffer to be written into the buffer
     * @note The atmoic operation is needed in order to
     */

    uint8_t *enterWrite(uint16_t len)
    {
        uint8_t *ptr;
        ATOMIC_ENTER_CRITICAL();
        {
            ptr = buffer + (wIndexTail + 1) % MISO_BUFFER_SIZE;
            wNestedNum++;
            wIndexTail = (wIndexTail + len) % MISO_BUFFER_SIZE;
        }
        ATOMIC_EXIT_CRITICAL();
        return ptr;
    }

    /**
     * @brief Atomically write the data into the buffer given the address and the length
     * @param pData The start address of the data to be written
     * @param len The length of the data to be written
     */

    bool write(uint8_t *pData, uint16_t len)
    {
        bool status = false;
        uint16_t availableSize;

        ATOMIC_ENTER_CRITICAL();
        {
            availableSize = MISO_BUFFER_SIZE - pendingSize;
        }
        ATOMIC_EXIT_CRITICAL();

        if (len > availableSize)
        {
            return status;
        }

        uint8_t *ptr           = this->enterWrite(len);
        uint32_t firstPartSize = (MISO_BUFFER_SIZE - ((uint32_t)ptr - (uint32_t)buffer));

        if (len > firstPartSize)
        {
            memcpy(ptr, pData, firstPartSize);
            memcpy(buffer, pData + firstPartSize, len - firstPartSize);
        }
        else
        {
            memcpy(ptr, pData, len);
        }

        this->exitWrite();
        status = true;
        return status;
    }

    /**
     * @brief After finish writing the data, the caller should called this function to exit its writing process
     * This process will update the head read index and decrement the nested number
     */

    void exitWrite()
    {
        ATOMIC_ENTER_CRITICAL();
        {
            wNestedNum--;
            if (wNestedNum == 0)
            {
                wIndexHead  = wIndexTail;
                pendingSize = (wIndexHead - rIndex + MISO_BUFFER_SIZE) % MISO_BUFFER_SIZE;
            }
        }
        ATOMIC_EXIT_CRITICAL();
    }
    /**
     * @brief Before reading the data, the reader should call this function to initialize the read process
     * @brief This function would increatment the reading nested number by 1 and verify that currently any other is occupying the buffer.
     * @param len The length of the data section to be written
     * @return Suppose the reading is valid, it will return the next readable data section's address, otherwise it would return nullptr
     */
    uint8_t *enterRead(uint16_t len)
    {
        uint8_t *ptr;

        ATOMIC_ENTER_CRITICAL();
        {
            if (rNestedNum > 0 || pendingSize == 0)
            {
                ptr = nullptr;
            }

            else
            {
                readNum = len;
                ptr     = buffer + (rIndex + 1) % MISO_BUFFER_SIZE;
            }
            rNestedNum++;
        }
        ATOMIC_EXIT_CRITICAL();
        return ptr;
    }

    /**
     * @brief After reading the data, the reader should call this function to release the buffer in thread
     * @brief This function would update the read index and decrement the number by 1
     */
    void exitRead()
    {
        ATOMIC_ENTER_CRITICAL();
        {
            rIndex      = (rIndex + readNum) % MISO_BUFFER_SIZE;
            pendingSize = (wIndexTail - rIndex + MISO_BUFFER_SIZE) % MISO_BUFFER_SIZE;
            rNestedNum--;
            readNum = 0;
        }
        ATOMIC_EXIT_CRITICAL();
    }
    /**
     * @brief After reading the data, the reader should call this function to release the buffer in interrupt routine
     * @brief This function would update the read index and decrement the number by 1
     */
    void exitReadFromISR()
    {
        BaseType_t interruptMask = taskENTER_CRITICAL_FROM_ISR();
        {
            rIndex      = (rIndex + readNum) % MISO_BUFFER_SIZE;
            pendingSize = (wIndexTail - rIndex + MISO_BUFFER_SIZE) % MISO_BUFFER_SIZE;
            rNestedNum--;
            readNum = 0;
        }
        taskEXIT_CRITICAL_FROM_ISR(interruptMask);
    }

    /**
     * @brief Get the number of bytes in the buffer pending for reading
     * @return The bytes number
     */
    uint16_t getNextAvailableReadSize() const { return pendingSize; }

    /**
     * @brief Get the number of bytes in the buffer pending for reading considering the discontiniuty in memory address
     * @brief Mainly provide by DMA operating in normal mode
     * @return The bytes number
     */
    uint16_t getNextAvailableReadSizeNoCircular()
    {
        uint16_t size;
        ATOMIC_ENTER_CRITICAL();
        {
            if (wIndexHead < rIndex && rIndex < MISO_BUFFER_SIZE - 1)
            {
                size = MISO_BUFFER_SIZE - rIndex - 1;
            }
            else
            {
                size = pendingSize;
            }
        }
        ATOMIC_EXIT_CRITICAL();
        return size;
    }

    uint16_t getWriteNestedNum() { return wNestedNum; }

    uint16_t getReadNestedNum() { return rNestedNum; }

   private:
    /*Memory storage space for the whole buffer*/
    uint8_t buffer[MISO_BUFFER_SIZE] = {0};
    /*Writing tail index*/
    // This index denotes the farest position that the coroutines have declared to write
    // Update when the routine calls "enterWrite()"
    int16_t wIndexTail = -1;

    /*Write head index*/
    // This index denotes the position that the coroutines have finished to write
    // Update when the routine calls "exitWrite()"
    int16_t wIndexHead = -1;

    /*Read index*/
    // This index denotes the position that the courtines have finished to read
    // Update when the routine calls "enterRead()"
    int16_t rIndex = -1;

    uint16_t readNum = 0;

    /*Write nested number*/
    // This number indicated the nested status of the read operation over the buffer,
    // which means how many routines are now reading the buffer.
    uint16_t wNestedNum  = 0;
    uint16_t rNestedNum  = 0;
    uint16_t pendingSize = 0;
};

}  // namespace RosComm

