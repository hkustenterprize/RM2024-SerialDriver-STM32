# 基于CH343P芯片的下位机高速DMA双工无流控无锁环形缓冲的UART通讯方案



## 前言

随着Robomaster赛场上竞争一年比一年激烈，技术迭代水平一年比一年迅速，机器人的代码逻辑也一年比一年复杂，在这种形势下，我们对上下位机的通讯带宽与质量必然需要提出更高的要求。在赛季初期，为了提高通讯速率，我们简单在旧有的通讯模块的基础上直接拉高了波特率，然而效果并不理想，总结如下：

- 随着波特率的增大，上位机下发的信息帧往往呈现出离散化与碎片化的趋势。譬如，两个乃至多个信息帧会杂糅在一起下发，而这个过程仅触发过一次空闲帧；有的时候一个信息帧会拆成多个碎片下发给下位机，而这个过程反而会产生多个空闲帧。
- 在通讯速率提高的同时，信号的抗干扰能力也会随之下降，帧传输错误率也会必然提高，如何在正确地处理这些错误帧的同时防止解包越界等异常操作成为了我们新的研究课题。
- 在这个的赛季我们采用了双头哨兵方案， 上下位机交换的信息多而杂，因此，我们需要在驱动的协议层面设计一种兼容性强的框架来支持对不同信息帧的IO管理。
- 通讯模块暴露出的调用接口应该在保证绝对线程安全性的同时，最大化地利用MCU系统资源如DMA、缓冲区内存以及外设等，以支持高性能的信息吞吐。

之后，我们历经一个月的研发与测试，最终完成了新一代上下位机通讯模块。该模块具有如下特点。

- 高速： 经过测试，该通讯模块可以达到双工4M(收发各2M)的稳定传输速率 （CRC错误率 < 0.01%）

- 无流控：不需要额外使能CTS与RTS功能，不仅节省了IO引脚还间接地提升了通讯效率。

- 无锁：通过巧妙地使用原子操作及设计程序流来规避给共享资源上锁，在避免共享资源抢占的同时提升了系统运行的效率。

- 协议层解耦： 通讯层与协议层分离，方便用户快速地添加、删减修改传输接收的信息帧，同时复用通讯层的功能。

  

在Robomaster 2024的海外赛区及复活赛中，我们在每台机器人上都部署了该通讯方案，并验证了其稳定性。现决定将其开源，以供大家学习点评交流。

## 供电电路

## 基本配置

本模块的部署环境为自研的STM32F407VE和STM32G473VE两款开发板，在STM32CUBEMX中的配置如下。

## 模块综述

我们的串口通讯模块基于HAL库封装的DMA IO函数和FreeRTOS嵌入式操作系统提供的内核级别的支持。

```c
/**
  * @brief Receive an amount of data in DMA mode till either the expected number
  *        of data is received or an IDLE event occurs.
  * @note  Reception is initiated by this function call. Further progress of reception is achieved thanks
  *        to DMA services, transferring automatically received data elements in user reception buffer and
  *        calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *        reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @note  When the UART parity is enabled (PCE = 1), the received data contain
  *        the parity bit (MSB position).
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the received data is handled as a set of uint16_t. In this case, Size must indicate the number
  *        of uint16_t available through pData.
  * @param huart UART handle.
  * @param pData Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param Size  Amount of data elements (uint8_t or uint16_t) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)；
    
/**
  * @brief Send an amount of data in DMA mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param huart UART handle.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be sent.
  * @retval HAL status已经读取
  */
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)；
 
```

然而， 作为一个合格的异步通讯模块封装，直接调用这些HAL库内置的API是不明智的，尤其是在对通讯有较为严苛的要求的情况下。

- **DMA外设的抢占：**假设我们有两个线程，A和B， 两个线程在一个SysTick周期下各调用一次DMA发送函数向上位机发送数据帧。由于我们需要发起DMA请求对指定的一片缓冲区进行读操作，而这个操作需要一定的时间开销，假设线程A先通过DMA发送函数发起请求，DMA开始处理并且搬运线程A指定内存地址的数据， 操作系统若调度至线程B，而线程B此时尝试发起DMA请求而上一轮DMA请求尚未结束，便会造成DMA被抢占的严峻后果。
- **缓冲区资源的抢占：**例如，DMA正在接收来自上位机的数据帧并将其写入RAM的缓冲区中，此时若不对该缓冲区施加一定的保护措施而放任用户进程（CPU）随意地进行访问，便会造成数据帧的污染。

为了弥补原有HAL库层级API对异步支持的不足，在保证对缓冲区的读写安全的同时尽可能提高通讯效率，我们有必要为串口通信模块引入**数据流控制**以及缓冲区的**管理机制**。对此，我们的串口通讯又可以分为TX（发送）与RX（接收）两个部分，两个模块都有一个**环形缓冲区**，各自独立地进行数据流的控制以及资源的调度，而具体实现方式却略有不同。

同时，我们模仿Robomaster裁判系统串口通讯协议设计了模块的串口通讯协议，实现了协议层与驱动层的解**耦**，利于用户定制拓展通讯数据帧，并在驱动层对这些数据帧提供了缓存支持。

### 模块简图



![UART_diagram](/home/fallengold/Documents/RM2024/Open Source/asset/UART_diagram.png)

### 前置知识（1）：环形缓冲区

> **圆形缓冲区**（circular buffer），也称作**圆形队列**（circular queue），**循环缓冲区**（cyclic buffer），**环形缓冲区**（ring buffer），是一种用于表示一个固定尺寸、头尾相连的[缓冲区](https://zh.wikipedia.org/wiki/缓冲区)的数据结构，适合缓存[数据流](https://zh.wikipedia.org/wiki/数据流)。——维基百科

环形队列具有**FIFO特性**（First Input First Output）且由读（read）、写(write)**双指针**维护的数据结构，广泛运用于各个适用于内存存储/交互的场景，例如，Linux内核中的 `kfifo.c` 的实现便依赖于环形缓冲区。

具体实现机制可简要的表述如下：

- 初始化一片**定长** $n$ 的内存（数组）作为储存空间， 其有效存储空间为$n - 1$。初始化读写双指针$i, j$为$-1$，为了规范化指针操作，我们规定$i$永远指向最后一位**已经写入**的内存地址，而$j$永远指向最后一位**已经读取**的内存地址。
- 用户可以向缓冲区写入大小为$s_1$大小的数据，并且更新*写指针*$i \to (i + s_1) \operatorname{mod}(n)$。注意到我们利用了模运算符来实现了缓冲区“环形”的特性，以下读取操作同理。
- 用户可以向缓冲区读取大小为$s_2$大小的数据，并且更新*读指针*$j \to (j + s_2)\operatorname{mod}(n)$。
- 当$i = j \quad (\operatorname{mod}n)$时候，缓冲区为空。当$j = i + 1 \quad (\operatorname{mod}n)$时， 缓冲区为满。在实际为用户设计读写接口时候需要谨慎地处理以上两种情况。

[^一图胜千言]: 

![Circular_Buffer_Animation](/home/fallengold/Documents/RM2024/Open Source/asset/Circular_Buffer_Animation.gif)

环形缓冲区提供了一些额外的好处：

- **无锁：**天然对单读单写（SISO）的操作提供了无锁支持。可以想象，只要不违反环形缓冲区的读写规则，读指针永远不会越过写指针造成非法的数据读取，而这个过程是不需要做上锁处理的。
- **环形DMA**: DMA外设支持配置为环形（Circular）模式， 在这种模式下DMA可以自动对一片内存循环读取，恰好与环形缓冲区的设计理念相合。

### 前置知识（2）：MISO 和 SISO

- SISO（Single Input Single Output）定义了一个系统只有一个输入和输出。
- MISO (Multiple Input Single Output) 定义了一个系统有多个输入和一个输出。
- MIMO (Multiple Input Multiple Output) 定义了一个系统有多个输入和多个输出。

以上概念大多被广泛的运用在控制论上(Control Theory)用来描述系统的特性。 然而，我们也可以意识到，用这些概念描述环形缓冲区乃大有裨益——有的环形缓冲区只需要支持单来源的用户写入，而有的环形缓冲区则可能需要支持多来源的用户输入，而实现这些不同特性的代码可能大相径庭。

### 模块文件结构

本模块使用C++基于OOP的范式进行编写。

```
NewRosComm/
|--- MISOCircularBuffer.hpp
|--- NewRosComm.cpp
|--- NewRosComm.hpp
|--- NewRosCommProtocol.hpp
```

- `MISOCircularBuffer.hpp`：提供了MISO环形缓冲的实现。

- `NewRosComm.*`：串口模块的主体部分，为用户调用提供了接口，负责整个模块的流程控制。

- `NewRosCommProtocol.hpp`：串口模块的协议层， 用户可以在这个文件内部定义新的数据帧格式。

### 发送模块

#### 简述

发送模块内蕴一个环形缓冲区，接受来自**多个**用户线程的写入与TX DMA的读取，因而，这个缓冲区是*MISO*的。前文已经叙述， 环形缓冲区天然对*SISO*的操作提供了无锁支持，然而在*MISO*的条件下，不同的用户线程之间对缓冲区的占用存在竞争。设想情况如下—— 线程A正在向缓冲区写入数据， 而此时操作系统的上下文被SysTick触发被转交给了线程B，而恰好线程B亦试图写入数据，由于此时线程A还来不及更新写指针，因而线程B的写入数据将会覆写线程A的写入区域，造成其被污染的不快后果。

解决这个问题的最直接的方法便是为环形缓冲区赋予一个互斥锁（Mutex），将缓冲区视为一个独占资源，每当一个线程试图访问一个被其他线程占有的缓冲区资源，直到这个缓冲区的独占权被占有的线程释放， 这个线程都会被操作系统阻塞。然而，如果程序中存在大量访问同一个缓冲区的线程，这个过程将会产生大量操作系统的上下文切换，单片机的算力本就紧张，这些额外的性能开销虽然不致命但是足以令人不快。另一个容易想到的办法便是将整个写入的操作声明为原子操作，禁止所有的上下文切换，这虽然可以规避互斥锁所带来的额外的调度开销，然而缺点亦显而易见—— 由于写入内存的操作的用时开销较大，系统便会长时间地处于原子屏蔽的状态中，操作系统的实时性也会因此降低。

本文提出了另外一种替代方案，通过巧妙地设置一些标志位成功地降低了原子操作的开销，同时亦能规避使用互斥锁。

#### 程序解析









### 接收模块





### 协议层















