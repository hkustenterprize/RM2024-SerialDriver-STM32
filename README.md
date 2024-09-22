# 文件结构说明

## 文件树



```markdown
├── asset 						## 素材文件
│   ├── Circular_Buffer_Animation.gif
│   ├── experiment_host.png
│   ├── experiment_MCU.png
│   ├── experiment_systemview.png
│   ├── RXDMA_Config.png
│   ├── RX_unpack_diagram.drawio
│   ├── RX_unpack_diagram.drawio.png
│   ├── sch_ttl_ch343p.png
│   ├── UART_diagram.excalidraw
│   └── UART_diagram.png
├── Doc  						## 开源报告 + 下位机驱动文档
│   ├── serial_driver_MCU_doc.md
│   └── UART.pdf
├── NewRosComm  				## 下位机驱动
│   ├── NewRosCommConfig.hpp
│   ├── NewRosComm.cpp
│   ├── NewRosComm.hpp
│   ├── NewRosComm.mk
│   ├── NewRosCommProtocol.hpp
│   └── Utils
│       ├── CRC.cpp
│       ├── CRC.hpp
│       └── MISOCircularBuffer.hpp
└── README.md
```

- `MISOCircularBuffer.hpp`：提供了MISO环形缓冲的实现。
- `NewRosComm.*`：串口模块的主体部分，为用户调用提供了接口，负责整个模块的流程控制。
- `NewRosCommProtocol.hpp`：串口模块的协议层， 用户可以在这个文件内部定义新的数据帧格式。
- `CRC.*` 提供了数据包CRC校验算法的实现与支持。

## 使用教程



本模块使用C++基于OOP的范式进行编写。文件的引用路径已经包含在`NewRosComm.mk`，使用时需要在主`Makefile`文件中手动添加模块的路，定义该模块的根目录`NewRosComm_PATH`，并手动将引用文件 `NewRosComm_INC`与源文件`NewRosComm_PATH_SRC` 路径添加进编译环境里面

```makefile
# Your Makefile
# Define Root directory of the module
NewRosComm_PATH = $(YOUR_DIR)/NewRosComm

# Include makefile
include $(NewRosComm_PATH)/NewRosComm.mk

# ...
# Append source files
CPP_SRC += $(NewRosComm_SRC) \
# ...

# Append include path
CPP_INCLUDES += $(NewRosComm_INC) \
```

```cpp
# NewRosComm.mk

# Sources
NewRosComm_SRC = 
$(NewRosComm_PATH)/NewRosComm.cpp \
$(NewRosComm_PATH)/Utils/CRC.cpp


# Includes
NewRosComm_INC =  \
-I$(NewRosComm_PATH) \
-I$(NewRosComm_PATH)/Utils
```



**-注意事项-**

- 本模块依赖于 `FreeRTOS` 提供的内核支持以及C标准库。如果直接使用本模块，请手动将`FreeRTOS` 与C标准库添加至编译环境中，本模块并不包含该环境。

- 本模块仅在ENTERPRIZE战队的嵌入式开发环境中成功运行，**无法保证**在其他嵌入式环境下的兼容性，开源仅供学习参考而不直接提供现成部署方案。

  

