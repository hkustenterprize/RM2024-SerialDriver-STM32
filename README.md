# 文件结构说明

## 文件树



```markdown
├── asset 								# 文档素材文件
│   ├── Circular_Buffer_Animation.gif
│   ├── config_1.png
│   ├── config_2.png
│   ├── config_3.png
│   ├── experiment_host.png
│   ├── experiment_MCU.png
│   ├── experiment_systemview.png
│   ├── extra_description_2.png
│   ├── extra_description.png
│   ├── RXDMA_Config.png
│   ├── RX_unpack_diagram.drawio
│   ├── RX_unpack_diagram.drawio.png
│   ├── sch_ttl_ch343p.png
│   ├── UART_diagram.excalidraw
│   └── UART_diagram.png
├── Doc									# 开源文档	
│   ├── serial_driver_MCU_doc.md
│   └── serial_driver_MCU_doc.pdf
├── NewRosComm							# 开源文件本体
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



本模块的代码开发基于队伍的 `arm-none-eabi-gcc`工具链，采用 `STM32CUBEMX` 生成项目配置文件。如果要直接使用本模块，用户需要自己下载相关环境。

文件的引用路径已经包含在`NewRosComm.mk`，使用时需要在主`Makefile`文件中手动添加模块的路径，定义该模块的根目录`NewRosComm_PATH`，并手动将引用文件 `NewRosComm_INC`与源文件`NewRosComm_PATH_SRC` 路径添加进编译环境里面

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

```makefile
# NewRosComm.mk

# Sources
NewRosComm_SRC = \
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

  

