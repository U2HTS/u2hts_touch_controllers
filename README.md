# U2HTS touch controllers
| Vendor / 制造商 | Series / 系列 | Auto configuration / 自动配置 | Test / 测试 | Controller name /  控制器名 | 总线 / Buses | Custom controller config / 自定义控制器配置 | 
| --- | --- | --- | --- | --- | --- | --- |
| Goodix / 汇顶 | `GT9xx` | Y | GT5688 | `gt9xx` | I2C | |
| Synaptics / 新思 | `RMI4-F11-I2C` | Y | S7300B | `rmi_f11` | I2C/SPI | |
| Focaltech / 敦泰 | `FT54x6` | N | ft3168, ft5406 |  `ft54x6` | I2C | |
| Hynitron / 海栎创 | `CST8xx` | N | cst816d | `cst8xx` | I2C | |
| Chipsemi / 基合半导体 | `CHSC6540` | N | chsc6540 | `chsc6540` | I2C | |
| dummy / 虚拟 | Report rate test / 回报率测试 | Y | N/A | `dummy` | N/A | dummy.rand_seed / 随机数种子 |

**Note**: 
- Currently only I2C bus supports controller detection.
- If `Controller name` is `auto`, then every available slave address on I2C bus will be detected, match the **first probed I2C slave** with an **integrated controller driver** and initialise it. Be advised that **different controllers may share same I2C address** or **different drivers may register same I2C address**. If an incorrect controller match occurs, please specify `Controller name` explicitly.  
- Controller parameters(resolution and max touch points) must be hardcoded into firmware if controller does NOT support auto configuration.  
- For some controllers (especially the Goodix GT9XX series), if touchpoint flickers, adjust `fetch_delay` value appropriately (approx. `16~20`).

**注意**: 
- 只有在使用I2C总线时支持检测控制器功能。
- 如果将`控制器名`配置为`auto`，则固件会检测i2c总线上的所有i2c从机地址，并将**第一个检测到的**i2c从机与**已集成**的控制器驱动进行匹配并进行初始化。鉴于**不同的控制器可能拥有同一个i2c地址**或**不同的驱动可能注册了同一个i2c地址**，如果控制器匹配有误，请手动配置`控制器名`。    
- 如果你的控制器**不支持**`自动配置`，则你需要将触摸屏相关的参数（如分辨率和触摸点数）硬编码进固件中。
- 如果部分控制器(特别是汇顶系列)出现触摸点闪烁的情况，请适当设置`fetch_delay`值（`16~20`即可）。
