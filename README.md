# U2HTS touch controllers
[zh_CN(简体中文)](./README_zh.md)

| Vendor | Series | Read configuration | Test | Controller name | Buses | Custom controller config |
| --- | --- | --- | --- | --- | --- | --- |
| Goodix | `GT9xx` | Y | GT5688/GT615/GT9271 | `gt9xx` | I2C | |
| Synaptics | `RMI4-F11-I2C` | Y | S7300B | `rmi_f11` | I2C/SPI | |
| Focaltech | `FT54x6` | N | ft3168, ft5406 | `ft54x6` | I2C | |
| Hynitron | `CST8xx` | N | cst816d | `cst8xx` | I2C | |
| Chipsemi | `CHSC6540` | N | chsc6540 | `chsc6540` | I2C | |
| Goodix | `Normandy` | N | GT9886 | `goodix_normandy` | I2C | |
| Jadard | `Jadard TDDI` | Y | JD9365TX | `jd9365` | I2C | |
| Microchip | `maxTouch` | Y | mxt1188s1 | `max1188s1` | I2C | |
| Linux | mt input device | Y | N/A | `linux-multitouch` | N/A | linux-mt.dev |
| dummy | Report rate test | Y | N/A | `dummy` | N/A | dummy.rand_seed |

**Note**:
- Currently only I2C bus supports controller detection.
- If `Controller name` is `auto`, then every available slave address on I2C bus will be detected, match the **first probed I2C slave** with an **integrated controller driver** and initialise it. Be advised that **different controllers may share same I2C address** or **different drivers may register same I2C address**. If an incorrect controller match occurs, please specify `Controller name` explicitly.
- Controller parameters (resolution and max touch points) must be hardcoded into firmware if controller does NOT support read configuration.
- For some controllers (especially the Goodix series), if touchpoint flickers, adjust `fetch_delay` value appropriately (approx. `16~20`).
