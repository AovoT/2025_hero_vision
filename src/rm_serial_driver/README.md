### 一些拙见
个人认为对于 ROS2 协作开发的项目来说，可读性的提升主要在能直观感受接口上
因此，对于其他开发者关心的 Publishers Subscribers Parameters 我都做了直观的封装，
并直接进行管理。

### SendPacket

```
 地址偏移   |   数据字段  |   大小（字节）    |   备注
-------------------------------------------------------

 0x00      | header    | 1  (0xA5)       | 单独存储
 0x01      | x         | 4               | float，占4字节
 0x05      | y         | 4               |
 0x09      | z         | 4               |
 0x0D      | yaw       | 4               |
 0x11      | vx        | 4               |
 0x15      | vy        | 4               |
 0x19      | vz        | 4               |
 0x1D      | v_yaw     | 4               |
 0x21      | r1        | 4               |
 0x25      | r2        | 4               |
 0x29      | dz        | 4               |

 0x2D      | crc       | 1               | 单独存储
-------------------------------------------------------

**总大小 = 1 + (4 × 11) + 1 = 46 字节**
```

