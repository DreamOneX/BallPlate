# Ball-Plate Remote Control Protocol v1

适用于 nRF24L01 无线链路的命令协议。接收端为纯被动角色，不发送任何响应。

---

## 1. 数据包格式

≤ 32 字节，适配 nRF24L01 单包 payload。

```
Byte:  0        1        2        3        4       5 ... N    N+1
     ┌────────┬────────┬────────┬────────┬────────┬──────────┬────────┐
     │ HEADER │ VER    │ CMD_ID │ SEQ    │ LEN    │ PAYLOAD  │ CRC8   │
     │ 0xA5   │ uint8  │ uint8  │ uint8  │ uint8  │ 0~26 B   │ uint8  │
     └────────┴────────┴────────┴────────┴────────┴──────────┴────────┘
```

| 字段 | 偏移 | 大小 | 说明 |
|------|------|------|------|
| HEADER | 0 | 1 | 固定 `0xA5`，帧同步标识 |
| VER | 1 | 1 | 协议版本号，当前 `0x01`。接收端拒绝不兼容版本 |
| CMD_ID | 2 | 1 | 命令类型枚举 |
| SEQ | 3 | 1 | 序列号 0–255 循环，用于去重 / 丢包检测 |
| LEN | 4 | 1 | 载荷字节数（0–26），不含头部和 CRC |
| PAYLOAD | 5 | 0–26 | 命令特定数据，小端序 |
| CRC8 | 5+LEN | 1 | CRC-8（多项式 0x07，初始值 0x00），覆盖 byte 0 至 byte 4+LEN |

总长度 = 5 + LEN + 1，最大 32 字节 → LEN ≤ 26。

---

## 2. CRC-8

- 多项式：`0x07`
- 初始值：`0x00`
- 计算范围：HEADER 至 PAYLOAD 末尾（不含 CRC 字节本身）
- 选择 CRC-8 是因为 nRF24L01 Enhanced ShockBurst 已有硬件 CRC-16，应用层 CRC-8 仅作额外协议完整性校验

---

## 3. 命令定义

| CMD_ID | 名称 | 载荷长度 | 说明 |
|--------|------|----------|------|
| 0x01 | REMOTE_STOP | 1 | 远程停机（Freeze / Park） |
| 0x02 | REMOTE_RESUME | 0 | 远程恢复（可由接收端配置禁用） |
| 0x03 | TARGET_UPDATE | 8 | 更新目标位置 |
| 0x10 | *HEARTBEAT* | *保留* | *预留* |
| 0x11 | *PID_TUNE* | *保留* | *预留* |

### 3.1 CMD 0x01 — REMOTE_STOP

```
PAYLOAD (1 byte):
  Byte 0: mode  (uint8)
          0x01 = Freeze  — 停止 PID，servo 保持当前角度
          0x02 = Park    — 停止 PID，servo 回到中位 (90°)
```

**时延边界：** 这是控制层软停机，不是硬件级 E-stop。
最坏生效延迟 = main loop 周期 + 1/控制频率 + I2C 写入时间。
**此机制不能替代物理急停开关。**

**可靠性建议：** 发送端应对 STOP 命令连续重发 3–5 次，或启用 nRF24L01 硬件 ACK + 自动重传。

### 3.2 CMD 0x02 — REMOTE_RESUME

```
PAYLOAD (0 bytes):
  无载荷。接收端 reset PID 后恢复控制。
```

接收端可通过编译期配置禁用此命令（静默丢弃），此时只能通过硬件复位恢复。

### 3.3 CMD 0x03 — TARGET_UPDATE

```
PAYLOAD (8 bytes):
  Byte 0-3: target_x  (IEEE-754 single precision, little-endian byte order)
  Byte 4-7: target_y  (IEEE-754 single precision, little-endian byte order)
```

---

## 4. SEQ 去重

接收端维护上一个已接受包的 SEQ 值：

- **首包**无条件接受（避免 SEQ=0 初始值误丢）
- 后续包与上一个 SEQ 相同 → 视为重复，丢弃
- SEQ 不同 → 接受

8-bit SEQ 仅防相邻重复（应对 nRF24L01 auto-retransmit），不防跨回卷碰撞。
当前所有命令均为幂等操作，偶尔重复执行无副作用。
SEQ 的主要价值是丢包监测（gap 检测），而非可靠去重。

---

## 5. 行为语义

### 5.1 TARGET_UPDATE 在停机状态下

停机时 PID 计算被跳过，pending 的 target 不会被消费。新 target 保留在 pending 中，直到 RESUME 后的下一个控制周期才生效。即：**停机期间收到的最后一次 TARGET_UPDATE 会在恢复后立即应用**。

### 5.2 异常包处理

| 异常情况 | 处理 |
|----------|------|
| 未知 CMD_ID | 静默丢弃 |
| 非法 StopMode（不在 {0x01, 0x02}） | 静默丢弃 |
| LEN 与命令不匹配（如 TARGET_UPDATE 但 LEN < 8） | 静默丢弃 |
| HEADER / VER / CRC 校验失败 | 静默丢弃 |

所有异常情况均静默丢弃，不发送任何响应。

---

## 6. 丢包与可靠性

- nRF24L01 接收 FIFO 仅 3 包深，溢出即硬件丢弃
- 协议层不保证可靠交付
- 安全关键命令（REMOTE_STOP）的可靠性由发送端负责（重发或硬件 ACK）
- 普通命令（TARGET_UPDATE）允许偶尔丢包，发送端按需频率持续发送即可
