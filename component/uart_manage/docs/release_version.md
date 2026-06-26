# Release Version

## 版本简述

| 版本 | 作者 | 发布日期 | 简述 |
|---|---|---|---|
| v0.0.1 | river | 2026-03-02 | 完成 `uart_manage` 首个可用版本，实现多串口统一管理、`IDLE + DMA` 不定长接收、DMA 发送续发与基础示例文档。 |

## v0.0.1 详细功能特性

### 1) 多串口统一对象管理

- 支持通过 `uart_manage_table` 进行静态注册。
- 支持按句柄与按名称查找对象（`uart_manage_get_obj*`）。
- 为上层业务提供统一串口抽象，避免直接耦合 `huartX`。

### 2) 不定长接收主链路（UART IDLE + DMA）

- 使用 `HAL_UARTEx_ReceiveToIdle_DMA` 接收不定长数据。
- 在 `HAL_UARTEx_RxEventCallback` 中按 `size` 处理有效数据块。
- 回调后自动重启 DMA 接收，保障持续接收能力。

### 3) 两种接收处理模式

- **Ring-Task 模式**：数据写入 `process_ring_buffer`，由线程侧读取处理。
- **Callback-Direct 模式**：收到数据后直接调用 `recv_callback`。
- 两种模式可按业务实时性与可维护性需求灵活选择。

### 4) DMA 发送与自动续发

- 发送链路采用“首包直发 + FIFO 缓冲续发”机制。
- `HAL_UART_TxCpltCallback` 触发后自动从 FIFO 取下一段继续发送。
- 可应对短时突发发送，降低上层处理复杂度。

### 5) 异常恢复与健壮性基础

- 接收异常路径下自动重挂 DMA 接收。
- Busy 状态下包含重试与重启逻辑，降低链路卡死概率。

### 6) STM32H7 DCache 一致性处理

- 发送前执行 cache clean。
- 接收后执行 cache invalidate。
- 降低 H7 平台 DMA 场景下的数据一致性风险。

### 7) 示例与文档

- 提供 Echo 示例（线程环形缓冲/直接回调两种思路）。
- 提供 quickstart 与设计说明文档，便于快速集成。

### 备注

该版本定位为首个可集成版本，后续将继续完善统计观测、并发健壮性和参数配置化能力。
