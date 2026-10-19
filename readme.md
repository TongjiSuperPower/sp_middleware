## 简介
同济大学SuperPower战队25赛季电控中间件

命名空间`sp::`

文件结构:
- io: 常用硬件(led, 蜂鸣器, 遥控器, 上位机曲线...)
- motor: 电机相关
- referee: 裁判系统相关
- tools: 常用工具和算法

## 安装
```bash
git submodule add https://github.com/TongjiSuperPower/sp_middleware.git
```

## 上位机曲线 SuperPlotter

`io/plotter` 提供一条只发"被订阅的变量"的单向链路，配套上位机是 SuperScope（AdvantageScope 的改造版）。
和原来的 `Plotter` 互不影响，两者可以同时使用。

```cpp
#include "io/plotter/super_plotter.hpp"

sp::SuperPlotter::record("arm/j0/pos", arm_j0.pos);   // 或简写 sp::record(...)
sp::SuperPlotter::record("arm/mode", static_cast<int>(arm_node.mode_));
```

任意任务、任意位置都能调用，第一次调用时自动注册路径，之后每次只写一个 float：
按 16 位哈希定位槽位，无锁、无内存分配。地址固定的热点可以用句柄形式：

```cpp
sp::PlotPublisher j0 = plotter.publish("arm/j0/pos");  // 注册一次
j0.set(arm_j0.pos);                                    // 之后是一次 float 写入
```

### 约束

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 路径长度 | ≤ 15 字符 | 超长**编译报错**，不会截断（截断会把两个路径并成一个字段） |
| 路径形式 | 字面量 | 运行期 `const char *` 不接受，长度必须编译期已知 |
| 注册上限 | 128 个 | 每槽 24 字节，共约 3 KB；可用 `-DSP_PLOT_MAX_TOPICS=<n>` 改 |
| 单帧载荷 | 192 字节 | 订阅数超过 37 个时分多帧连续发出，不丢数据 |

注册和发送是两个独立的量：注册只占 RAM，**只有上位机勾选的路径才会被发送**。
所以路径可以放心留在代码里，串口带宽只取决于勾了多少条。

### 协议

```
板 -> 上位机  AA BC | id u8 | name, NUL          首次 flush 时为每个主题发一次
上位机 -> 板  AA BD | op u8 | name, NUL          1 订阅 2 取消 3 全订阅 4 清空
板 -> 上位机  AA BB | count u8 | (id u8, f32le)* 每次 flush 发订阅值
```

### 接入

```cpp
sp::SuperPlotter plotter(&huart7, false);   // 在 plot_task 里创建，注册全局实例

extern "C" void plot_task()
{
  plotter.initialize(scope_rows());         // 启动接收 + 注册轮询表（表可省略）
  plotter.loop([] {                         // 1 kHz：采样轮询表 → record → flush
    sp::SuperPlotter::record("chassis/power", power_meter.power);
  });
}
```

`initialize()` 只做初始化，`loop()` 不会返回。想自己控制节奏就照旧手写循环调
`flush()`：`record()` 是静态的，在哪个任务里调都可以。

UART 中断里调用 `on_rx()` 交出收到的字节即可。主机侧工具见 SuperScope 仓库。

## 参考
TODO
