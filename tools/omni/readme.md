# omni - 四全向轮底盘解算

四全向轮(X 型布局)底盘的正解算/反解算, 接口与 `tools/mecanum` 保持一致。

## 布局与坐标系

四个全向轮位于底盘四个角, **每个轮子的滚动方向垂直于该角到底盘中心的连线(对角线)**。

俯视图 (底盘坐标系: x 向前, y 向左, z 向上, 故逆时针为正):

```
                  +y (左)
                    ↑
       LR  ╱        |        ╲  LF
                    |
   ─────────────────+──────────────────→ +x (前)
                    |
       RR  ╲        |        ╱  RF
```

图中 `╱ ╲` 为轮子滚动方向, 均垂直于所在角的对角线 (对角线与滚动方向在图中互余)。

各轮位置 p 与滚动方向 u (令半对角线长度 `D = sqrt(half_length^2 + half_width^2)`):

| 轮子 | 位置 p | 滚动方向 u |
| --- | --- | --- |
| LF | (+l, +w) | (-w, +l) / D |
| LR | (-l, +w) | (-w, -l) / D |
| RF | (+l, -w) | (+w, +l) / D |
| RR | (-l, -w) | (+w, -l) / D |

正方形底盘 (`l == w`) 时滚动方向与坐标轴成 45°, 即常见的 X 型全向轮布局。

### 几何性质 (已数值校验)

- 对角两轮的滚动方向**相反**(`LF ∥ RR`, `LR ∥ RF`), 相邻两轮**正交**, 因此四个方向只占两条直线;
- 完整约束矩阵(4x3, 含 `wz` 列)的秩恒为 3, 即 3 个自由度全部可控, 底盘是全向的;
  四个约束中只有一条线性相关关系 `u_lf - u_lr - u_rf + u_rr = 0`;
- 布局的秩对长宽比不敏感(`l != w` 时秩同样为 3), 反解对任意长宽比都成立; 本底盘按正方形设计, 取 `l == w`。

## 正解 (底盘速度 -> 轮速)

全向轮只能沿自身滚动方向 u 滚动(垂直于 u 的方向由辊子自由滑动), 轮心速度为
`v_轮心 = v + wz × p = (vx - wz * y, vy + wz * x)`, 于是

```
speed_i = u_i · v_轮心 / r
        = (-y_i * vx + x_i * vy + (x_i^2 + y_i^2) * wz) / (r * D)
```

四个轮子 `D` 相同, 代入各轮位置后:

```
speed_lf = sign_lf * ( -w * vx + l * vy + D^2 * wz ) / (r * D)
speed_lr = sign_lr * ( -w * vx - l * vy + D^2 * wz ) / (r * D)
speed_rf = sign_rf * ( +w * vx + l * vy + D^2 * wz ) / (r * D)
speed_rr = sign_rr * ( +w * vx - l * vy + D^2 * wz ) / (r * D)
```

正方形底盘 (`l == w == a`, `D == a * sqrt(2)`) 时化简为

```
speed_lf = sign_lf * ( -vx + vy + 2 * a * wz ) / (r * sqrt(2))
speed_lr = sign_lr * ( -vx - vy + 2 * a * wz ) / (r * sqrt(2))
speed_rf = sign_rf * ( +vx + vy + 2 * a * wz ) / (r * sqrt(2))
speed_rr = sign_rr * ( +vx - vy + 2 * a * wz ) / (r * sqrt(2))
```

即 X 型布局下 (取 `l + w = 2a`):

```
speed_lf = ( -vx + vy + (l + w) * wz ) / (r * sqrt(2))
speed_lr = ( -vx - vy + (l + w) * wz ) / (r * sqrt(2))
speed_rf = ( +vx + vy + (l + w) * wz ) / (r * sqrt(2))
speed_rr = ( +vx - vy + (l + w) * wz ) / (r * sqrt(2))
```

## 反解 (轮速 -> 底盘速度)

把电机转速换算成轮子滚动方向上的线速度 `v_i = speed_i / sign_i * r`, 由正解得

```
v_i = (-y_i / D) * vx + (x_i / D) * vy + D * wz
```

写成 `v = H * [vx, vy, wz]^T` 时, `H^T * H` 恰为对角阵
`diag(4 * w^2 / D^2, 4 * l^2 / D^2, 4 * D^2)`, 因此最小二乘解就是精确解:

```
vx = D * ( -v_lf - v_lr + v_rf + v_rr ) / (4 * w)
vy = D * ( +v_lf - v_lr + v_rf - v_rr ) / (4 * l)
wz =     ( +v_lf + v_lr + v_rf + v_rr ) / (4 * D)
```

## reverse_xx 的含义与标定

`reverse_xx = false` 表示"该电机正转时, 轮子沿逆时针切线方向滚动"这一约定成立
(即上表中 u 的方向, 四个轮子的 u 都指向绕底盘中心的逆时针切线方向)。

默认全部为 `false` 时各基础运动的轮子转向:

| 底盘运动 | LF | LR | RF | RR |
| --- | --- | --- | --- | --- |
| `vx > 0` (前进) | 反转 | 反转 | 正转 | 正转 |
| `vy > 0` (左移) | 正转 | 反转 | 正转 | 反转 |
| `wz > 0` (逆时针) | 正转 | 正转 | 正转 | 正转 |

即可化简为: 纯 `vx > 0` 时 `LF = LR = -vx / (r * sqrt(2))`、`RF = RR = +vx / (r * sqrt(2))`,
纯 `vy > 0` 时 `LF = RF = +vy / (r * sqrt(2))`、`LR = RR = -vy / (r * sqrt(2))`。
注意对角两轮滚动方向相反, 所以纯平移时车身左右两侧的轮速是反号的, 这与 mecanum 不同。

标定方法:

1. 底盘悬空, 只给 `wz > 0` (默认参数下四个 `speed` 同为正), 观察四个轮子: 它们都应
   朝同一个方向滚动(绕底盘中心的逆时针切线方向)。若某个轮子与其它三个相反, 就把该轮的
   `reverse_xx` 置为 `true`。
2. 底盘落地, 分别给 `vx > 0`、`vy > 0`、`wz > 0`, 应分别前进、左移、逆时针旋转。
   若三者**全部**反向(后退、右移、顺时针), 说明四个电机的装配方向与默认约定整体相反,
   把四个 `reverse` 全部取反即可。

## 用法

```cpp
#include "tools/omni/omni.hpp"

// 注意: sp_middleware/tools/omni/omni.cpp 需加入 CMakeLists.txt 的 target_sources
sp::Omni chassis(wheel_radius, chassis_half_length, chassis_half_width);

// 底盘速度 -> 轮速 (rad/s)
chassis.calc(vx, vy, wz);
move_data.set_drive_speed[0] = chassis.speed_rf;
move_data.set_drive_speed[1] = chassis.speed_lf;
move_data.set_drive_speed[2] = chassis.speed_lr;
move_data.set_drive_speed[3] = chassis.speed_rr;

// 轮速 -> 底盘速度 (m/s, rad/s)
chassis.update(rf_3508.speed, lf_3508.speed, lr_3508.speed, rr_3508.speed);
// chassis.vx, chassis.vy, chassis.wz
```

## 附: 若长宽不等而轮子仍按 45° 装配

本解算严格按"滚动方向垂直于对角线"推导。若底盘 `l != w`, 但轮子仍与坐标轴成 45°
(此时轮子并不垂直于矩形对角线, 滚动方向取 `(+/-1, +/-1) / sqrt(2)`), 则系数不同:

```
speed_lf = sign_lf * ( -vx + vy + (l + w) * wz ) / (r * sqrt(2))
speed_lr = sign_lr * ( -vx - vy + (l + w) * wz ) / (r * sqrt(2))
speed_rf = sign_rf * ( +vx + vy + (l + w) * wz ) / (r * sqrt(2))
speed_rr = sign_rr * ( +vx - vy + (l + w) * wz ) / (r * sqrt(2))
```

```
vx =     ( -v_lf - v_lr + v_rf + v_rr ) / (2 * sqrt(2))
vy =     ( +v_lf - v_lr + v_rf - v_rr ) / (2 * sqrt(2))
wz =     ( +v_lf + v_lr + v_rf + v_rr ) / (2 * sqrt(2) * (l + w))
```

即 `calc()` 中的项 `(-w * vx + l * vy) / D` 需换成 `(-vx + vy) / sqrt(2)`, 项 `D * wz`
需换成 `(l + w) * wz / sqrt(2)`; `update()` 中的系数 `D / (4 * w)`、`D / (4 * l)`、
`1 / (4 * D)` 需分别换成 `1 / (2 * sqrt(2))`、`1 / (2 * sqrt(2))`、
`1 / (2 * sqrt(2) * (l + w))`。仅在 `l == w` 时两套系数才完全相等
(`D / (4 * w) = D / (4 * l) = 1 / (2 * sqrt(2))`), `l != w` 时两套系数不同,
45° 装配必须换成上面这套。
45° 布局时相邻两轮的滚动方向互相垂直, 而"垂直于对角线"的一般布局只在正方形时才呈 45°。
