# 使用方法

可以直接使用
```cpp
sp::FuzzyPID motor_pid(dt, kp, ki, kd, max_out, max_iout, error_scale, error_rate_scale,alpha, angular, dynamic);
```
代替
```cpp
sp::PID motor_pid(dt,kp,ki,kd,max_out,max_iout,alpha,angular,dynamic);
```

- 其中相同命名的参数可以完全一样，调用方法也和普通pid完全一样，可以直接得到一点点提升，并且对不同的应用场景有更好的适应性。

- **Warning** ：假如你的PID已经调到震荡边缘，可能需要适当减小30%左右的PID参数值。


`error_scale`：误差范围 
- 以角度环（外环）为例，误差范围是 $ [-\pi,\pi] $ ,那么`error_scale`就该给 $ \pi $。

`error_rate_scale`：误差变化率范围
- 主要取决于电机扭矩和转动惯量，建议实测。

`error_scale`和`error_rate_scale`给得越准，模糊PID的效果也会相应更好。

