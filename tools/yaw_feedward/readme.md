# ʹ�÷���

### �༭`CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/yaw_feedward/yaw_feedward.cpp # <- �����һ��
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- �����һ��
)
```

### ��ʼ��
```cpp
sp::YawFeedward yaw_feedward(dt,inertia,k_ff);
```
`dt`: ѭ�����ڣ���λΪs

`inertia`: ת������

`k_ff`: ǰ������,Ĭ��ֵΪ1.0�������ϵͳ��ʵ�ʶ�̬��Ӧ���𲽵���

### ����

```cpp
yaw_feedward.calc(yaw_pos_pid.out); 
```

### ��ѯ���

```cpp
give_torque = yaw_feedward.out + yaw_speed_pid.out;
```

# ԭ��
����ĽǼ��ٶ� �� ��Ť�� T ֮��Ĺ�ϵ����ת����ѧ����������
$$T = J \cdot \alpha$$
���У�
- T����������Ť�أ���λ�� $N��m$ ��
- J�������ת����������λ�� $kg \cdot m^2$ ��
- �����Ǽ��ٶȣ���λ�� $rad/s^2$ ��

ǰ������ͨ������Ŀ����ٶ� $\omega_{set}$ �ͽǼ��ٶ� $\alpha_{set}$ ����������Ĳ���Ť�ء�

���ǽ�λ�û���Ϊ�ٶȹ滮��

����ɢϵͳ��,λ�û�����Ĳ�ּ�ΪĿ����ٶ�

$$\alpha_{set}[k] = \frac{\omega_{set}[k] - \omega_{set}[k-1]}{\Delta t}$$
