# ʹ�÷���

����ֱ��ʹ��
```cpp
sp::FuzzyPID motor_pid(dt, kp, ki, kd, max_out, max_iout, error_scale, error_rate_scale,alpha, angular, dynamic);
```
����
```cpp
sp::PID motor_pid(dt,kp,ki,kd,max_out,max_iout,alpha,angular,dynamic);
```

- ������ͬ�����Ĳ���������ȫһ�������÷���Ҳ����ͨpid��ȫһ��������ֱ�ӵõ�һ������������ҶԲ�ͬ��Ӧ�ó����и��õ���Ӧ�ԡ�

- **Warning** ���������PID�Ѿ������𵴱�Ե��������Ҫ�ʵ���С30%���ҵ�PID����ֵ��


`error_scale`����Χ 
- �ԽǶȻ����⻷��Ϊ������Χ�� $ [-\pi,\pi] $ ,��ô`error_scale`�͸ø� $ \pi $��

`error_rate_scale`�����仯�ʷ�Χ
- ��Ҫȡ���ڵ��Ť�غ�ת������������ʵ�⡣

`error_scale`��`error_rate_scale`����Խ׼��ģ��PID��Ч��Ҳ����Ӧ���á�

