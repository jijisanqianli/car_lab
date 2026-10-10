#pragma once

/* TB6612FNG 电机驱动接口 —— 第2课·靶标B 的审计对象就是本接口三函数的实现 */

// 初始化: LEDC(20kHz/10bit, 与呼吸灯同款参数) + IN1/IN2/STBY 方向脚
// 注意: 本函数结尾必须拉高 STBY(真值表: STBY=0 全输出关断)——靶标B必查项①
void motor_init(void);

// 带符号速度接口: motor 0=左(A路) 1=右(B路); duty ∈ [-1,1], 正=正转 负=反转
// duty=0 语义: IN1=1/IN2=0 且 PWM 恒低 → 真值表"短接制动"(捏轮反向拖动吃力)
//              需要高阻滑行请显式调 motor_coast()
// 必查项: ② set_duty 后 update_duty ③ duty 夹限 [-1,1]
void motor_set(int motor, float duty);

// 双路立即停车: IN1=IN2=0 短接制动 + 占空比归零; STBY 不动(留给上层决定)
void motor_stop_all(void);

// 单路滑行(高阻): 真值表 IN1=IN2=1 → 输出关断, 轮子自由转
// 用途: 急停后释放应力/被推行/需要"无冲击停止"的场合(精确停车用 brake 不用 coast)
void motor_coast(int motor);

// 彻底下电: 先停车, 再拉低 STBY 断总闸 + 停 LEDC 输出(引脚定义只在本驱动一处维护)
void motor_disable(void);
