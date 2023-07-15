/* please use GB2312 to open the file */
#ifndef __MOTOR_ALGLIB_H_
#define __MOTOR_ALGLIB_H_



#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>



/* PID struct */
typedef struct {
  /* pid param*/
  volatile float kp;
  volatile float ki;
  volatile float kd;
  /* Integral max */
  float INTEGRAL_MAX;
  /* out max */
  float OUTPUT_MAX;
  /* stop/start pid运行 */
  volatile uint8_t IS_STOP;
  /* input/output PN */
  volatile uint8_t IS_ION;
  /* Sample time */
  volatile float SampleTime;
  /* private */
  volatile float Integral_sum;
  volatile float last_error;
  volatile float error;
  /* ----------Inc private PID-------------*/
  volatile float I_last_error;
  volatile float I_error;
  volatile float I_llast_error;
}PID_Typedef;

/**
 * @brief PID对象初始化
 * @param hpid  pid句柄
 * @param kp ki kd
 * @param tim 采样间隔 Ision IO正负相关 I_max 积分最大值 O_max 输出最大值 
 * @return none 
*/
void PID_Init(PID_Typedef * hpid,float kp,float ki,float kd,
              float tim,uint8_t Ision,float I_max,float O_max);
/**
 * @brief 计算位置式PID 
 * @param hpid  pid句柄
 * @param actual 输入测量值
 * @param target 输入目标值
 * @return  PID输出（+-）
*/
float PidRunPos(PID_Typedef * hpid,float actual,float target);
/**
 * @brief 计算增量式PID 
 * @param hpid  pid句柄
 * @param actual 输入测量值
 * @param target 输入目标值
 * @return  PID输出（+-）
*/
float PidRunInc(PID_Typedef *hpid,float actual,float target);
/**
 * @brief 带前馈控制位置式PID 
 * @param hpid  pid句柄
 * @param actual 输入测量值
 * @param target 输入目标值
 * @return  PID输出（+-）
*/
float PidRunPos_F(PID_Typedef * hpid,float actual,float target);
/* Set PID Param API */
void SetPidIsIOn(PID_Typedef * hpid,uint8_t en);
void SetPidIsStop(PID_Typedef * hpid,uint8_t en);
void SetPidLimitVal(PID_Typedef * hpid,float integral_max,float output_max);
void SetPidSampleTim(PID_Typedef * hpid,float tim);
void SetPidParam(PID_Typedef * hpid,float kp,float ki,float kd);
/* Common Filter Lib */
/**
 * @brief 低通滤波
 * @param now 输入时刻ms
 * @param Tf 时间常数(>0)s
 * @param input 输入数据
 * @return  滤波输出
*/
float Filter_Low(uint32_t now,float Tf,float input);
/**
 * @brief 均值滤波
 * @param CallBack 传感器数据获取回调
 * @param n 采样次数
 * @return ret 均值输出
*/
float Filter_Mean(void(*CallBack)(float*),int n);
/**
 * @brief 限幅滤波
 * @param Max 数据最大值绝对值
 * @param input 输入数据
 * @return  滤波输出
*/
float Filter_LimitAmp(float Max,float input);


#ifdef __cplusplus
}
#endif




#endif


