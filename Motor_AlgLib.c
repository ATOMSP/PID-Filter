/* please use GB2312 to open the file */
#include "Motor_AlgLib.h"
#include <string.h>




/**
 * @brief 计算位置式PID 
 * @param hpid  pid句柄
 * @param actual 输入测量值
 * @param target 输入目标值
 * @return  PID输出（+-）
*/
float PidRunPos(PID_Typedef * hpid,float actual,float target)
{
  /* stop pos PID */
  if(hpid->IS_STOP){ 
    return 0.0f;
  };
  /* 计算误差与积分和 */
  hpid->error = target - actual;
  hpid->Integral_sum += hpid->ki * ((hpid->error + hpid->last_error) * 0.5f);
  /* 积分限幅 */
  hpid->Integral_sum = (hpid->Integral_sum > hpid->INTEGRAL_MAX) ? 
                       hpid->INTEGRAL_MAX : 
                       ((hpid->Integral_sum < -(hpid->INTEGRAL_MAX)) ? -(hpid->INTEGRAL_MAX) : hpid->Integral_sum);
  /* PID计算输出值 */
  float output;
  output = hpid->kp * hpid->error + hpid->Integral_sum + hpid->kd * (hpid->error - hpid->last_error);
  /* 输出限幅 */
  output = (output > hpid->OUTPUT_MAX) ? 
                 hpid->OUTPUT_MAX : 
                 ((output < -(hpid->OUTPUT_MAX)) ? -(hpid->OUTPUT_MAX) : output);
  /* 传递上一次的误差 */
  hpid->last_error = hpid->error;
  return output;
}


/**
 * @brief 计算增量式PID 
 * @param hpid  pid句柄
 * @param actual 输入测量值
 * @param target 输入目标值
 * @return  PID输出（+-）
*/
float PidRunInc(PID_Typedef *hpid,float actual,float target)
{
  volatile static float out = 0.0f;
  /* 暂停PID */
  if(hpid->IS_STOP) return;
  hpid->I_error = target - actual;
  out += (hpid->kp * (hpid->I_error - hpid->I_last_error)
        + hpid->ki * hpid->I_error
        + hpid->kd * (hpid->I_error - (2.0f * hpid->I_last_error) + hpid->I_llast_error));
  /* 输出限幅 */
  out = (out > hpid->OUTPUT_MAX) ? 
                 hpid->OUTPUT_MAX : 
                 ((out < -(hpid->OUTPUT_MAX)) ? -(hpid->OUTPUT_MAX) : out);
  hpid->I_llast_error = hpid->I_last_error;
  hpid->I_last_error = hpid->I_error;
  return out;
}


/**
 * @brief 前馈控制
 * @param input 信号输入
 * @param T 采样间隔ms
 * @return 前馈输出
*/
float ForwardCTL(float input,float T)
{
  volatile static float last_in;
  float out = (input - last_in) / (T * 0.001f) + input;
  last_in = input;
  return out;
}

/**
 * @brief 带前馈控制位置式PID 
 * @param hpid  pid句柄
 * @param actual 输入测量值
 * @param target 输入目标值
 * @return  PID输出（+-）
*/
float PidRunPos_F(PID_Typedef * hpid,float actual,float target)
{
  float out = ForwardCTL(actual,hpid->SampleTime);
  out += PidRunPos(hpid,actual,target);
  return out;
}


/**
 * @brief 设置PID参数
 * @param hpid  pid句柄
 * @param kp ki kd (> 0)
 * @param none
*/
void SetPidParam(PID_Typedef * hpid,float kp,float ki,float kd)
{
  if(!hpid->SampleTime) return;
  hpid->kp = kp;
  hpid->ki = ki * (hpid->SampleTime * 0.001f);
  hpid->kd = kd / (hpid->SampleTime * 0.001f);
  /* 设置输入输出关系 */
  if(hpid->IS_ION){
    hpid->kp = (0.0f - hpid->kp);
    hpid->ki = (0.0f - hpid->ki);
    hpid->kd = (0.0f - hpid->kd);
  }
}

/**
 * @brief 设置PID采样间隔
 * @param hpid  pid句柄
 * @param tim (+ ms)
 * @param none
*/
void SetPidSampleTim(PID_Typedef * hpid,float tim)
{
  if(tim <= 0) return;
  if(!hpid->SampleTime) return;
  float ratio  = tim / hpid->SampleTime;
  /* 更新ki与kd */
  hpid->ki *= ratio;
  hpid->kd /= ratio;
  hpid->SampleTime = tim;  
}


/**
 * @brief 设置PID积分与输出限幅
 * @param hpid  pid句柄
 * @param integral_max 积分最大值
 * @param output_max 输出最大值
 * @param none 
*/
void SetPidLimitVal(PID_Typedef * hpid,float integral_max,float output_max)
{
  if(integral_max < 0 || output_max < 0) return;
  hpid->INTEGRAL_MAX = integral_max;
  hpid->OUTPUT_MAX = output_max;
}

/**
 * @brief PID对象初始化
 * @param hpid  pid句柄
 * @param kp ki kd
 * @param tim 采样间隔 I_max 积分最大值 O_max 输出最大值 
 * @param none 
*/
void PID_Init(PID_Typedef * hpid,float kp,float ki,float kd,
              float tim,uint8_t Ision,float I_max,float O_max){
  memset((void*)hpid,0,sizeof(PID_Typedef));
  hpid->IS_STOP = 0;
  hpid->IS_ION = Ision;
  hpid->SampleTime = tim;
  hpid->INTEGRAL_MAX = I_max;
  hpid->OUTPUT_MAX = O_max;
  SetPidParam(hpid,kp,ki,kd);
}

/**
 * @brief 设置PID是否停止
 * @param hpid  pid句柄
 * @param en (0 / 1)
 * @param none 
*/
void SetPidIsStop(PID_Typedef * hpid,uint8_t en)
{
  hpid->IS_STOP = en;
}

/**
 * @brief 设置PID是否输入输出正/负相关
 * @param hpid  pid句柄
 * @param en (0 / 1)
 * @param none 
*/
void SetPidIsIOn(PID_Typedef * hpid,uint8_t en)
{
  hpid->IS_ION = en;
  if(en){
    SetPidParam(hpid,hpid->kp,hpid->ki,hpid->kd);
  }
}

/**
 * @brief 速度位置串级位置式PID
 * @param E_hpid I_hpid 外环内环pid句柄
 * @param E_acgtual 外环测量值
 * @param I_actual 内环测量值
 * @param target 外环目标值
 * @return output 输出（+-）
*/
float CascadePid(PID_Typedef * E_hpid,PID_Typedef * I_hpid,float E_actual,float I_actual,float target)
{
  float out;
  out = PidRunPos(E_hpid,E_actual,target);
  return PidRunPos(I_hpid,I_actual,out);
}


/*--------------------滤波算法------------------------*/

/**
 * @brief 均值滤波
 * @param CallBack 传感器数据获取回调
 * @param n 采样次数
 * @param ret 均值输出
*/
float Filter_Mean(void(*CallBack)(float*),int n)
{
  float temp[n];
  float ret;
  for(int i = 0 ;i < n; i++){
    CallBack(temp + i);
    ret += temp[i];
  }
#ifdef ARM_MATH_CM4
  ret = 0.0f;
  arm_mean_f32(temp,n,&ret);
  return ret;
#else
  return (float)ret / n;
#endif
}

/**
 * @brief return time dif
 * @param now now time
 * @param last last time
 * @return  time dif
*/
static uint32_t getTimeDif(uint32_t now, uint32_t last){
#define TIME_CNT_MAX ((uint32_t)~((uint32_t)0))
    if ((int)((int)now - (int)last) >= 0) {
        return (now - last);
    }
    else {
        return ((TIME_CNT_MAX - last + 1) + now);
    }
#undef TIME_CNT_MAX
}

/**
 * @brief 低通滤波
 * @param now 输入时刻ms
 * @param Tf 时间常数(>0)s
 * @param input 输入数据
 * @return  滤波输出
*/
float Filter_Low(uint32_t now,float Tf,float input)
{
  volatile static uint32_t last;
  volatile static float last_out;
  float dt = (float)getTimeDif(now,last) * 0.001f;
  if(dt > 0.3f){
    last_out = input;
    last = now;
    return input;
  }
  float alpha = Tf / (Tf + dt);
  float out = alpha * last_out + (1.0f - alpha) * input;
  last_out = out;
  last = now;
  return out;
}


/**
 * @brief 限幅滤波
 * @param Max 数据最大值绝对值
 * @param input 输入数据
 * @return  滤波输出
*/
float Filter_LimitAmp(float Max,float input)
{
#define GET_ABS(num) ((num > 0) ? num : -num)
  volatile static float last;
  if(GET_ABS(input) >= Max) return last;
  else{
    last = input;
    return input;
  }
#undef GET_ABS
}

















