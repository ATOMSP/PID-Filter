/* please use GB2312 to open the file */
#include "Motor_AlgLib.h"




int main(){
  /* 定义PID对象 */
  PID_Typedef pid;
  float out;
  /* 初始化PID对象 */
  PID_Init(&pid,0,0,0,10,0,200,200);
  while(1){
  /* 10ms调用位置式PID */
    out = PidRunPos(&pid,GetSensorInfo(),10);
    /* 使用PID输出 */
    Load(out);
    delayms(10);
  }
}