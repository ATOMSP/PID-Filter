/* please use GB2312 to open the file */
#include "Motor_AlgLib.h"




int main(){
  /* ����PID���� */
  PID_Typedef pid;
  float out;
  /* ��ʼ��PID���� */
  PID_Init(&pid,0,0,0,10,0,200,200);
  while(1){
  /* 10ms����λ��ʽPID */
    out = PidRunPos(&pid,GetSensorInfo(),10);
    /* ʹ��PID��� */
    Load(out);
    delayms(10);
  }
}