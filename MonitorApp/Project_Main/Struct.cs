using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Project_Main
{

    struct PACKET
    {
        public UInt16 STX;
        public UInt16 len;
        public UInt16 msgid;      
        public UInt16 end;
    };

    //모터 상태정보 payload
    struct RECVMOTOR
    {
        public UInt32 ang;
        public UInt32 RPM;
        public UInt32 setpoint;
        public UInt32 pwm;
        public UInt32 pre_rpm;
        public UInt32 x;
        public UInt32 y;
    };

    // RC 입력 담은 payload
    struct RECVRC
    {
        public UInt32 throttle;
        public UInt32 roll;
        public UInt32 pitch;
        public UInt32 yaw;
        public UInt32 aux1;
        public UInt32 aux2;
        public UInt32 aux3;
    };

    //오류 메시지 정의 payload
    struct RECVLOG
    {
        public UInt16 logMsg;
    }
}
