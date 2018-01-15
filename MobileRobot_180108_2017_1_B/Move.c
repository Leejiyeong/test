#include "Move.h"
#include "Interface.h"
#include <math.h>

#define READ_STATUS(X, Y)   ((Y) = MOTOR_CMD(X))
#define	CHECK_BUSY(X, Y)	while(((Y) = MOTOR_CMD(X)) & 0x01)

unsigned char status1, status2, status3;


// Function  : 모든 모터의 모션을 시작한다.
// Parameter : 없음
// Return    : 없음
void StartMotion(void)

{
    WriteCommand(MOTOR0, STT);
    WriteCommand(MOTOR1, STT);
    WriteCommand(MOTOR2, STT);
}


// Function  : 모든 모터의 모션을 정지한다.
// Parameter : 없음
// Return    : 없음
void StopMotion(uint16_t mode)
{
    // Motor1 정지
    // LTRJ 커맨드 출력
    WriteCommand(MOTOR0, LTRJ);
    // 컨트롤 워드 출력
    WriteDataTwoByte(MOTOR0, (1 << mode));

    // Motor2 정지
    // LTRJ 커맨드 출력
    WriteCommand(MOTOR1, LTRJ);
    // 컨트롤 워드 출력
    WriteDataTwoByte(MOTOR1, (1 << mode));

    // Motor3 정지
    // LTRJ 커맨드 출력
    WriteCommand(MOTOR2, LTRJ);
    // 컨트롤 워드 출력
    WriteDataTwoByte(MOTOR2, (1 << mode));

    // 모션 정지
    WriteCommand(MOTOR0, STT);
    WriteCommand(MOTOR1, STT);
    WriteCommand(MOTOR2, STT);
}

void MOTOR_CTR(long MT0,long MT1,long MT2){
	
	unsigned char i=0;
	double V[3]={MT0,MT1,MT2};

	for(i=0;i<3;++i){
		
		V[i]=V[i]*0.2728;

		SetVelocity(i, V[i]*65536);
	}
	StartMotion();	
}
