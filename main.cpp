
#define CAN_ID 0x0

#include "mbed.h"
#include "math_ops.h"


Serial       pc(PA_2, PA_3);
CAN          can1(PB_8, PB_9);  // CAN Rx pin name, CAN Tx pin name
CAN          can2(PB_5, PB_13);  // CAN Rx pin name, CAN Tx pin name
CANMessage   rxMsg1, rxMsg2;
CANMessage   abad1, abad2, hip1, hip2, knee1, knee2;    //TX Messages
int                     ledState;
Ticker                  sendCAN;
int                     counter = 0;
volatile bool           msgAvailable = false;
Ticker loop;

///[[abad1,  abad2]
///[hip1,   hip2]
///[knee1, knee2]]
float q[3][2];   //Joint states for both legs
float dq[3][2];
float tau[3][3];
float kp = 60;
float kd = 0.8;
int enabled = 0;

/// Value Limits ///
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -30.0f
 #define V_MAX 30.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 #define I_MAX 40.0f
 
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void pack_cmd(CANMessage * msg, float p_des, float v_des, float kp, float kd, float t_ff){
     /// limit data to be within bounds ///
     p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);                    
     v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
     kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
     kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
     t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
     /// convert floats to unsigned ints ///
     int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
     int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
     int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
     /// pack ints into the can buffer ///
     msg->data[0] = p_int>>8;                                       
     msg->data[1] = p_int&0xFF;
     msg->data[2] = v_int>>4;
     msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     msg->data[4] = kp_int&0xFF;
     msg->data[5] = kd_int>>4;
     msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     msg->data[7] = t_int&0xff;
     }
     
/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]

void unpack_reply(CANMessage msg, int leg_num){
    /// unpack ints from can buffer ///
    int id = msg.data[0];
    int p_int = (msg.data[1]<<8)|msg.data[2];
    int v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    /// convert ints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    
    q[id-1][leg_num] = p;
    dq[id-1][leg_num] = v;
    /*
    if(id == 2){
        theta1 = p;
        dtheta1 = v;
        }
    else if(id ==3){
        theta2 = p;
        dtheta2 = v;
        }
        */
    
    //printf("%d  %.3f   %.3f   %.3f\n\r", id, p, v, i);
    } 
    
 void rxISR1() {
    can1.read(rxMsg1);                    // read message into Rx message storage
    unpack_reply(rxMsg1, 0);
}
void rxISR2(){
    can2.read(rxMsg2);
    unpack_reply(rxMsg2, 1);
    }

void WriteAll(){
    can1.write(abad1);
    wait(.0001);
    can2.write(abad2);
    wait(.0001);
    can1.write(hip1);
    wait(.0001);
    can2.write(hip2);
    wait(.0001);
    can1.write(knee1);
    wait(.0001);
    can2.write(knee2);
    wait(.0001);
    }

void sendCMD(){
    /// bilateral teleoperation demo ///
    counter ++;
    
    if(enabled){
        if(counter>100){
            //tcmd = -1*tcmd;
            //printf("%.4f   %.4f   %.4f     %.4f   %.4f   %.4f\n\r", q[0][0], q[1][0], q[2][0], q[0][1], q[1][1], q[2][1]);
            counter = 0 ;
            }
            
        tau[0][1] = kp*(q[0][0] - q[0][1]) + kd*(dq[0][0] - dq[0][1]);
        tau[0][0] = kp*(q[0][1] - q[0][0]) + kd*(dq[0][1] - dq[0][0]);
        tau[1][1] = kp*(q[1][0] - q[1][1]) + kd*(dq[1][0] - dq[1][1]);
        tau[1][0] = kp*(q[1][1] - q[1][0]) + kd*(dq[1][1] - dq[1][0]);
        tau[2][1] = (kp/1.5f)*(q[2][0] - q[2][1]) + (kd/2.25f)*(dq[2][0] - dq[2][1]);
        tau[2][0] = (kp/1.5f)*(q[2][1] - q[2][0]) + (kd/2.25f)*(dq[2][1] - dq[2][0]);
        
        pack_cmd(&abad1, 0, 0, 0, .01, tau[0][0]); 
        pack_cmd(&abad2, 0, 0, 0, .01, tau[0][1]); 
        pack_cmd(&hip1, 0, 0, 0, .01, tau[1][0]); 
        pack_cmd(&hip2, 0, 0, 0, .01, tau[1][1]); 
        pack_cmd(&knee1, 0, 0, 0, .006, tau[2][0]); 
        pack_cmd(&knee2, 0, 0, 0, .006, tau[2][1]); 
        /*
        pack_cmd(&abad1, q[0][1], dq[0][1], kp, kd, 0); 
        pack_cmd(&abad2, q[0][0], dq[0][0], kp, kd, 0); 
        pack_cmd(&hip1, q[1][1], dq[1][1], kp, kd, 0); 
        pack_cmd(&hip2, q[1][0], dq[1][0], kp, kd, 0); 
        pack_cmd(&knee1, q[2][1], dq[2][1], kp/1.5f, kd/2.25f, 0); 
        pack_cmd(&knee2, q[2][0], dq[2][0], kp/1.5f, kd/2.25f, 0); 
        */
    }
/*
    pack_cmd(&abad1, 0, 0, 10, .1, 0); 
    pack_cmd(&abad2, 0, 0, 10, .1, 0); 
    pack_cmd(&hip1, 0, 0, 10, .1, 0); 
    pack_cmd(&hip2, 0, 0, 10, .1, 0); 
    pack_cmd(&knee1, 0, 0, 6.6, .04, 0); 
    pack_cmd(&knee2, 0, 0, 6.6, .04, 0); 
*/    
    WriteAll();
    }
    
void Zero(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFE;
    WriteAll();
    }

void EnterMotorMode(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFC;
    WriteAll();
    }
    
void ExitMotorMode(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFD;
    WriteAll();
    }
void serial_isr(){
     /// handle keyboard commands from the serial terminal ///
     while(pc.readable()){
        char c = pc.getc();
        switch(c){
            case(27):
                loop.detach();
                printf("\n\r exiting motor mode \n\r");
                ExitMotorMode(&abad1);
                ExitMotorMode(&abad2);
                ExitMotorMode(&hip1);
                ExitMotorMode(&hip2);
                ExitMotorMode(&knee1);
                ExitMotorMode(&knee2);
                enabled = 0;
                break;
            case('m'):
                printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&abad1);
                Zero(&abad1);
                EnterMotorMode(&abad2);
                Zero(&abad2);
                EnterMotorMode(&hip1);
                Zero(&hip1);
                EnterMotorMode(&hip2);
                Zero(&hip2);
                EnterMotorMode(&knee1);
                Zero(&knee1);
                EnterMotorMode(&knee2);
                Zero(&knee2);
                wait(.5);
                enabled = 1;
                loop.attach(&sendCMD, .001);
                break;
            case('z'):
                printf("\n\r zeroing \n\r");
                Zero(&abad1);
                Zero(&abad2);
                Zero(&hip1);
                Zero(&hip2);
                Zero(&knee1);
                Zero(&knee2);
                break;
            }
        }
        WriteAll();
        
    }
    
int can_packet[8] = {1, 2, 3, 4, 5, 6, 7, 8};
int main() {
    pc.baud(921600);
    pc.attach(&serial_isr);
    can1.frequency(1000000);                     // set bit rate to 1Mbps
    can1.attach(&rxISR1);                 // attach 'CAN receive-complete' interrupt handler
    can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0); //set up can filter
    can2.frequency(1000000);                     // set bit rate to 1Mbps
    can2.attach(&rxISR2);                 // attach 'CAN receive-complete' interrupt handler
    can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0); //set up can filter
    
    printf("\n\r Master\n\r");
    //printf("%d\n\r", RX_ID << 18);
    abad1.len = 8;                         //transmit 8 bytes
    abad2.len = 8;                         //transmit 8 bytes
    hip1.len = 8;
    hip2.len = 8;
    knee1.len = 8;
    knee2.len = 8;
    rxMsg1.len = 6;                          //receive 5 bytes
    rxMsg2.len = 6;                          //receive 5 bytes

    
    abad1.id = 0x1;                        
    abad2.id = 0x1;                 
    hip1.id = 0x2;
    hip2.id = 0x2;
    knee1.id = 0x3;
    knee2.id = 0x3;       
    pack_cmd(&abad1, 0, 0, 0, 0, 0);       //Start out by sending all 0's
    pack_cmd(&abad2, 0, 0, 0, 0, 0);
    pack_cmd(&hip1, 0, 0, 0, 0, 0);
    pack_cmd(&hip2, 0, 0, 0, 0, 0);
    pack_cmd(&knee1, 0, 0, 0, 0, 0);
    pack_cmd(&knee2, 0, 0, 0, 0, 0);
    
    
    while(1) {

        }
        
    }
    




