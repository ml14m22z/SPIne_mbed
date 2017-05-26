
#define CAN_ID 0x1

#include "mbed.h"
#include "math_ops.h"


Serial       pc(PA_2, PA_3);
CAN          can(PB_8, PB_9);  // CAN Rx pin name, CAN Tx pin name
CANMessage   rxMsg;
CANMessage   txMsg1;
CANMessage    txMsg2;
int                     ledState;
Timer                   timer;
Ticker                  sendCAN;
int                     counter = 0;
volatile bool           msgAvailable = false;
Ticker loop;

 float theta1, theta2, dtheta1, dtheta2;

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

void unpack_reply(CANMessage msg){
    /// unpack ints from can buffer ///
    int id = msg.data[0];
    int p_int = (msg.data[1]<<8)|msg.data[2];
    int v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    /// convert ints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    
    if(id == 2){
        theta1 = p;
        dtheta1 = v;
        }
    else if(id ==3){
        theta2 = p;
        dtheta2 = v;
        }
    
    } 
    
 void onMsgReceived() {
    can.read(rxMsg);                    // read message into Rx message storage
    unpack_reply(rxMsg);
}


void sendCMD(){
    /// bilateral teleoperation demo ///
    pack_cmd(&txMsg1, theta2, dtheta2, 10, .1, 0); 
    pack_cmd(&txMsg2, theta1, dtheta1, 10, .1, 0);
    can.write(txMsg2);
    wait(.0003);        // Give motor 1 time to respond.
    can.write(txMsg1);
    }
    
void serial_isr(){
     /// hangle keyboard commands from the serial terminal ///
     while(pc.readable()){
        char c = pc.getc();
        switch(c){
            case(27):
                printf("\n\r exiting motor mode \n\r");
                txMsg1.data[0] = 0xFF;
                txMsg1.data[1] = 0xFF;
                txMsg1.data[2] = 0xFF;
                txMsg1.data[3] = 0xFF;
                txMsg1.data[4] = 0xFF;
                txMsg1.data[5] = 0xFF;
                txMsg1.data[6] = 0xFF;
                txMsg1.data[7] = 0xFD;
                
                txMsg2.data[0] = 0xFF;
                txMsg2.data[1] = 0xFF;
                txMsg2.data[2] = 0xFF;
                txMsg2.data[3] = 0xFF;
                txMsg2.data[4] = 0xFF;
                txMsg2.data[5] = 0xFF;
                txMsg2.data[6] = 0xFF;
                txMsg2.data[7] = 0xFD;
                break;
            case('m'):
                printf("\n\r entering motor mode \n\r");
                txMsg1.data[0] = 0xFF;
                txMsg1.data[1] = 0xFF;
                txMsg1.data[2] = 0xFF;
                txMsg1.data[3] = 0xFF;
                txMsg1.data[4] = 0xFF;
                txMsg1.data[5] = 0xFF;
                txMsg1.data[6] = 0xFF;
                txMsg1.data[7] = 0xFC;
                
                txMsg2.data[0] = 0xFF;
                txMsg2.data[1] = 0xFF;
                txMsg2.data[2] = 0xFF;
                txMsg2.data[3] = 0xFF;
                txMsg2.data[4] = 0xFF;
                txMsg2.data[5] = 0xFF;
                txMsg2.data[6] = 0xFF;
                txMsg2.data[7] = 0xFC;
                break;
            case('z'):
                printf("\n\r zeroing \n\r");
                txMsg1.data[0] = 0xFF;
                txMsg1.data[1] = 0xFF;
                txMsg1.data[2] = 0xFF;
                txMsg1.data[3] = 0xFF;
                txMsg1.data[4] = 0xFF;
                txMsg1.data[5] = 0xFF;
                txMsg1.data[6] = 0xFF;
                txMsg1.data[7] = 0xFE;
                
                txMsg2.data[0] = 0xFF;
                txMsg2.data[1] = 0xFF;
                txMsg2.data[2] = 0xFF;
                txMsg2.data[3] = 0xFF;
                txMsg2.data[4] = 0xFF;
                txMsg2.data[5] = 0xFF;
                txMsg2.data[6] = 0xFF;
                txMsg2.data[7] = 0xFE;
                break;
            }
        }
        can.write(txMsg1);
        can.write(txMsg2);
        
    }
    
int can_packet[8] = {1, 2, 3, 4, 5, 6, 7, 8};
int main() {
    pc.baud(921600);
    pc.attach(&serial_isr);
    can.frequency(1000000);                     // set bit rate to 1Mbps
    can.attach(&onMsgReceived);                 // attach 'CAN receive-complete' interrupt handler
    can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0); //set up can filter
    printf("Master\n\r");
    //printf("%d\n\r", RX_ID << 18);
    int count = 0;
    txMsg1.len = 8;                         //transmit 8 bytes
    txMsg2.len = 8;                         //transmit 8 bytes
    rxMsg.len = 6;                          //receive 5 bytes
    loop.attach(&sendCMD, .001);
    txMsg1.id = 0x2;                        //1st motor ID
    txMsg2.id = 0x3;                        //2nd motor ID
    pack_cmd(&txMsg1, 0, 0, 0, 0, 0);       //Start out by sending all 0's
    pack_cmd(&txMsg2, 0, 0, 0, 0, 0);
    timer.start();
    
    while(1) {

        }
        
    }
    




