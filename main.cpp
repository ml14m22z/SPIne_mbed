
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
AnalogIn     knob(PC_0);
DigitalOut  toggle(PC_3);
DigitalOut  toggle2(PC_2);

///[[abad1,  abad2]
///[hip1,   hip2]
///[knee1, knee2]]
float q1raw[3];
float q2raw[3];
float dq1raw[3];
float dq2raw[3];
float q1[3];    //Leg 1 joint angles
float dq1[3];   //Leg 1 joint velocities
float tau1[3];  //Leg 1 joint torques
float q2[3];    //Leg 2 joint angles
float dq2[3];   //Leg 2 joint velocities
float tau2[3];  //Leg 2 joint torques
float p1[3];    //Leg 1 end effector position
float v1[3];    //Leg 1 end effector velocity
float J1[3][3]; //Leg 1 Jacobian
float f1[3];    //Leg 1 end effector forces
float p2[3];    //Leg 2 end effector position
float v2[3];    //Leg 2 end effector velocity
float J2[3][3]; //Leg 2 Jacobian
float f2[3];    //Leg 2 end effector forces

float q[3][2];   //Joint states for both legs
float dq[3][2];
float tau[3][3];

float I[3] = {0.005f, 0.0045f, 0.006f}; //Joint space inertias
float M1[3][3]; //Leg 1 end effector inverse mass
float M2[3][3]; //Leg 2 end effector inverse mass
float KD1[3]; //Joint space damping
float KD2[3];
float contact1[3];
float contact2[3];

const float offset[3] = {0.0f, 3.493f, -2.766f}; //Joint angle offsets at zero position

float kp = 800.0f;
float kd = 100.0f;
float kp_q = 100.0f;
float kd_q = 0.8f;
int enabled = 0;
float scaling = 0;


int control_mode = 0;

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
 
 #define L1 0.0577f
 #define L2 0.2088f
 #define L3 0.175f
 
 void kinematics(const float q[3], const float dq[3], float* p, float* v, float (* J)[3], float (* M)[3]){
     const float s1 = sinf(q[0]);
     const float s2 = sinf(q[1]);
     const float s3 = sinf(q[2]);
     const float c1 = cosf(q[0]);
     const float c2 = cosf(q[1]);
     const float c3 = cosf(q[2]);
     
     const float c23 = c2*c3 - s2*s3;
     const float s23 = s2*c3 + c2*s3;
     
     p[0] = L3*s23 + L2*s2;
     p[1] = L1*c1 + L3*s1*c23 + L2*c2*s1;
     p[2] = L1*s1 - L3*c1*c23 - L2*c1*c2;
     
     J[0][0] = 0;
     J[0][1] = L3*c23 + L2*c2;
     J[0][2] = L3*c23;
     J[1][0] = L3*c1*c23 + L2*c1*c2 - L1*s1;
     J[1][1] = -L3*s1*s23 - L2*s1*s2;
     J[1][2] = -L3*s1*s23;
     J[2][0] = L3*s1*c23 + L2*c2*s1 + L1*c1;
     J[2][1] = L3*c1*s23 + L2*c1*s2;
     J[2][2] = L3*c1*s23;
     
     M[0][0] = J[0][0]*J[0][0]/I[0] + J[0][1]*J[0][1]/I[1] + J[0][2]*J[0][2]/I[2];
     M[0][1] = 0;
     M[0][2] = 0;
     M[1][0] = 0;
     M[1][1] = J[1][0]*J[1][0]/I[0] + J[1][1]*J[1][1]/I[1] + J[1][2]*J[1][2]/I[2];
     M[1][2] = 0;
     M[2][0] = 0;
     M[2][1] = 0;
     M[2][2] = J[2][0]*J[2][0]/I[0] + J[2][1]*J[2][1]/I[1] + J[2][2]*J[2][2]/I[2];
     
     v[0] = 0; v[1] = 0; v[2] = 0;
     for(int i = 0; i<3; i++){
         for(int j = 0; j<3; j++){
             v[i] += J[i][j]*dq[j];
             }
            }
     }
 
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
    float qraw = p;
    float vraw = v;
    if(id==3){  //Extra belt 28:18 belt reduction on the knees;
        p = -p*0.643f;
        v = -v*0.643f;
        }
    else if(id==1){
        p = -p;
        v = -v;
        }
    p = p+offset[id-1];
    if(leg_num == 0){
        q1raw[id-1] = qraw;
        dq1raw[id-1] = vraw;
        q1[id-1] = p;
        dq1[id-1] = v;
        }
    else if(leg_num==1){
        q2raw[id-1] = qraw;
        dq2raw[id-1] = vraw;
        q2[id-1] = p;
        dq2[id-1] = v;
        }
    
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
    //toggle = 1;
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
    //toggle = 0;
    }

void sendCMD(){
    /// bilateral teleoperation demo ///
    toggle2 = 1;
    counter ++;
    scaling = .99f*scaling + .01f*knob.read();
    
    kinematics(q1, dq1, p1, v1, J1, M1);
    kinematics(q2, dq2, p2, v2, J2, M2);
    
    if(enabled){
            switch(control_mode){
                case 0:
                    {
                    KD1[0] = 0;  KD1[1] = 0;  KD1[2] = 0;
                    KD2[0] = 0;  KD2[1] = 0;  KD2[2] = 0;
                    tau1[0] = 0; tau1[1] = 0; tau1[2] = 0;
                    tau2[0] = 0; tau2[1] = 0; tau2[2] = 0;
                    pack_cmd(&abad1, 0, 0, 0, 0, 0); 
                    pack_cmd(&abad2, 0, 0, 0, 0, 0); 
                    pack_cmd(&hip1, 0, 0, 0, 0, 0); 
                    pack_cmd(&hip2, 0, 0, 0, 0, 0); 
                    pack_cmd(&knee1, 0, 0, 0, 0, 0); 
                    pack_cmd(&knee2, 0, 0, 0, 0, 0); 
                    }
                    break;
                case 1:
                {
                    //Joint Coupling
                    KD1[0] = 0;  KD1[1] = 0;  KD1[2] = 0;
                    KD2[0] = 0;  KD2[1] = 0;  KD2[2] = 0;
                    tau1[0] = -scaling*(kp_q*(q2[0] - q1[0]) + kd_q*(dq2[0] - dq1[0]));
                    tau2[0] = -scaling*(kp_q*(q1[0] - q2[0]) + kd_q*(dq1[0] - dq2[0]));
                    tau1[1] = scaling*(kp_q*(q2[1] - q1[1]) + kd_q*(dq2[1] - dq1[1]));
                    tau2[1] = scaling*(kp_q*(q1[1] - q2[1]) + kd_q*(dq1[1] - dq2[1]));
                    tau1[2] = -scaling*((kp_q/1.5f)*(q2[2] - q1[2]) + (kd_q/2.25f)*(dq2[2] - dq1[2]));
                    tau2[2] = -scaling*((kp_q/1.5f)*(q1[2] - q2[2]) + (kd_q/2.25f)*(dq1[2] - dq2[2]));
                    
                    pack_cmd(&abad1, 0, 0, 0, KD1[0]+.005f, tau1[0]); 
                    pack_cmd(&abad2, 0, 0, 0, KD2[0]+.005f, tau2[0]); 
                    pack_cmd(&hip1, 0, 0, 0, KD1[1]+.005f, tau1[1]); 
                    pack_cmd(&hip2, 0, 0, 0, KD2[1]+.005f, tau2[1]); 
                    pack_cmd(&knee1, 0, 0, 0, KD1[2]+.0033f, tau1[2]); 
                    pack_cmd(&knee2, 0, 0, 0, KD2[2]+.0033f, tau2[2]); 
                    }
                    break;
                
                case 2:
                {
                    //Virtual Walls
                    const float kmax = 25000.0f;
                    const float wn_des = 100000.0f;
                    const float xlim = 0.0f;
                    const float ylim = 0.2f;
                    const float zlim = -.2f;
                    
                    contact1[0] = p1[0]<xlim;
                    contact2[0] = p2[0]<xlim;
                    contact1[1] = p1[1]>ylim;
                    contact2[1] = p2[1]>ylim;
                    contact1[2] = p1[2]<zlim;
                    contact2[2] = p2[2]<zlim;
                    
                    float kx1 = wn_des/M1[0][0];
                    float kx2 = wn_des/M2[0][0];
                    kx1 = fminf(kmax, kx1);
                    kx2 = fminf(kmax, kx2);
                    f1[0] = scaling*(kx1*(xlim - p1[0]) + 0.0f*kd*(0 - v1[0]))*contact1[0];
                    f2[0] = scaling*(kx2*(xlim - p2[0]) + 0.0f*kd*(0 - v2[0]))*contact2[0];
                    
                    float ky1 = wn_des/M1[1][1];
                    float ky2 = wn_des/M2[1][1];
                    ky1 = fminf(kmax, ky1);
                    ky2 = fminf(kmax, ky2);
                    f1[1] = scaling*(ky1*(ylim - p1[1]) + 0.0f*kd*(0 - v1[1]))*contact1[1];
                    f2[1] = scaling*(ky2*(ylim - p2[1]) + 0.0f*kd*(0 - v2[1]))*contact2[1];
                    
                    float kz1 = wn_des/M1[2][2];
                    float kz2 = wn_des/M2[2][2];
                    kz1 = fminf(kmax, kz1);
                    kz2 = fminf(kmax, kz2);
                    f1[2] = scaling*(kz1*(zlim - p1[2]) + 0.0f*kd*(0 - v1[2]))*contact1[2];
                    f2[2] = scaling*(kz2*(zlim - p2[2]) + 0.0f*kd*(0 - v2[2]))*contact2[2];
                    //
                    
                    tau1[0] = -1*(f1[0]*J1[0][0] + f1[1]*J1[1][0] + f1[2]*J1[2][0]);
                    tau2[0] = -1*(f2[0]*J2[0][0] + f2[1]*J2[1][0] + f2[2]*J2[2][0]);
                    tau1[1] = f1[0]*J1[0][1] + f1[1]*J1[1][1] + f1[2]*J1[2][1];
                    tau2[1] = f2[0]*J2[0][1] + f2[1]*J2[1][1] + f2[2]*J2[2][1];
                    tau1[2] = -1*(f1[0]*J1[0][2] + f1[1]*J1[1][2] + f1[2]*J1[2][2]);
                    tau2[2] = -1*(f2[0]*J2[0][2] + f2[1]*J2[1][2] + f2[2]*J2[2][2]);
                    
                    KD1[0] = 0.4f*(kd*scaling)*(contact1[0]*J1[0][0]*J1[0][0] + contact1[1]*J1[1][0]*J1[1][0] + contact1[2]*J1[2][0]*J1[2][0]);
                    KD2[0] = 0.4f*(kd*scaling)*(contact2[0]*J2[0][0]*J2[0][0] + contact2[1]*J2[1][0]*J2[1][0] + contact2[2]*J2[2][0]*J2[2][0]);
                    KD1[1] = 0.4f*(kd*scaling)*(contact1[0]*J1[0][1]*J1[0][1] + contact1[1]*J1[1][1]*J1[1][1] + contact1[2]*J1[2][1]*J1[2][1]);
                    KD2[1] = 0.4f*(kd*scaling)*(contact2[0]*J2[0][1]*J2[0][1] + contact2[1]*J2[1][1]*J2[1][1] + contact2[2]*J2[2][1]*J2[2][1]);
                    KD1[2] = 0.4f*0.44f*(kd*scaling)*(contact1[0]*J1[0][2]*J1[0][2] + contact1[1]*J1[1][2]*J1[1][2] + contact1[2]*J1[2][2]*J1[2][2]);
                    KD2[2] = 0.4f*0.44f*(kd*scaling)*(contact2[0]*J2[0][2]*J2[0][2] + contact2[1]*J2[1][2]*J2[1][2] + contact2[2]*J2[2][2]*J2[2][2]);
                    
                    pack_cmd(&abad1, 0, 0, 0, KD1[0]+.005f, tau1[0]); 
                    pack_cmd(&abad2, 0, 0, 0, KD2[0]+.005f, tau2[0]); 
                    pack_cmd(&hip1, 0, 0, 0, KD1[1]+.005f, tau1[1]); 
                    pack_cmd(&hip2, 0, 0, 0, KD2[1]+.005f, tau2[1]); 
                    pack_cmd(&knee1, 0, 0, 0, KD1[2]+.0033f, tau1[2]); 
                    pack_cmd(&knee2, 0, 0, 0, KD2[2]+.0033f, tau2[2]); 
                    }
                    break;
                
                case 3:
                {
                    pack_cmd(&abad1, q2raw[0], dq2raw[0], 2.0f*kp_q*scaling, 2.0f*kd_q*scaling, 0); 
                    pack_cmd(&abad2, 0, 0, 0, 0, 0); 
                    pack_cmd(&hip1, q2raw[1], dq2raw[1], 2.0f*kp_q*scaling, 2.0f*kd_q*scaling, 0); 
                    pack_cmd(&hip2, 0, 0, 0, 0, 0); 
                    pack_cmd(&knee1, q2raw[2], dq2raw[2], 2.0f*kp_q*scaling, 2.0f*kd_q*scaling, 0); 
                    pack_cmd(&knee2, 0, 0, 0, 0, 0); 
                    }
                    break;
            //
            
            }
    

            
            
            
            
            

        if(counter>100){
            //tcmd = -1*tcmd;
            //printf("%.4f   %.4f  \n\r", q1[1], q2[1]);
            //printf("%f\n\r", scaling);
            counter = 0 ;
            }
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
toggle2 = 0;
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
            case('0'):
                control_mode = 0;
                break;
            case('1'):
                control_mode = 1;
                break;
            case('2'):
                control_mode = 2;
                break;
            case('3'):
                control_mode = 3;
                break;
            }
        }
        WriteAll();
        
    }
    
int can_packet[8] = {1, 2, 3, 4, 5, 6, 7, 8};
int main() {
    //wait(.5);
    
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
    //WriteAll();
    
    wait(.5);
    EnterMotorMode(&abad1);
    EnterMotorMode(&abad2);
    EnterMotorMode(&hip1);
    EnterMotorMode(&hip2);
    EnterMotorMode(&knee1);
    EnterMotorMode(&knee2);
    Zero(&knee2);
    Zero(&knee1);
    Zero(&hip1);
    Zero(&hip2);
    Zero(&abad2);
    Zero(&abad1);



    wait(.5);
    enabled = 1;
    loop.attach(&sendCMD, .001);
    while(1) {
        

        }
        
    }
    




