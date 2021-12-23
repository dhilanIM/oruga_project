#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "Wire.h" // librería Wire.h


// MOTORES
const int in1 = 7;  // Pin que controla el sentido de giro Motor L
const int in2 = 8;  // Pin que controla el sentido de giro Motor L
const int EnA = 5; // Pin que controla la velocidad del  Motor L

const int in3 = 4;  // Pin que controla el sentido de giro Motor  R
const int in4 = 9;  // Pin que controla el sentido de giro Motor R
const int EnB = 6; // Pin que controla la velocidad del  Motor R


int L_motor_pwm = 39; // 0.15 m/s
int R_motor_pwm = 80;


unsigned short L_motor_pos = 0; 
unsigned short R_motor_pos = 0;
double L_wheel_vel = 0;
double R_wheel_vel = 0;

// ODOMETRIA DE LOS ENCODERS
#define PI 3.1415926535897932384626433832795

const int N=20;         // Numero de ticks 
const float R=0.055;     // Radio de la llanta en metros
const float L = 0.48;   // Distancia de una llanta a otra en metros

const int encoderR = 3;
const int encoderL = 2;
double Dr=0;
int Rtick = 0;
int RtickAnt =0;
int deltaRtick = 0;

float Dl=0;
int Ltick = 0;
int LtickAnt =0;
int deltaLtick = 0;

double Dc = 0;
double x=0;
double y=0;
double theta=0;

float v;
float w;

// ROS Handle, necesario para comunicarnos con ros
ros::NodeHandle nh;

// TRANSFORMS
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

geometry_msgs::TransformStamped t2;
tf::TransformBroadcaster broadcaster2;
char base_link[] = "/base_link";
char odom[] = "/odom";
char imu_[] = "/imu";


//  Para publicar odometria de encoders
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

// IMU
// Para publicar orientacion del imu 
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);
double x_imu;
double y_imu;
double z_imu;
double w_imu;

// Para fusion de sensores
double x_fusion = 0;
double y_fusion = 0;
double theta_fusion = 0;

//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0    // 32768/2
#define G_R 131.0       // 32768/250
 
//Conversion de radianes a grados 180/PI
//#define RAD_A_DEG = 57.295779
 #define RAD_A_DEG = 1
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];
 
String valores;
 
long tiempo_prev;
float dt;

void pose2D_Callback(const  geometry_msgs::Pose2D& msg){
  x_fusion = msg.x;
  y_fusion = msg.y;
  theta_fusion = msg.theta;
}
ros::Subscriber<geometry_msgs::Pose2D> sub2("pose2D",pose2D_Callback);


void cmd_velCallback( const geometry_msgs::Twist& msg){
  v = msg.linear.x;
  w = msg.angular.z;
/*
  v = constrain(v,-0.15,0.15);
  w = constrain(w,-2.5,2.5);*/
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_velCallback);


void odometria(){
  deltaRtick = Rtick - RtickAnt;
  Dr = 2*PI*R*(deltaRtick/(double) N);

  deltaLtick = Ltick - LtickAnt;
  Dl = 2*PI*R*(deltaLtick/(double) N);
  
  Dc = (Dl + Dr)/2;

  x = x + Dc*cos(theta)*0.5;
  y = y + Dc*sin(theta)*0.5;

  theta = theta + ((Dr - Dl)/L)*0.5;
  theta = atan2(sin(theta),cos(theta));
  RtickAnt = Rtick;
  LtickAnt = Ltick;
}

void LEncoder(){
  if(L_motor_pos == 1) //Hacia adelante
    Ltick++;
  else
    Ltick--;
    
  //Serial.print("Lt: ");
  //Serial.println(Ltick);
}
void REncoder(){
  if(R_motor_pos == 1)
    Rtick++;
 else
    Rtick--;
  //Serial.print("Rt: ");
  //Serial.println(Rtick);
}


void SetMotor(char motor,int pwm,char pos){
  if (motor == 'L'){  // motor L
    if(pos == 'f'){ // Hacia adelante
      analogWrite(EnA,pwm);      
      digitalWrite(in1, LOW);  
      digitalWrite(in2, HIGH);
      //L_motor_pos = 1;
    }
    else if(pos == 'b'){     // Hacia atras
      analogWrite(EnA,pwm);      
      digitalWrite(in1, HIGH);  
      digitalWrite(in2, LOW);
      //L_motor_pos = 0;
    }
    else if(pos == 'x'){
      analogWrite(EnA,pwm);      
      digitalWrite(in1, LOW);  
      digitalWrite(in2, LOW);
    }
  }
  else if(motor == 'R'){
    if(pos == 'f'){ // Hacia adelante
      analogWrite(EnB,pwm);      
      digitalWrite(in3, LOW);  
      digitalWrite(in4, HIGH);
      //R_motor_pos = 1;
    }
    else if(pos == 'b'){     // Hacia atras
      analogWrite(EnB,pwm);      
      digitalWrite(in3, HIGH);  
      digitalWrite(in4, LOW);
      //R_motor_pos = 0;
    } 
    else if(pos == 'x'){ // Pausa
      analogWrite(EnB,pwm);      
      digitalWrite(in3, LOW);  
      digitalWrite(in4, LOW);
    }
  }
}



void imu(){
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;
 
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];
 
   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = 0.98*(Angle[2]+Gy[2]*dt);
 
   //Mostrar los valores por consola
   valores = "90, " +String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90";
   //valores = "90, " +String(Gy[0]) + "," + String(Gy[1]) + "," + String(Gy[2]) + ",-90";
   //valores =String(Angle[2]); 
}

void getQuaternionFromRPY(double yaw, double pitch, double roll){
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    w_imu = cr * cp * cy + sr * sp * sy;
    x_imu = sr * cp * cy - cr * sp * sy;
    y_imu = cr * sp * cy + sr * cp * sy;
    z_imu = cr * cp * sy - sr * sp * cy;
}
void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  broadcaster.init(nh);
  
  pinMode(in1, OUTPUT);    // Configura  los pines como salida Motor L
  pinMode(in2, OUTPUT);
  pinMode(EnA, OUTPUT);

  pinMode(in3, OUTPUT);    // Configura  los pines como salida Motor R
  pinMode(in4, OUTPUT);
  pinMode(EnB, OUTPUT);

  pinMode(encoderR, INPUT_PULLUP);    // encoder pins
  pinMode(encoderL, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderR),REncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderL),LEncoder,CHANGE);

  Wire.begin(); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
    
  }

void loop() {
  nh.spinOnce();
  // put your main code here, to run repeatedly:
  imu();
  odometria();
  
 // tf odom->base_link
  geometry_msgs::TransformStamped t; // Creamos mensaje para transformaciones
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x_fusion;
  t.transform.translation.y = y_fusion;

  t.transform.rotation = tf::createQuaternionFromYaw(theta_fusion);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);


  // Publica odometria de encoders
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = x_fusion;
  odom_msg.pose.pose.position.y = y_fusion;
  odom_msg.pose.pose.position.z = 0.0;
  // Publicar velocidad en X obtenida en cmd_vel?
  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta_fusion);
  odom_msg.child_frame_id = base_link;
  odom_pub.publish(&odom_msg);



  
  getQuaternionFromRPY(Angle[2],Angle[1],Angle[0]);
  // Publica datos del imu
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = nh.now();
  imu_msg.orientation.x = x_imu;  
  imu_msg.orientation.y = y_imu;  
  imu_msg.orientation.z = z_imu;  
  imu_msg.orientation.w = w_imu; 
  //imu_msg.angular_velocity.z = GyZ;   // QUIEN SABE
  //imu_msg.linear_acceleration.x = Acc[0]; 
  imu_msg.header.frame_id = imu_;
  imu_pub.publish(&imu_msg);
  

  
  /*
  L_wheel_vel = v - (w*L*0.5);
  R_wheel_vel = v + (w*L*0.5);
  */ 
  
  L_wheel_vel = (2*v - w*L*0.5)/(2*R);
  R_wheel_vel = (2*v + w*L*0.5)/(2*R);


  if(L_wheel_vel > 0.0)
  {
    L_motor_pos = 1;
    L_motor_pwm = constrain(L_wheel_vel,0,5);
    L_motor_pwm = map(L_motor_pwm,0,5,30,56);
    SetMotor('L',L_motor_pwm,'f');
  }
  
  else if(L_wheel_vel < 0){
    L_motor_pos = 0;
    L_motor_pwm = constrain(L_wheel_vel,0,5);
    L_motor_pwm = map(L_motor_pwm,0,5,30,56);
    SetMotor('L',L_motor_pwm,'b');
  }
  else if(L_wheel_vel == 0){
    L_motor_pos = 1;
    SetMotor('L',0,'x');
  }

  if(R_wheel_vel >0.0 )
  {
    R_motor_pos = 1;
    R_motor_pwm = constrain(R_wheel_vel,0,5);
    R_motor_pwm = map(R_motor_pwm,0,5,65,150);
    SetMotor('R',R_motor_pwm,'f');
  }
  else if(L_wheel_vel < 0){
    R_motor_pos = 0;
    R_motor_pwm = constrain(R_wheel_vel,0,5);
    R_motor_pwm = map(R_motor_pwm,0,5,65,150);
    SetMotor('R',R_motor_pwm,'b');
  }
  else if(R_wheel_vel == 0){
    //R_motor_pos = 1;
    SetMotor('R',0,'x');
  }


}
