
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>

#define  T  (2*M_PI)/4096
#define MAX_SENS 4096

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("/odom",&odom_msg);
std_msgs::Int16 left_msg;
std_msgs::Int16 right_msg;
ros::Publisher left_pub("/signed_left_ticks",&left_msg);
ros::Publisher right_pub("/signed_right_ticks",&right_msg);
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float k0=0.00031263;
float k1=0.00031263;
float baseline=0.91;
char base_link[] = "/base_link";
char odom[] = "/odom";
// Declarate

const int CSn[2] = {25,37}; // Chip select
const int CLK[2] = {29,41}; // Clock signal
const int DO[2] = {33,45}; // Digital Output from the encoder which delivers me a 0 or 1, depending on the bar angle..

unsigned int* sens;
int old_sens[]={0,0};
int current_sens[2];
int delta[2];

void setup(){

//  Serial.begin(115200);
//  Serial.println("Hello World!"); //send sample text to monitor for debug
  
  //Fix de tris

  pinMode(CSn[0], OUTPUT);
  pinMode(CLK[0], OUTPUT);
  pinMode(DO[0], INPUT);
  
    pinMode(CSn[1], OUTPUT);
  pinMode(CLK[1], OUTPUT);
  pinMode(DO[1], INPUT);

  //Let's start here
  digitalWrite(CLK[0], HIGH);
  digitalWrite(CSn[0], HIGH);
  
  digitalWrite(CLK[1], HIGH);
  digitalWrite(CSn[1], HIGH);
 // nh.getHardware()->setBaud(115200); //or what ever baud you want
  nh.initNode();
  
//  broadcaster.init(nh);
    // tf odom->base_link
//  t.header.frame_id = odom;
//  t.child_frame_id = base_link;
  nh.advertise(odom_pub);
  nh.advertise(left_pub);
  nh.advertise(right_pub);
  delay(100);
  sens = readSensor();

  current_sens[0]=(int16_t)sens[0];
  current_sens[1]=(int16_t)sens[1];
  old_sens[0]=current_sens[0];
  old_sens[1]=current_sens[1];
  delay(100);
}



void loop() {

  sens = readSensor();
  
  current_sens[0]=(int16_t)sens[0];
  current_sens[1]=(int16_t)sens[1];
  
  delta[0]=normalizeDelta(current_sens[0]-old_sens[0]);
  delta[1]=normalizeDelta(current_sens[1]-old_sens[1]);
  
  if (abs(delta[0])<1000 && abs(delta[1])<1000){
  old_sens[0]=current_sens[0];
  old_sens[1]=current_sens[1];
  left_msg.data=delta[0];
  right_msg.data=delta[1];
  left_pub.publish(&left_msg);
  right_pub.publish(&right_msg);
 

  
    float t_0 = delta[0]*k0;
    float t_1 = delta[1]*k1;

    theta += (t_1 - t_0)/baseline;
    normalizeTheta(theta);
    float rho = (t_0 + t_1)/2;
    x += rho*cos(theta);
    y += rho*sin(theta);
/*  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  */  
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
    odom_pub.publish(&odom_msg);
    
  /*  odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(0);
    odom_pub.publish(&odom_msg);
    */
  }
  nh.spinOnce();  
  
  delay(10);
//  delayMicroseconds(1); //Tcs waiting for another read in
}
int normalizeDelta(int delta){
  int d=delta;
  if(d>MAX_SENS/2){
    d-=MAX_SENS;
  }else if(d<-(MAX_SENS/2)){
    d+=MAX_SENS;
  }
  return d;
}
unsigned int* readSensor(){
  unsigned int dataOut[]={0,0};

  digitalWrite(CSn[0], LOW);
  digitalWrite(CSn[1], LOW);
  delayMicroseconds(1); //Waiting for Tclkfe

  //Passing 12 times, from 0 to 11
  for(int x=0; x<12; x++){
    digitalWrite(CLK[0], LOW); 
    digitalWrite(CLK[1], LOW); 
    delayMicroseconds(1); //Tclk/2
    digitalWrite(CLK[0], HIGH);
    digitalWrite(CLK[1], HIGH);
    delayMicroseconds(1); //Tdo valid, like Tclk/2
   // unsigned int d;
    dataOut[0]= (dataOut[0] << 1) | digitalRead(DO[0]); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
    dataOut[1]= (dataOut[1] << 1) | digitalRead(DO[1]); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
  }

  digitalWrite(CSn[0], HIGH); //deselects the encoder from reading
    digitalWrite(CSn[1], HIGH); //deselects the encoder from reading
 //       Serial.println(dataOut);
  return dataOut;

}
void normalizeTheta(float &theta){
    theta = atan2(sin(theta),cos(theta));
}
