#include "DualMC33926MotorShield.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include<ros.h>
#include<ros/time.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<tf/tf.h>
#include<std_msgs/Float64.h>

double   wRSetpoint,wLSetpoint,LSetpoint,RSetpoint, LOutput, ROutput,Lerror,Rerror,Lsv,Rsv,wLsv,wRsv;
double Lpidin,Lpidout,Rpidin,Rpidout,Rthresh,Lthresh;
double wLpidin,wLpidout=0,wRpidin,wRpidout=0,wRthresh=0,wLthresh=0;
float angvel;
 float timeout=0;
 long pt=0,ct,tt;



DualMC33926MotorShield md;

Encoder leftW(5,2);
Encoder rightW(3,11);
long leftold=0, rightold=-999;
float d=0.12065,LcumError,RcumError,LlastError,RlastError,LrateError,RrateError;
float wLcumError,wRcumError,wLlastError,wRlastError,wLrateError,wRrateError,wRerror,wLerror;

void speedset(const geometry_msgs::Twist& speedvalue)
{

  Lsv=speedvalue.linear.x;
  Lsv=float( long( speedvalue.linear.x * 100)) / 100;
  Rsv=speedvalue.linear.x;
  LSetpoint=float( long( speedvalue.linear.x * 100)) / 100;
    RSetpoint=float( long( speedvalue.linear.x * 100)) / 100;
  wRsv=speedvalue.angular.z;
  wRSetpoint=speedvalue.angular.z ;
    wLSetpoint=-wRSetpoint;
  if(speedvalue.linear.x!=0 or speedvalue.angular.z!=0) 
  { md.setM1Speed(550*Lpidout+(157.5*wLpidout));  //right
  md.setM2Speed(550*Rpidout-(-157.5*wRpidout));
  Linearpid();
     wpid();
  }
else
{
    md.setM2Speed(0);//Right
     md.setM1Speed(0);
     leftW.Reset();
     rightW.Reset();
     
}

}
ros::NodeHandle nh;
geometry_msgs::Twist speedmsg;
std_msgs::Float64 lvelmsg,rvelmsg;
ros::Publisher lp("/lvel",&lvelmsg);
ros::Publisher rp("/rvel",&rvelmsg);
ros::Subscriber<geometry_msgs::Twist>sub("/cmd_vel",speedset);
 
class encoder
{
    private:
    float d=0.12065,l=0.20;
   float r=d/2;
    public:
    float Lrev,Rrev,Lvel,Rvel,Ldist,Rdist,x,y,theta,dth;
    float  leftnew;
  float rightnew;
 
    int dt=0;  
    encoder();
    void count();
    void odom();
};
encoder::encoder()
{
 // Lrev=Rrev=Lvel=Rvel=0;
  //count();
}
void encoder:: count()
{

 leftnew=leftW.read();
 rightnew=rightW.read();
  //Serial.println(leftnew);  
  if(leftnew!=leftold or rightnew!=rightold)
  { 
     //Serial.println( leftold-leftnew);
     leftold=leftnew;
      rightold=rightnew;
      
     
  
  }
  
 leftnew=leftW.Reset();
  rightnew=rightW.Reset();
   tt=ct-pt;
 ;  
     // Serial.println(leftold);    
      // Serial.println(tt);     
     
     pt=ct;
 }

void encoder::odom()
{
     
      Lrev=float(leftold*10)/1700;
      Rrev=float(rightold*10)/1700;
      Ldist=Lrev*PI*d; 
      Rdist=Rrev*PI*d;  
      Lvel=(Ldist);
      Rvel=(Rdist); 
      Lvel=float( long(Lvel * 100)) / 100;
      Rvel=float( long(Rvel * 100)) / 100;
      dth=((r/l)*(Rvel-Lvel)/2)*10;
      dth=dth/2;
      theta=theta+dth;
      float dx=(((Rvel+Lvel)/2)*cos(int(theta)));
     x=x+(dx/10);
      float dy=(((Rvel+Lvel)/2)*sin(int(theta)));
       y=y+(dy/10);
      //dt=dt+1;
    lvelmsg.data=Lvel;
rvelmsg.data=Rvel;
//Serial.println(Lvel);
Serial.println(Rvel);      
}
  


 char base[]="/base_link";
 char o[]="/odom";


encoder e;

float cerrorL=0,cerrorR=0,wcerrorL=0,wcerrorR=0;
void Linearpid()
{
e.count();
    e.odom();
  Lpidin=e.Lvel;
  Rpidin=e.Rvel;
  Rerror=RSetpoint-Rpidin;  
  Lerror=LSetpoint-Lpidin;
 cerrorL=(cerrorL)+Lerror*0.01;
 cerrorR=(cerrorR)+Rerror*0.01;
  Rpidout=RSetpoint+(0.03*Rerror+0.04*cerrorR);
  Lpidout=LSetpoint+(0.03*Lerror+0.04*cerrorL);
 // Serial.println(Lpidout);
   LlastError =Lerror;
    RlastError=Rerror;
    
    
}
void wpid()
{
//e.count();
    // e.odom();
wLpidin=e.Lvel;
  wRpidin=e.Rvel;
  wcerrorL=(wcerrorL)+wLerror*0.01;
 wcerrorR=(wcerrorR)+wRerror*0.01;
  wRpidout=wRSetpoint+(0.03*wRerror+0.04*wcerrorR);
  wLpidout=wLSetpoint+(0.03*wLerror+0.04*wcerrorL);
  wLlastError =wLerror;
   wRlastError=wRerror;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
 
  Serial.begin(9600);

  
 
 e.count();
    
 
  
  nh.subscribe(sub);
  md.init();
  Lpidout=0;
  Rpidout=0;
   nh.initNode();
  nh.advertise(lp);
  nh.advertise(rp);
}
void loop()
{

     e.odom();
    
         lp.publish(&lvelmsg);
    rp.publish(&rvelmsg); 
  nh.spinOnce();

   Linearpid();
     wpid();
  delay(100);
  
}
