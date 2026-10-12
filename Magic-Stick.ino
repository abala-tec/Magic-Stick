// ---- palm-yaw (gyro) ball drive ----
#define YAW_AXIS     2      // gyro axis pointing up when stick flat on palm (0=x 1=y 2=z)
#define YAW_SIGN     1.0    // flip if ball rolls opposite to palm rotation
#define YAW_DEADBAND 3.0    // dps, below this bias tracker runs (bias/noise)
#define ROLL_GAIN    4      // rolling audio amplitude (1 = original)
#define YAW_BIAS_ALPHA 0.001 // gyro bias tracker rate (harvested while still)
#define DEMO_MODE    3      // 1 = classic tilt_y | 2 = absolute yaw, no gate | 3 = absolute yaw, gated

#include "accel_control.h"  // MPU control (for stick roll and acceleration)
#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include "ball_table.h"

#define CONTROL_RATE 256 // Hz, powers of 2 are most reliable

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

float acc=0; float p_acc=0;  // simulated marbel acceleration
float vel=0; float p_vel=0;  // simulated marbel velocity
float dist; float p_dist;  // simulated marbel distance
const float LEN = 0.6; //2.0; //stick lenght in m
const float LEN_MM = 600; //2000; //stick lenght in mm
const float H = 1.0/256.0;  // drag simulation teime step in seconds
float impact_vel=0;
float yawAngle = 0;  // absolute yaw since boot [rad], DEMO_MODE 2/3
#define IMPACT_GAIN 32.0            // calibracion actuador
volatile int impact_amp = 0;        // = IMPACT_GAIN * v_impacto
volatile int impact_samples = 0;    // muestras restantes del pulso

void setup(){
  Serial.begin(115200);
  // Serial.begin(9600);
  while (!Serial) delay(10); // will pause until serial console opens-
  setup_accelero();
  vel = 0.0; acc= 0.0;
  p_vel = 0.0; p_acc=0.0;
  dist = p_dist = LEN / 2.0;
  startMozzi(CONTROL_RATE);
}

// Impacto segun paper: pulso de amplitud fija durante 1 periodo de muestreo,
// amplitud proporcional a v_impacto -> energia ~ v^2, duracion siempre 3.9 ms.
void marbelImpact(){
  impact_amp = (int)(IMPACT_GAIN * fabs(impact_vel));
  impact_samples = AUDIO_RATE / CONTROL_RATE;  // 1 periodo control
}


float yawRateClean(){  // dps, gyro bias tracked while stick is still
  static float bias = 0;
  float rate = gyro_f[YAW_AXIS];
  if (fabs(rate) < YAW_DEADBAND) bias += YAW_BIAS_ALPHA * (rate - bias);
  return rate - bias;
}

// Function when the ball rolls up
void rollingUp(float roll){
  acc = 7.0 * sin(roll);
  //acc = az;

  if (p_dist > LEN){
    if (vel != 0) { impact_vel = vel; marbelImpact(); }  // evita retrigger en pared
    vel = 0;
  }else{
    vel = p_vel + (H * ( (p_acc + acc ) / 2.0 )   ) ;
  }

  dist = p_dist + (H * ((p_vel + vel )/ 2.0)  );
   
  p_acc = acc;
  p_vel = vel;
  p_dist = dist;
}

// Function when the ball rolls DOWN
void rollingDown(double roll){
   float sak = sin(roll);
   if (sak*sak < 0.2) {
    acc = 0;
   }else{
    acc = (9.8*sak - 1.96*(sak*-1)) * sqrt (1- (sak*sak) ) ;
   }

   if (p_dist < 0){
      if (vel != 0)  { impact_vel = -vel; marbelImpact(); }
      vel = 0;
   }else{
      vel = p_vel + (H * ( (p_acc + acc ) / 2.0 )   ) ;
   }

   dist = p_dist + (H *((p_vel + vel)/2.0 ) );
   
   p_acc = acc;
   p_vel = vel;
   p_dist = dist;
}


// DEMO_MODE 2/3: symmetric drive, absolute yaw angle fed like a tilt
void rollingYaw(float a){
  acc = 7.0 * sin(a);
  if ((p_dist > LEN && acc > 0) || (p_dist < 0 && acc < 0)){
    if (vel != 0) { impact_vel = (acc > 0) ? vel : -vel; marbelImpact(); }
    vel = 0;
  }else{
    vel = p_vel + (H * ((p_acc + acc) / 2.0) );
  }

  dist = p_dist + (H * ((p_vel + vel) / 2.0) );

  p_acc = acc;
  p_vel = vel;
  p_dist = dist;
}


float TILT_LIMIT = DEG_TO_RAD * 1;  
void updateControl(){
  getAccInfo();  // gets the accelerometer info 
  
#if DEMO_MODE == 2 || DEMO_MODE == 3
  yawAngle += YAW_SIGN * yawRateClean() * DEG_TO_RAD * H;
  if (yawAngle > PI)  yawAngle -= 2*PI;
  if (yawAngle < -PI) yawAngle += 2*PI;
#endif
#if DEMO_MODE == 2
  rollingYaw(yawAngle);                    // totally symmetric, ignores TILT_LIMIT
#elif DEMO_MODE == 3
  if (yawAngle > TILT_LIMIT || yawAngle < -TILT_LIMIT) rollingYaw(yawAngle);
#else
  if ((tilt_y < 2*PI-TILT_LIMIT) && (tilt_y > PI)){
    rollingDown(tilt_y);
    // Serial.print ("down ");
  }else if ((tilt_y > TILT_LIMIT) && (tilt_y < PI)){
    rollingUp(tilt_y);  
    // Serial.print ("up ");
  }
#endif

//  Serial.print (tilt_y);
//  Serial.print (",");
//  Serial.print (acc);
//  Serial.print (",");
//  Serial.print (vel);
//  Serial.print (",");
//  Serial.print (impact_vel);
//  Serial.print (",");
//  Serial.println (dist);

}

int dd = 0;
int updateAudio(){
  if (impact_samples > 0){ impact_samples--; return impact_amp; }  // pulso impacto
  dd = (int)(dist * 1000);
  if ((dd < 0) || (dd > LEN_MM)) return 0;  // roca fuera: silencio
  return isinTable8[dd % 30] * ROLL_GAIN;   // rodadura
}

void loop() {
  audioHook(); // required to be here
}
