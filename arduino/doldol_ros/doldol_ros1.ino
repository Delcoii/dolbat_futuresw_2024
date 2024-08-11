/*
 * 
 * no Serial print while using ROS communication!!!
*/

#include <ros.h>
#include <std_msgs/Float32.h>

#define LOOP_MS                 16

#define PULSE_ERROR_MAX_US      2500
#define PULSE_ERROR_MIN_US      500
#define DETECTION_ERR           -999

#define STEERING_PULSE_PIN      18
#define ACCEL_PULSE_PIN         19

#define MODE_PULSE_PIN          21
#define MANUAL_MODE             1200    // for pretty plotting
#define AUTO_MODE               1500    // for pretty plotting     
#define BRAKE_MODE              1800    // for pretty plotting

#define CH4_PULSE_PIN           20

#define ACCEL_OFFSET            0.07
#define STEER_OFFSET            0.0
#define SIGNAL_THRESHOLD        0.1
       
#define STEER_DETECT_MAX        410    // rIght max steer
#define STEER_DETECT_MIN         749    // left max steer
#define MAX_STEER_TIRE_DEG      20.09
#define KP                      0.2
#define KI                       0
#define KD                     0

volatile long g_steering_edge_now_us = DETECTION_ERR;
volatile long g_steering_edge_before_us = DETECTION_ERR;
volatile long g_steering_us = DETECTION_ERR;

volatile long g_accel_edge_now_us = DETECTION_ERR;
volatile long g_accel_edge_before_us = DETECTION_ERR;
volatile long g_accel_us = DETECTION_ERR;

volatile long g_mode_edge_now_us = DETECTION_ERR;
volatile long g_mode_edge_before_us = DETECTION_ERR;
volatile long g_mode_us = DETECTION_ERR;




// #include <Car_Library.h>
int PIN_LEFT_IN1 = 4;
int PIN_LEFT_IN2 = 5;
int PIN_RIGHT_IN1 = 6;
int PIN_RIGHT_IN2 = 7;
int PIN_STEER_IN1 = 8;
int PIN_STEER_IN2 = 9;
int PIN_10 = 10;
int PIN_11 = 11;
int PIN_12 = 12;
int STEER_DETECT_PIN = A0;


double PID(double ref, double sense, double dt_us) {
  static double prev_err = 0.0;
  double err = ref - sense;
  double P = err * KP;
  double I = err * dt_us * KI;
  double D = ((err - prev_err) / dt_us) * KD;

  prev_err = err;
  return P+I+D;
}


void SteeringPulseInt() {
    g_steering_edge_now_us = micros();

    g_steering_us = g_steering_edge_now_us - g_steering_edge_before_us;
    g_steering_edge_before_us = g_steering_edge_now_us;
}

void AccelPulseInt() {
    g_accel_edge_now_us = micros();

    g_accel_us = g_accel_edge_now_us - g_accel_edge_before_us;
    g_accel_edge_before_us = g_accel_edge_now_us;
}

void ModePulseInt() {
    g_mode_edge_now_us = micros();
    
    g_mode_us = g_mode_edge_now_us - g_mode_edge_before_us;
    g_mode_edge_before_us = g_mode_edge_now_us;
}


float Mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void StopMotor() {
  analogWrite(PIN_LEFT_IN1, 0);
  analogWrite(PIN_LEFT_IN2, 0);
  analogWrite(PIN_RIGHT_IN1, 0);
  analogWrite(PIN_RIGHT_IN2, 0);
  analogWrite(PIN_STEER_IN1, 0);
  analogWrite(PIN_STEER_IN2, 0);
}


// input throttle 0.0 ~ 1.0
void MoveForward(double throttle) {
  if (throttle > 1.0) {
    throttle = 1.0;
  } else if (throttle < 0.0) {
    throttle = 0.0;
  }
  int in = (int)(Mapping(throttle, 0.0, 1.0, 0.0, 255.0));
  if (abs(throttle) < SIGNAL_THRESHOLD) {
    in = 0; 
  }
  
  analogWrite(PIN_LEFT_IN1, 0);
  analogWrite(PIN_LEFT_IN2, in);
  analogWrite(PIN_RIGHT_IN1, in);
  analogWrite(PIN_RIGHT_IN2, 0);
}

// input backward 0.0 ~ 1.0
void MoveBackward(double throttle) {
  if (throttle > 1.0) {
    throttle = 1.0;
  } else if (throttle < 0.0) {
    throttle = 0.0;
  }
  int in = (int)(Mapping(throttle, 0.0, 1.0, 0.0, 255.0));
  if (abs(throttle) < SIGNAL_THRESHOLD) {
    in = 0; 
  }
  

  analogWrite(PIN_LEFT_IN1, in);
  analogWrite(PIN_LEFT_IN2, 0);
  analogWrite(PIN_RIGHT_IN1, 0);
  analogWrite(PIN_RIGHT_IN2, in);
}


// input steer -1.0 ~ 1.0
void Steer(double throttle) {
  if (throttle > 1.0) {
    throttle = 1.0;
  } else if (throttle < -1.0) {
    throttle = -1.0;
  }
  int in = (int)(Mapping(throttle, -1.0, 1.0, -255.0, 255.0));
  if (abs(throttle) < SIGNAL_THRESHOLD) {
    in = 0; 
  }
  

  if (in > 0.0) {
    analogWrite(PIN_STEER_IN1, in);
    analogWrite(PIN_STEER_IN2, 0);
  }
  else {
    in *= -1.0;
    analogWrite(PIN_STEER_IN1, 0);
    analogWrite(PIN_STEER_IN2, in);
  }
}

// ros settings
ros::NodeHandle nh;
float throttle_msg_ = 0.0;   // -1.0 ~ 1.0 required
float steer_msg_ = 0.0;      // -1.0 ~ 1.0 required
std_msgs::Float32 o_steer_deg;
std_msgs::Float32 o_forward;
std_msgs::Float32 o_backward;
// std_msgs::Float32 o_potentiometer;

ros::Subscriber<std_msgs::Float32> s_throttle("/car/throttle", ThrottleCb);
ros::Subscriber<std_msgs::Float32> s_steer("/car/steering", SteerCb);
ros::Publisher p_steer_deg("/car/steer_out_deg", &o_steer_deg);
ros::Publisher p_forward("/car/forward_out", &o_forward);
ros::Publisher p_backward("/car/backward_out", &o_backward);
// ros::Publisher p_potentio("/car/potentiometer", &o_potentiometer);


void ThrottleCb( const std_msgs::Float32& msg) {
  if (msg.data >= -1.0 && msg.data <= 1.0) {
    throttle_msg_ = msg.data;
  } else if (msg.data > 1.0) {
    throttle_msg_ = 1.0;
  } else if (msg.data < -1.0) {
    throttle_msg_ = -1.0;
  }
}


// value to be -19.85 ~ 19.85
void SteerCb( const std_msgs::Float32& msg) {
  steer_msg_ = msg.data;
}




void setup() {
  // Serial.begin(9600);       // 시리얼 통신 시작, 통신 속도 설정
  
  pinMode(STEERING_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);
  pinMode(ACCEL_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);
  pinMode(MODE_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_PULSE_PIN), ModePulseInt, CHANGE);
  pinMode(STEER_DETECT_PIN, INPUT_PULLUP);
  
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_STEER_IN1, OUTPUT);
  pinMode(PIN_STEER_IN2, OUTPUT);
  
  StopMotor();

  nh.initNode();
  nh.advertise(p_steer_deg);
  nh.advertise(p_forward);
  nh.advertise(p_backward);

  nh.subscribe(s_throttle);
  nh.subscribe(s_steer);

}

void loop() {
  static int prev_t_us = 0;
  int t_us = micros();

  static int steering_val;
  static int accel_val;
  static int mode_val = BRAKE_MODE;

  cli();
  if ((g_steering_us > PULSE_ERROR_MIN_US) && (g_steering_us < PULSE_ERROR_MAX_US)) {
      steering_val = g_steering_us;
  }
  if ((g_accel_us > PULSE_ERROR_MIN_US) && (g_accel_us < PULSE_ERROR_MAX_US)) {
      accel_val = g_accel_us;
  }

  if ((g_mode_us > PULSE_ERROR_MIN_US) && (g_mode_us < PULSE_ERROR_MAX_US)) {
      if ((g_mode_us >= 800) && (g_mode_us <= 1200)) {
          mode_val = MANUAL_MODE;
      }
      else if ((g_mode_us > 1200) && g_mode_us <= 1700) {
          mode_val = AUTO_MODE;
      }
      else {
          mode_val = BRAKE_MODE;
      }
  }
  sei();
  


  float throttle_input = Mapping(accel_val, 1000.0, 2000.0, -1.0, 1.0);
  float steer_input = Mapping(steering_val, 1224.0, 1736.0, -1.0, 1.0);     //1250,1750   //1000,2000

  // Serial.print(throttle_input);
  // Serial.print("\t");
  // Serial.print(steer_input);
  // Serial.print("\t");
  // Serial.println(mode_val);
  
  throttle_input += ACCEL_OFFSET;
  steer_input += STEER_OFFSET;
  // Serial.print(throttle_input);
  // Serial.print("\t");
  // Serial.print(steer_input);
  // Serial.print("\t");
  // Serial.println(mode_val);


  digitalWrite(PIN_10, HIGH);
  digitalWrite(PIN_11, HIGH);
  digitalWrite(PIN_12, HIGH);

  /////////////////////////////////////////////////////////
  double ref_steer_deg = Mapping(steer_input, -1.0, 1.0, -MAX_STEER_TIRE_DEG, MAX_STEER_TIRE_DEG);
  ref_steer_deg *= -1.0;

  int potentio_val = analogRead(STEER_DETECT_PIN);
  
  double deg = Mapping(potentio_val, STEER_DETECT_MIN, STEER_DETECT_MAX, MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);
  int dt = t_us - prev_t_us;




  ///////////////////////////////////////////////////////////////////

  if (mode_val == BRAKE_MODE) {
    StopMotor();
  }
  else if (mode_val == MANUAL_MODE) {
    if (throttle_input > 0.0) {
      MoveForward(throttle_input);
      o_forward.data = throttle_input;
      o_backward.data = 0.0;
    }
    else {
      throttle_input *= -1.0;
      MoveBackward(throttle_input);
      o_forward.data = 0.0;
      o_backward.data = throttle_input;
    }
    // Steer(steer_input);
    double pid_return = PID(ref_steer_deg, deg, dt);
    // Serial.println(pid_return);
    if (pid_return > 1.0) {
      pid_return = 1.0;
    } else if (pid_return < -1.0) {
      pid_return = -1.0;
    }
    Steer(pid_return);
    o_steer_deg.data = deg;
    o_forward.data = throttle_input;
    // o_potentiometer.data = potentio_val;
  }
  else if (mode_val == AUTO_MODE) {
    if (throttle_msg_ > 0.0) {
      MoveForward(throttle_msg_);
      o_forward.data = throttle_msg_;
      o_backward.data = 0.0;
    }
    else {
      throttle_msg_ *= -1.0;
      MoveBackward(throttle_msg_);
      o_forward.data = 0.0;
      o_backward.data = throttle_msg_;
    }

    double pid_out = PID(steer_msg_, deg, dt);
    if (pid_out > 1.0) {
      pid_out = 1.0;
    } else if (pid_out < -1.0) {
      pid_out = -1.0;
    }
    Steer(pid_out);
    o_steer_deg.data = deg;
    o_forward.data = throttle_msg_;
    // o_potentiometer.data = potentio_val;
  }


  // publish by equal freq
  static int before_ms = 0;
  int now_ms = millis();
  if ((now_ms - before_ms) >= LOOP_MS) {
    before_ms = now_ms;

    p_steer_deg.publish(&o_steer_deg);
    p_forward.publish(&o_forward);
    p_backward.publish(&o_backward);
    // p_potentio.publish(&o_potentiometer);
  }
  // Serial.print("ref:");
  // Serial.print(ref_steer_deg);
  // Serial.print("\t,");
  // Serial.print("deg:");
  // Serial.println(deg);
  // Serial.print("pid out:");
  // Serial.println(pid_return);

  nh.spinOnce();

  prev_t_us = t_us;

}
