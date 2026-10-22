u8 gimbalID[2] = {GIMBAL_PAN_ID, GIMBAL_TILT_ID};

s16 gimbalPos[2];
u16 gimbalSpd[2];
u8  gimbalAcc[2];

ServoFeedback gimbalFeedback[2];
// [0] PAN
// [1] TILT

float steadyGoalY = 0;

float constrainFloat(float value, float min, float max) {
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  }
  return value;
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void gimbalCtrlSimple(float Xinput, float Yinput, float spdInput, float accInput) {
  Xinput = constrainFloat(Xinput, -180, 180);
  Yinput = constrainFloat(Yinput, -30, 90);

  gimbalPos[0] = 2047 + (int)round(map(Xinput, 0, 360, 0, 4095));
  gimbalPos[1] = 2047 - (int)round(map(Yinput, 0, 360, 0, 4095));

  gimbalSpd[0] = (int)round(map(spdInput, 0, 360, 0, 4095));
  gimbalSpd[1] = (int)round(map(spdInput, 0, 360, 0, 4095));

  gimbalAcc[0] = (int)round(map(accInput, 0, 360, 0, 4095));
  gimbalAcc[1] = (int)round(map(accInput, 0, 360, 0, 4095));

  st.SyncWritePosEx(gimbalID, 2, gimbalPos, gimbalSpd, gimbalAcc);
}

void gimbalCtrlMove(float Xinput, float Yinput, float spdInputX, float spdInputY) {
  Xinput = constrainFloat(Xinput, -180, 180);
  Yinput = constrainFloat(Yinput, -30, 90);

  spdInputX = constrain(spdInputX, 1, 2500);
  spdInputY = constrain(spdInputY, 1, 2500);

  gimbalPos[0] = 2047 + (int)round(map(Xinput, 0, 360, 0, 4095));
  gimbalPos[1] = 2047 - (int)round(map(Yinput, 0, 360, 0, 4095));

  gimbalSpd[0] = spdInputX;
  gimbalSpd[1] = spdInputY;

  gimbalAcc[0] = 0;
  gimbalAcc[1] = 0;

  st.SyncWritePosEx(gimbalID, 2, gimbalPos, gimbalSpd, gimbalAcc);
}

float panAngleCompute(int inputPos) {
  return mapFloat((inputPos - 2047), 0, 4095, 0, 360);
}

float tiltAngleCompute(int inputPos) {
  return mapFloat((2047 - inputPos), 0, 4095, 0, 360);
}

void gimbalCtrlStop() {
  st.EnableTorque(GIMBAL_PAN_ID, 0);
  st.EnableTorque(GIMBAL_TILT_ID, 0);
  delay(SERVO_STOP_DELAY);
  st.EnableTorque(GIMBAL_PAN_ID, 1);
  st.EnableTorque(GIMBAL_TILT_ID, 1);
}

void getGimbalFeedback() {
  if(st.FeedBack(GIMBAL_PAN_ID)!=-1) {
    gimbalFeedback[0].status = true;
    gimbalFeedback[0].pos = st.ReadPos(-1);
    gimbalFeedback[0].speed = st.ReadSpeed(-1);
    gimbalFeedback[0].load = st.ReadLoad(-1);
    gimbalFeedback[0].voltage = st.ReadVoltage(-1);
    gimbalFeedback[0].current = st.ReadCurrent(-1);
    gimbalFeedback[0].temper = st.ReadTemper(-1);
    gimbalFeedback[0].mode = st.ReadMode(GIMBAL_PAN_ID);
  } else{
    servoFeedback[0].status = false;
    if(InfoPrint == 1){
      jsonInfoHttp.clear();
      jsonInfoHttp["T"] = 1005;
      jsonInfoHttp["id"] = GIMBAL_PAN_ID;
      jsonInfoHttp["status"] = 0;
      String getInfoJsonString;
      serializeJson(jsonInfoHttp, getInfoJsonString);
      Serial.println(getInfoJsonString);
    }
  }

  if(st.FeedBack(GIMBAL_TILT_ID)!=-1) {
    gimbalFeedback[1].status = true;
    gimbalFeedback[1].pos = st.ReadPos(-1);
    gimbalFeedback[1].speed = st.ReadSpeed(-1);
    gimbalFeedback[1].load = st.ReadLoad(-1);
    gimbalFeedback[1].voltage = st.ReadVoltage(-1);
    gimbalFeedback[1].current = st.ReadCurrent(-1);
    gimbalFeedback[1].temper = st.ReadTemper(-1);
    gimbalFeedback[1].mode = st.ReadMode(GIMBAL_TILT_ID);
  } else{
    servoFeedback[1].status = false;
    if(InfoPrint == 1){
      jsonInfoHttp.clear();
      jsonInfoHttp["T"] = 1005;
      jsonInfoHttp["id"] = GIMBAL_TILT_ID;
      jsonInfoHttp["status"] = 0;
      String getInfoJsonString;
      serializeJson(jsonInfoHttp, getInfoJsonString);
      Serial.println(getInfoJsonString);
    }
  }
}

void gimbalSteadySet(bool inputCmd, float inputY) {
  steadyMode = inputCmd;
  if (inputY < -45) {
    inputY = -45;
  } else if (inputY > 90) {
    inputY = 90;
  }
  steadyGoalY = inputY;
}

// --- ПИД для стабилизации PT (самописный, без PID_v2) ---
float pid_p = 0.35f;
float pid_i = 0.005f;
float pid_d = 0.15f;

float steady_integral = 0.0f;
float steady_last_error = 0.0f;
unsigned long steady_last_time = 0;

float filtered_pitch = 0.0f;
bool filter_initialized = false;

void gimbalSteadyInit() {
  steady_integral = 0.0f;
  steady_last_error = 0.0f;
  steady_last_time = 0;
  filter_initialized = false;
}

void gimbalSteady(float inputBiasY) {
  if (!steadyMode) {
    steady_integral = 0.0f;
    steady_last_error = 0.0f;
    filter_initialized = false;
    return;
  }

  unsigned long current_time = millis();
  float dt = (current_time - steady_last_time) / 1000.0f;

  if (steady_last_time == 0 || dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
    steady_last_error = inputBiasY - icm_pitch;
  }
  steady_last_time = current_time;

  if (!filter_initialized) {
    filtered_pitch = icm_pitch;
    filter_initialized = true;
  }
  filtered_pitch = (0.05f * icm_pitch) + (0.95f * filtered_pitch);

  float error = inputBiasY - filtered_pitch;

  float p_term = pid_p * error;

  steady_integral += error * dt;
  steady_integral = constrain(steady_integral, -5.0f, 5.0f);
  float i_term = pid_i * steady_integral;

  float derivative = (error - steady_last_error) / dt;
  float d_term = pid_d * derivative;
  steady_last_error = error;

  float compensation = p_term + i_term + d_term;
  compensation = constrainFloat(compensation, -30.0f, 90.0f);

  gimbalCtrlSimple(0, compensation, 0, 0);
}

// Установка коэффициентов ПИД извне (команда T:1400)
void gimbalPidSet(float p, float i, float d) {
  pid_p = p;
  pid_i = i;
  pid_d = d;
  Serial.printf("Gimbal PID set: P=%.3f I=%.4f D=%.3f\n", p, i, d);
}

void gimbalUserCtrl(int inputX, int inputY, int inputSpd) {
  static float goalX = 0;
  static float goalY = 0;

  if(inputX == -1 && inputY == 1){
    goalX = -180;
    goalY = 90;
  }
  else if(inputX == 0 && inputY == 1){
    goalY = 90;
  }
  else if(inputX == 1 && inputY == 1){
    goalX = 180;
    goalY = 90;
  }
  else if(inputX == -1 && inputY == 0){
    goalX = -180;
  }
  else if(inputX == 1 && inputY == 0){
    goalX = 180;
  }
  else if(inputX == -1 && inputY == -1){
    goalX = -180;
    goalY = -45;
  }
  else if(inputX == 0 && inputY == -1){
    goalY = -45;
  }
  else if(inputX == 1 && inputY == -1){
    goalX = 180;
    goalY = -45;
  }

  if(inputX == 2 && inputY == 2){
    gimbalCtrlSimple(0, 0, 0, 10);
  }
  else{
    gimbalCtrlSimple(goalX, goalY, inputSpd, 0);
    if(inputX == 0){
      servoTorqueCtrl(GIMBAL_PAN_ID, 0);
      delay(5);
      servoTorqueCtrl(GIMBAL_PAN_ID, 1);
      getGimbalFeedback();
      goalX = panAngleCompute(gimbalFeedback[0].pos);
    }
    if(inputY == 0){
      servoTorqueCtrl(GIMBAL_TILT_ID, 0);
      delay(5);
      servoTorqueCtrl(GIMBAL_TILT_ID, 1);
      getGimbalFeedback();
      goalY = tiltAngleCompute(gimbalFeedback[1].pos);
    }
  }
}
