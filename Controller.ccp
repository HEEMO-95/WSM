#include <Arduino.h>

byte uart[10]; // raw readings
byte data[7]; // raw reding request 
byte cmd[7]; // movment command

String getValue(String, char, int);
String myCmd;
String yaw;
String pitch;
String mode;

float thr = 0.2;
int control_mode = 0;

int ang = 1;
int i;
int t_new;
int t_old;
int dt;

int yaw_k1 = 5;
int yaw_k2 = 40;

int pitch_k1 = 10;
int pitch_k2 = 10;

int pitch_control;
int yaw_control;

int16_t pitch_raw;
int16_t yaw_raw;

float pitch_target = 90;
float pitch_current;
float pitch_error = 0;
float pitch_old_error;
float pitch_d_error;
float pitch_i;

float yaw_target = 90;
float yaw_current;
float yaw_error = 0;
float yaw_old_error;
float yaw_d_error;
float yaw_i;

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(115200);
  Serial2.begin(115200);  // Initialize Serial1 port

  data[0] = 0xFF;  data[1] = 0x01;  data[2] = 0xA0;  data[3] = 0x00;  data[4] = 0x00;  data[5] = 0x00;
  data[6] = (data[1] + data[2] + data[3] + data[4] + data[5]) & 0xff;


  cmd[0] = 0xFF;  cmd[1]  = 0x20;  cmd[2]  = 0x00;  cmd[3]  = 0x00;  cmd[4]  = 0x00;  cmd[5]  = 0x00;
  cmd[6] = (cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5]) & 0xff;
  Serial2.write(cmd,7);  // Initialize stop command
  delay(100);

  cmd[0]  = 0xFF;  cmd[1]  = 0x20;  cmd[2]  = 0x13;  cmd[3]  = 0x03;  cmd[4]  = 0x00;  cmd[5]  = 0x00;
  cmd[6] = (cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5]) & 0xff;
  Serial2.write(cmd,7);  // Initialize center command
  delay(750);

  cmd[0]  = 0xFF;  cmd[1]  = 0x20;  cmd[2]  = 0x13;  cmd[3]  = 0x02;  cmd[4]  = 0x00;  cmd[5]  = 0x00;
  cmd[6] = (cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5]) & 0xff;
  Serial2.write(cmd,7);  // Initialize center command
  delay(100);

  cmd[0]  = 0xFF;  cmd[1]  = 0x20;  cmd[2]  = 0x00;  cmd[3]  = 0x00;  cmd[4]  = 0x00;  cmd[5]  = 0x00;
  cmd[6] = (cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5]) & 0xff;
  Serial2.write(cmd,7);  // Initialize stop command again

  delay(1000);
  t_new = millis();
}

void loop() {

  // sensor serial bytes reading 
  delay(10);
  Serial2.write(data,7);
  digitalWrite(LED_BUILTIN, HIGH);
  if (Serial2.available() >= 10) { // Wait until at least (10) bytes are available in the buffer
    Serial2.readBytes(uart, 10); // Read 10 ! bytes into the msg array
    
    if (uart[0] == 0xF0 && uart[1] == 0x01 && uart[2] == 0x06) {
      byte check = 0x00; 
      for (i = 1; i < 9; i++) {
      check = check + uart[i];
      }

      if (uart[9] == check){
        pitch_raw = (uart[3] << 8) | uart[4]; // byte to int_16 
        yaw_raw = (uart[7] << 8) | uart[8]; // byte to int_16
        
        pitch_current = pitch_raw;          // int_16 to float 
        pitch_current = pitch_current/-100;
        if (pitch_current < -90) {pitch_current = pitch_current +360;}
        yaw_current = yaw_raw;
        yaw_current = yaw_current/-100;

        if ( ang == 1 && yaw_current > 170){ang = 2;}
          
        if (ang == 2 && yaw_current < 0){
          yaw_current = yaw_current +360;}

        if (ang == 2 && yaw_current < 170){ang =1;}

        if ( ang == 1 && yaw_current < -170){ang = 3;}
          
        if (ang == 3 && yaw_current > 0){
          yaw_current = yaw_current -360;}

        if (ang == 3 && yaw_current > -170){ang =1;}

      }
    }
  }

  // serial command  
  if (Serial.available() > 0){
    myCmd=(Serial.readStringUntil('\r'));

    mode = getValue(myCmd, ',', 0);
    yaw = getValue(myCmd, ',', 1);
    pitch = getValue(myCmd, ',', 2);

    control_mode = mode.toInt();;
    pitch_target = pitch.toFloat();;
    yaw_target = yaw.toFloat();
  }
  
  // controll:

  if ( control_mode > 0 ) {
    pitch_control = abs(pitch_target) * 255;
    yaw_control = abs(yaw_target) * 255;

    pitch_error = pitch_target;
    yaw_error = yaw_target;

  } else if (control_mode < 1){
    t_old = t_new;
    t_new = millis();
    dt = t_new - t_old;

    yaw_old_error = abs(yaw_error);
    pitch_old_error = abs(pitch_error);

    pitch_error = pitch_target - pitch_current;
    yaw_error = yaw_target - yaw_current;

    yaw_d_error = yaw_old_error - abs(yaw_error);
    yaw_i = yaw_d_error/dt;

    pitch_d_error = pitch_old_error - abs(pitch_error);
    pitch_i = pitch_d_error/dt;

    pitch_control = pitch_k1 * abs(pitch_error)+ pitch_k2 * pitch_i;
    yaw_control = yaw_k1 * abs(yaw_error)+ yaw_k2 * yaw_i;
  }
  
  if (pitch_control > 255) {pitch_control = 255;}
  if (yaw_control > 255) {yaw_control = 255;}

  if (abs(yaw_error) >= 0.2 && abs(pitch_error) >= 0.2 ) { 
    thr = 0.2;

  } else if (control_mode < 1){
      thr = 0.05;
      yaw_control = 1;
      pitch_control = 1;
  }

 // pure pitch
  if (abs(pitch_error) >= thr && abs(yaw_error) < thr ) { 
    yaw_control = 0;

    if (pitch_error >= thr) {
      cmd[3] = 0x10; // down command

      } else if (pitch_error <= -thr) {
        cmd[3] = 0x08; // up command
      }

 // pure yaw
  } else if (abs(yaw_error) >= thr && abs(pitch_error) < thr ) { 
    pitch_control = 0;

    if (yaw_error >= thr) {
      cmd[3] = 0x04; // right command

    } else if (yaw_error <= thr) {
        cmd[3] = 0x02; // left command
    }
  }

  if (abs(yaw_error) >= thr && abs(pitch_error) >= thr ) { 

    if (pitch_error >= thr && yaw_error >= thr) {
      cmd[3] = 0x14;

    } else if (pitch_error >= thr && yaw_error <= -thr) {
        cmd[3] = 0x12; 
    }
      else if (pitch_error <= -thr && yaw_error >= thr) {
        cmd[3] = 0x0C;
    }
      else if (pitch_error <= -thr && yaw_error <= -thr){
        cmd[3] = 0x0A;
    }

  }

  cmd[4] = yaw_control; // pitch speed
  cmd[5] = pitch_control; // yaw speed
  
  if (abs(yaw_error) < thr && abs(pitch_error) < thr ) {
    cmd[0]  = 0xFF;  cmd[1] = 0x05;  cmd[3] = 0x00;  cmd[4] = 0x00;  cmd[5] = 0x00;
  }

  cmd[6] = (cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5]) & 0xff; //checksum

  delay(5);
  Serial2.write(cmd,7);
  String serial_data = "!" + String(pitch_current) + "," + String(yaw_current);
  Serial.println(serial_data);
  digitalWrite(LED_BUILTIN, LOW);

    // Serial.print("mode : ");
    // Serial.print(cmd[3]);
    // Serial.print(" / thr : ");
    // Serial.print(thr);

    // Serial.print(" / yaw_error : ");
    // Serial.print(yaw_error);
    // Serial.print(" / pitch_error : ");
    // Serial.print(pitch_error);

    // Serial.print(" / yaw_control : ");
    // Serial.print(cmd[4]);
    // Serial.print(" / pitch_control : ");
    // Serial.println(cmd[5]);



}
// put function definitions here:
String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
