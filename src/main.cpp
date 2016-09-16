#include "mbed.h"
#include "TextLCD.h"
#include "Adafruit_PWMServoDriver.h"
#include <cmath>
#include <algorithm>

/// 手動機・自動機共有部分
Serial pc(USBTX, USBRX);

AnalogIn shield_button(A0);
TextLCD lcd(D8, D9, D4, D5, D6, D7);
DigitalOut backlight(D10);
Ticker shield_input_ticker;
class ShieldInput {
  public:
    static bool Up, Down, Right, Left, Select;
};
bool ShieldInput::Up = false, ShieldInput::Down = false,
  ShieldInput::Right = false, ShieldInput::Left = false,
  ShieldInput::Select = false;
void get_shield_input(void) {
  int value = shield_button.read() * 1000;
  ShieldInput::Right = value < 50;
  ShieldInput::Up = 50 <= value && value < 350;
  ShieldInput::Down = 350 <= value && value < 700;
  ShieldInput::Left = 900 <= value && value < 950;
  ShieldInput::Select = 950 <= value;
}
void initialize_shield(void) {
  shield_input_ticker.attach(&get_shield_input, 0.1f);
  backlight.write(1);
  lcd.cls();
}
Ticker backlight_toggler;
void backlight_toggle(void) {
  backlight.write(!backlight.read());
}

DigitalOut buzzer(PD_2);
DigitalIn button(USER_BUTTON);
void initialize_buzzer(void) {
  buzzer.write(0);
}
void buzz(float time) {
  buzzer.write(1);
  wait(time);
  buzzer.write(0);
}
void repeat_buzz(float time, int n) {
  for (int i = 0; i < n; i++) {
    wait(time);
    buzz(time);
  }
}

/*I2C i2c(PB_9, PB_8);
void initialize_i2c(void) {
  i2c.frequency(100000); //100kHz
}
const uint8_t compass_addr = 0x1E << 1;
void compass_write(uint8_t reg, uint8_t data) {
  char command[2] = { reg, data };
  i2c.write(compass_addr, command, 2);
}
volatile int16_t x = 0, z = 0, y = 0;
volatile float radian = 0, degree = 0;
void parse_compass_data(void) {
  char data[6], command[2] = { 0x03 };
  i2c.lock();
  i2c.write(compass_addr, command, 1, true); //point to first data register 03
  i2c.read(compass_addr, data, 6);
  i2c.unlock();
  x = (data[0] << 8) | data[1];
  z = (data[2] << 8) | data[3];
  y = (data[4] << 8) | data[5];
  radian = std::atan2(y, x);
  degree = 180 + radian * 180 / M_PI;
}
Ticker compass_watcher;
void initialize_compass(void) {
  //                    Configuration Register A
  //                    +-------- 0   Reserved
  //                    |++------ 11  Samples averaged -> 8
  //                    |||+++--- 110 Typical Data Output Rate -> 75Hz
  //                    ||||||++- 00  Measurement Mode -> Default
  compass_write(0x00, 0b00011000);
  //                    Configuration Register B
  //                    +++----- 001 Gain Configuration -> 1090
  compass_write(0x01, 0b00100000);
  //                    Mode Register
  //                    +-------- 0  High Speed I2C -> Disabled
  //                    |     ++- 00 Operating Mode -> Continuous-Measurement
  compass_write(0x02, 0b00000000);
  //75Hz -> 13.3ms 15ms秒毎にサンプルするのが妥当?
  compass_watcher.attach(parse_compass_data, 0.015);
}*/
/// 手動機・自動機共有部分
Serial ps3(PA_11, PA_12); // tx, rx
const uint16_t buffer_size = 130;
uint8_t buffer[buffer_size];
volatile uint16_t rx_in = 0;
volatile uint16_t rx_out = 0;
void Rx_interrupt(void) {
  while(ps3.readable()) {
    uint16_t next = (rx_in + 1) % buffer_size;
    if (rx_out == next) {
      pc.printf("\n\r***buffer is full!***\n\r");
      break;
    }
    buffer[rx_in] = ps3.getc();
    rx_in = next;
  }
  return;
}

class Input { public:
    static bool Up, Down, Right, Left, Triangle, Cross, Circle, Square,
      Start, Select, L1, L2, R1, R2;
    static uint8_t LeftX, LeftY, RightX, RightY;
};
bool Input::Up = false, Input::Down = false,
  Input::Right = false, Input::Left = false,
  Input::Triangle = false, Input::Cross = false,
  Input::Circle = false, Input::Square = false,
  Input::L1 = false, Input::L2 = false, Input::R1 = false, Input::R2 = false,
  Input::Start = false, Input::Select = false;

uint8_t Input::LeftX  = 0, Input::LeftY  = 0,
  Input::RightX = 0, Input::RightY = 0;

bool is_set(uint8_t position, uint8_t data) {
  uint8_t bit = 1 << position;
  return (data & bit) == bit;
}

uint8_t data[13];
void parse_input_data(bool debug) {
  __disable_irq();
  for (int i = 0; i < 13; i++) {
    if (rx_in == rx_out) {
      __enable_irq();
      while (rx_in == rx_out);
      __disable_irq();
    }
    data[i] = buffer[rx_out];
    rx_out = (rx_out + 1) % buffer_size;
  }
  __enable_irq();

  bool start_bytes = data[0] == 0x0D && data[1] == 0x00 && data[2] == 0x02 &&
      data[3] == 0x50 && data[4] == 0x03 && data[5] == 0x00;
  if (!start_bytes) {
    lcd.cls();
    lcd.locate(0, 0);
    pc.printf("input: ");
    for(int i = 0; i < 6; i++) {
      lcd.printf("%02X", data[i]);
      pc.printf("%02X ", data[i]);
    }
    lcd.locate(0,1);
    lcd.printf(" skipping...\n\r");
    pc.printf("skipping...\n\r");
    return;
  }

  uint8_t sum = 0;
  for (int i = 0; i < 12; i++) {
    sum += data[i];
  }
  if (data[12] != sum) {
    lcd.cls();
    lcd.locate(0, 0);
    for(int i = 0; i < 6; i++) {
      lcd.printf("%02X", data[6+i]);
      pc.printf("%02X ", data[6+i]);
    }
    lcd.locate(0, 1);
    lcd.printf("NG(ex: %02X ac: %02X)", sum, data[12]);
    pc.printf("NG (expected: %02X actual: %02X)\n\r", sum, data[12]);
    return;
  }

  if (debug) {
    lcd.cls();
    lcd.locate(0, 0);
    pc.printf("debug:");
    for (int i = 0; i < 6; i++) {
      lcd.printf("%02X", data[6+i]);
      pc.printf(" %02X", data[6+i]);
    }
    pc.printf("\n\r");
  }

  Input::Up       = is_set(0, data[7]);
  Input::Down     = is_set(1, data[7]);
  Input::Right    = is_set(2, data[7]);
  Input::Left     = is_set(3, data[7]);

  Input::Triangle = is_set(4, data[7]);
  Input::Cross    = is_set(5, data[7]);
  Input::Circle   = is_set(6, data[7]);
  Input::Square   = is_set(0, data[6]);

  Input::L1       = is_set(1, data[6]);
  Input::L2       = is_set(2, data[6]);
  Input::R1       = is_set(3, data[6]);
  Input::R2       = is_set(4, data[6]);

  Input::Start    = Input::Up && Input::Down;
  Input::Select   = Input::Right && Input::Left;

  Input::LeftY    = data[8];
  Input::LeftX    = data[9];
  Input::RightY   = data[10];
  Input::RightX   = data[11];
}

volatile bool kill_flag = false;
void check_kill_switch(void) {
  if (Input::Select && Input::Start && !kill_flag) {
    kill_flag = true;
    backlight_toggler.attach(&backlight_toggle, 1.0f);
  }
}
Ticker kill_switch_watcher;
void initialize_kill_switch(void) {
  kill_switch_watcher.attach(&check_kill_switch, 0.1f);
}

PwmOut servo[3] = { PwmOut(PB_0), PwmOut(PA_1), PwmOut(PB_7) };
bool is_open[3] = { true, true, true };
float open_close[3][2] = {
  { 1.0f, 0.5f }, // top
  { 1.0f, 0.5f }, // middle
  { 0.5f, 1.0f }  // bottom
};
void set_servo(PwmOut s, float percent) {
  s.pulsewidth_us(percent * (1800-500) + 500);
}
void apply_servo(void) {
  for (int i = 0; i < 3; i++) {
    set_servo(servo[i], open_close[i][!is_open[i]]);
  }
}
void initialize_servo(void) {
  for (int i = 0; i < 3; i++) {
    servo[i].period_ms(20);
  }
  apply_servo();
}

Adafruit_PWMServoDriver pwm(PB_9, PB_8, 0x40); //sda, scl, addr
void initialize_pwm(void) {
  pwm.begin();
  pwm.setPWMFreq(50.0f); //20ms -> 50Hz -> これの周期を調べる
}
void set_pwm(uint8_t n, uint16_t v) {
  switch(v) {
    case 4095:
      pwm.setPWM(n, 4096, 0);
      break;
    case 0:
      pwm.setPWM(n, 0, 4096);
      break;
    default:
      pwm.setPWM(n, 0, v);
      break;
  }
}
void set_pwm_with_float(uint8_t n, float p) {
  set_pwm(n, std::floor(4095*p+0.5f));
  //std::round only available from c++11 :(((
}
uint8_t motor[4][2] = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 } };

/*PwmOut motor[4][2] = {
  { PwmOut(PC_8), PwmOut(PC_6) },
  { PwmOut(PA_5), PwmOut(PB_9) },
  { PwmOut(PC_9), PwmOut(PB_8) },
  { PwmOut(PB_13), PwmOut(PA_10) }
};*/
void initialize_motor(void) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
//      motor[i][j].period_ms(20);
      //adjust and test around
      //40us ~ 2ms
      set_pwm_with_float(motor[i][j], 0.0f);
//      motor[i][j].write(0.0);
    }
  }
}
float normalize(float num) {
  return std::max(-1.0f, std::min(1.0f, num));
}
void set_motor(uint8_t in[2], float power) {
//  in[0].write((power > 0.0) ? power : 0.0f);
//  in[1].write((power > 0.0) ? 0.0f : -power);
  set_pwm_with_float(in[0], (power > 0.0) ? power : 0.0f);
  set_pwm_with_float(in[1], (power > 0.0) ? 0.0f : -power);
}
void release_motor(uint8_t in[2]) {
  set_motor(in, 0.0);
}
//const float approach_constant = 10.0f;
//const float approach_constant = 1.0f;
//void approach_motor(uint8_t in[2], float target) {
//  //float a = in[0].read(), b = in[1].read();
//  float now = (a == 0.0f) ? -b : a;
//  float value = normalize(now + ((target - now) / approach_constant));
//  set_motor(in, value);
//}
enum Direction { Clockwise, CounterClockwise, Stop };
void set_motor_with_direction(uint8_t in[2], Direction d, float percent) {
  if (d == Stop) {
    release_motor(in);
  } else {
    set_motor(in, (d == Clockwise) ? percent : -percent);
  }
}
//void approach_motor_with_direction(uint8_t in[2], Direction d, float percent) {
//  if (d == Stop) {
//    approach_motor(in, 0.0);
//  } else {
//    approach_motor(in,(d == Clockwise) ? percent : -percent);
//  }
//}

//motor[0] -> front left
//motor[1] -> back left
//motor[2] -> front right
//motor[3] -> back right
Direction forward[4] = {
  Clockwise, Clockwise, CounterClockwise, CounterClockwise
};
Direction backward[4] = {
  CounterClockwise, CounterClockwise, Clockwise, Clockwise
};
Direction right[4] = {
  Clockwise, CounterClockwise, Clockwise, CounterClockwise
};
Direction left[4] = {
  CounterClockwise, Clockwise, CounterClockwise, Clockwise
};
Direction right_forward[4] = {
  Clockwise, Stop, Stop, CounterClockwise
};
Direction right_backward[4] = {
  Stop, CounterClockwise, Clockwise, Stop
};
Direction left_forward[4] = {
  Stop, Clockwise, CounterClockwise, Stop
};
Direction left_backward[4] = {
  CounterClockwise, Stop, Stop, Clockwise
};
Direction clockwise[4] = {
  Clockwise, Clockwise, Clockwise, Clockwise
};
Direction counter_clockwise[4] = {
  CounterClockwise, CounterClockwise, CounterClockwise, CounterClockwise
};
void set_motor_with_cartesian(float x, float y) {
//void move(float x, float y) {
  //old implementation, moved to set_motor
  //float a = x+y, b = -x+y;
  //float pa = normalize(std::abs(a)), pb = normalize(std::abs(b));
  //set_motor_with_direction(motor[0], a > 0.0 ? Clockwise : CounterClockwise, pa);
  //set_motor_with_direction(motor[3], a > 0.0 ? CounterClockwise : Clockwise, pa);
  //set_motor_with_direction(motor[1], b > 0.0 ? Clockwise : CounterClockwise, pb);
  //set_motor_with_direction(motor[2], b > 0.0 ? CounterClockwise : Clockwise, pb);
  float pa = normalize(x+y), pb = normalize(-x+y);
  set_motor(motor[0], pa);
  set_motor(motor[3], -pa);
  set_motor(motor[1], pb);
  set_motor(motor[2], -pb);
}
void drive(float power, bool debug) {
  if (Input::LeftX != 0x80 || Input::LeftY != 0x80) {
    float x =
      Input::LeftX < 0x80 ?
      (-((float)(0x80-Input::LeftX))/0x80) :
      (((float)(Input::LeftX-0x80))/0x7F);
    float y =
      Input::LeftY < 0x80 ?
      (-((float)(0x80-Input::LeftY))/0x80) :
      (((float)(Input::LeftY-0x80))/0x7F);
    set_motor_with_cartesian(x, y);
  } else if (Input::Up) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], forward[i], power);
    }
  } else if (Input::Down) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], backward[i], power);
    }
  } else if (Input::Right) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], right[i], power);
    }
  } else if (Input::Left) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], left[i], power);
    }
  } else if (Input::Triangle) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], right_forward[i], power);
    }
  } else if (Input::Cross) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], left_backward[i], power);
    }
  } else if (Input::Circle) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], right_backward[i], power);
    }
  } else if (Input::Square) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], left_forward[i], power);
    }
  } else if (Input::R1) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], clockwise[i], power);
    }
  } else if (Input::L1) {
    for (int i = 0; i < 4; i++) {
      set_motor_with_direction(motor[i], counter_clockwise[i], power);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      release_motor(motor[i]);
      //set_motor_with_direction(motor[i], Stop, 0.0f);
    }
  }
}

void test_servo(void) {
  /*if (!button) {
    for (int i = 0; i < 3; i++) {
      set_servo(servo[i], 1.0f);
    }
    wait(1.0);
  } else {
    for (int i = 0; i < 3; i++) {
      set_servo(servo[i], 0.0f);
    }
    wait(1.0);
  }*/
  float pw = 1.0f;
  while(true) {
    wait(0.1);
    if (ShieldInput::Right) pw = min(1.0, pw+0.1);
    else if (ShieldInput::Left) pw = max(0.0, pw-0.1);
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("pw: %f", pw);
    set_servo(servo[0], pw);
  }
}

void test_compass(void) {
}

void initialize_io(void) {
  pc.baud(19200);
  ps3.baud(2400);
  ps3.format(8, SerialBase::Even, 1);
  ps3.attach(&Rx_interrupt, Serial::RxIrq);
  initialize_buzzer();
  initialize_shield();
  initialize_kill_switch();
  initialize_servo();
//  initialize_motor();
//  initialize_i2c();
  initialize_pwm();
//  initialize_compass();

  lcd.cls();
  wait(0.5);
  lcd.locate(0, 0);
  lcd.printf("Initialized.");
  pc.printf("Initialized.\n\r");
  wait(1.0);
  lcd.cls();

  repeat_buzz(0.05, 2);
}
void kill(void) {
  lcd.cls();
  lcd.locate(0, 0);
  lcd.printf("killed.");
}
int main() {
  initialize_io();
  //test_servo();
  while(true) {
    if (kill_flag) break;
    //wait(0.2);
//    parse_compass_data();
    //lcd.cls();
    //lcd.locate(0, 0);
    //lcd.printf("x: %d y: %d", x, y);
    //lcd.printf("r: %f", radian);
    //lcd.locate(0, 1);
    //lcd.printf("z: %d", z);
    //lcd.printf("d: %f", degree);
    //pc.printf("%d, %d\n\r", x, y);
    parse_input_data(false); //test_servo();
    drive(1.0, false);
  }
  kill();
  while(true);
}

