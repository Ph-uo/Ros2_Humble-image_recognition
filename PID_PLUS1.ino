// bài sau có  khai báo, điều khiển tốc độ động cơ 1 chiều cho 4 bánh sử dụng chung 1 thông số cho bộ điều khiển pid
// có giới hạn mức điện áp dầu ra khoảng 10v
// có thêm chương trình đọc serial có tách dấu phẩy, có thể chương trình chính nằm trên python

//#include <Wire.h> // Library for I
float kp = 0.02;    //dat gia tri cho Kp
float ki = 0.0001;   //dat gia tri cho Ki
float kd = 0.5;   // dat gia tri cho Kd

unsigned long t_now;
unsigned long t_prev = 0;

const byte interruptPinA = 18; //ngat mt1
const byte interruptPinB = 19; //ngat mt2
const byte interruptPinC = 20; //ngat mt3
const byte interruptPinD = 21; //ngat mt4

//biendiem xung
volatile long EncoderCount = 0;
volatile long EncoderCount2 = 0;
volatile long EncoderCount3 = 0;
volatile long EncoderCount4 = 0;

const byte DirPin1 = 22;       //Motor1
const byte DirPin2 = 23;
const byte PWMPin = 3;

const byte DirPin2_1 = 24;    //Motor2
const byte DirPin2_2 = 25;
const byte PWMPin2 = 4;

const byte DirPin3_1 = 26;    //Motor3
const byte DirPin3_2 = 27;
const byte PWMPin3 = 5;

const byte DirPin4_1 = 28;    //Motor4
const byte DirPin4_2 = 29;
const byte PWMPin4 = 6;

int PWMval=0; //gia tri ban dau cua xung
int PWMval2=0; //gia tri ban dau cua xung
int PWMval3=0; //gia tri ban dau cua xung
int PWMval4=0; //gia tri ban dau cua xung

volatile unsigned long count = 0;
unsigned long count_prev = 0;

// Theta là vị trí góc
// Theta_now và Theta_prev đều cần thiết để tính RPM (vòng mỗi phút)
// RPM có thể được tính là sự khác biệt về vị trí theo thời gian
float Theta_now; 
float Theta_prev = 0;

float Theta_now2; 
float Theta_prev2 = 0;

float Theta_now3; 
float Theta_prev3 = 0;

float Theta_now4; 
float Theta_prev4 = 0;

//RPM_input là RPM đầu vào của người dùng (giá trị đặt)
//RPM_output là RPM đầu ra của động cơ được đo bằng bộ mã hóa
float RPM_output, RPM_input=0;
float RPM_output2, RPM_input2=0;
float RPM_output3, RPM_input3=0;
float RPM_output4, RPM_input4=0;

int dt;                      // Khoảng thời gian dùng để tính RPM

//******************************MISC VARIABLES*****************
//Maximum motor voltage in clockwise rotation
float Vmax = 10;      
float Vmin = 0;
float V = 0; // đặt điện áp ban đầu về 0
float V2 = 0;
float V3 = 0; // đặt điện áp ban đầu về 0
float V4 = 0;
// Tín hiệu lỗi và thuật ngữ PID (Đạo hàm tích phân theo tỷ lệ)
float error_now, error_prev = 0, integ_now, integ_prev = 0;
float error_now2, error_prev2 = 0, integ_now2, integ_prev2 = 0;
float error_now3, error_prev3 = 0, integ_now3, integ_prev3 = 0;
float error_now4, error_prev4 = 0, integ_now4, integ_prev4 = 0;

void ISR_EncoderA() {
  EncoderCount++;
}

void ISR_EncoderB() {
  EncoderCount2++;
}

void ISR_EncoderC() {
  EncoderCount3++;
}

void ISR_EncoderD() {
  EncoderCount4++;
}

void WriteDriverVoltage(float V,float V2,float V3,float V4, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);  //Motor1
  if (PWMval > 255) {
    PWMval = 255;
  }

  int PWMval2 = int(255 * abs(V2) / Vmax);  //Motor2
  if (PWMval2 > 255) {
    PWMval2 = 255;
  }

  int PWMval3 = int(255 * abs(V3) / Vmax);  //Motor3
  if (PWMval3 > 255) {
    PWMval3 = 255;
  }

  int PWMval4 = int(255 * abs(V4) / Vmax);  //Motor4
  if (PWMval4 > 255) {
    PWMval4 = 255;
  }

  analogWrite(PWMPin,PWMval);
  analogWrite(PWMPin2,PWMval2);
  analogWrite(PWMPin3,PWMval3);
  analogWrite(PWMPin4,PWMval4);
}

ISR(TIMER1_COMPA_vect) {
  count++;
}
void setup() {
  //General setup

  Serial.begin(115200);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(interruptPinC, INPUT_PULLUP);
  pinMode(interruptPinD, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinC), ISR_EncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinD), ISR_EncoderD, CHANGE);

  RPM_input = 0;       // toc do dat vao
  RPM_input2 = 0;
  RPM_input3 = 0;       // toc do dat vao
  RPM_input4 = 0;


  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  digitalWrite(DirPin1,HIGH);
  digitalWrite(DirPin2,LOW);

  pinMode(DirPin2_1, OUTPUT);
  pinMode(DirPin2_2, OUTPUT);
  digitalWrite(DirPin2_1,HIGH);
  digitalWrite(DirPin2_2,LOW);

  pinMode(DirPin3_1, OUTPUT);
  pinMode(DirPin3_2, OUTPUT);
  digitalWrite(DirPin3_1,1);
  digitalWrite(DirPin3_2,0);

  pinMode(DirPin4_1, OUTPUT);
  pinMode(DirPin4_2, OUTPUT);
  digitalWrite(DirPin4_1,1);
  digitalWrite(DirPin4_2,0);
  

//ngat cho arduino
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  //OCR1A = 2500; //Prescaler = 64  , 10ms
  OCR1A = 6250; //Prescaler = 64  , 25ms
  //OCR1A = 12500; //Prescaler = 64  , 50ms
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void loop() {
  if (count > count_prev) {
    t_now = millis();
    Theta_now = (EncoderCount / 230.4);
    Theta_now2 = (EncoderCount2 / 230.4);
    Theta_now3 = (EncoderCount3 / 230.4);
    Theta_now4 = (EncoderCount4 / 230.4);
    dt = (t_now - t_prev);

//************ TÍNH TOÁN LỖI VÀ PWM *********************
//************ TÍNH TOÁN ĐIỀU KHIEN ĐỘNG CƠ PID ******************
    RPM_output = ((Theta_now - Theta_prev) / (dt / 1000.0) * 60)/2;
    error_now = RPM_input - RPM_output;
    integ_now =  integ_prev+ (dt * (error_now + error_prev) / 2);

    RPM_output2 = ((Theta_now2 - Theta_prev2) / (dt / 1000.0) * 60)/2;
    error_now2 = RPM_input2 - RPM_output2;
    integ_now2 =  integ_prev2+ (dt * (error_now2 + error_prev2) / 2);

    RPM_output3 = ((Theta_now3 - Theta_prev3) / (dt / 1000.0) * 60)/2;
    error_now3 = RPM_input3 - RPM_output3;
    integ_now3 =  integ_prev3+ (dt * (error_now3 + error_prev3) / 2);

    RPM_output4 = ((Theta_now4 - Theta_prev4) / (dt / 1000.0) * 60)/2;
    error_now4 = RPM_input4 - RPM_output4;
    integ_now4 =  integ_prev4+ (dt * (error_now4 + error_prev4) / 2);

//  tính toán điện áp Động cơ hoặc đầu ra bộ điều khiển PID
    V  = kp * error_now + ki * integ_now + (kd * (error_now - error_prev) / dt) ;
    V2 = kp * error_now2 + ki * integ_now2 + (kd * (error_now2 - error_prev2) / dt) ;
    V3 = kp * error_now3 + ki * integ_now3 + (kd * (error_now3 - error_prev2) / dt) ;
    V4 = kp * error_now4 + ki * integ_now4 + (kd * (error_now4 - error_prev4) / dt) ;

    if (V > Vmax) {
      V = Vmax;
      integ_now = integ_prev;
    }
    if (V < Vmin) {
      V = 0;
    }

    if (V2 > Vmax) {
      V2 = Vmax;
      integ_now2 = integ_prev2;
    }
    if (V2 < Vmin) {
      V2 = 0;
    }

    if (V3 > Vmax) {
      V3 = Vmax;
      integ_now3 = integ_prev3;
    }
    if (V3 < Vmin) {
      V3 = 0;
    }

    if (V4 > Vmax) {
      V4 = Vmax;
      integ_now4 = integ_prev4;
    }
    if (V4 < Vmin) {
      V4 = 0;
    }
    
    WriteDriverVoltage(V,V2,V3,V4, Vmax);      
    //Serial.println(V2);

    Theta_prev = Theta_now;
    count_prev = count;
    t_prev = t_now;
    integ_prev = integ_now;
    error_prev = error_now;

    Theta_prev2 = Theta_now2;
    integ_prev2 = integ_now2;
    error_prev2 = error_now2;

    Theta_prev3 = Theta_now3;
    integ_prev3 = integ_now3;
    error_prev3 = error_now3;

    Theta_prev4 = Theta_now4;
    integ_prev4 = integ_now4;
    error_prev4 = error_now4;
  }

  if (Serial.available() >= 0) {
    String input = Serial.readStringUntil('\n');
    
    // Tìm vị trí của các ký tự phẩy
    int firstCommaPos = input.indexOf(',');
    int secondCommaPos = input.indexOf(',', firstCommaPos + 1);
    int thirdCommaPos = input.indexOf(',', secondCommaPos + 1);
    int fourthCommaPos = input.indexOf(',', thirdCommaPos + 1);
    
    if (firstCommaPos > 0 && secondCommaPos > 0 && thirdCommaPos > 0 && fourthCommaPos > 0) {
      // Trích xuất các giá trị
      String command = input.substring(0, firstCommaPos);
      int value1 = input.substring(firstCommaPos + 1, secondCommaPos).toInt();
      int value2 = input.substring(secondCommaPos + 1, thirdCommaPos).toInt();
      int value3 = input.substring(thirdCommaPos + 1, fourthCommaPos).toInt();
      int value4 = input.substring(fourthCommaPos + 1).toInt();

      if (command == "FORWARD") {
        Serial.println(" 3 ");
        RPM_input = value1;       // toc do dat vao
        RPM_input2 = value2;
        RPM_input3 = value3;       // toc do dat vao
        RPM_input4 = value4;

        digitalWrite(DirPin1,HIGH);
        digitalWrite(DirPin2,LOW);

        digitalWrite(DirPin2_1,HIGH);
        digitalWrite(DirPin2_2,LOW);

        digitalWrite(DirPin3_1,1);
        digitalWrite(DirPin3_2,0);

        digitalWrite(DirPin4_1,1);
        digitalWrite(DirPin4_2,0);
      }
    
      if (command == "BACKWARD") {
        Serial.println(" 4 ");
        RPM_input = value1;       // toc do dat vao
        RPM_input2 = value2;
        RPM_input3 = value3;       // toc do dat vao
        RPM_input4 = value4;

        digitalWrite(DirPin1,0);
        digitalWrite(DirPin2,1);

        digitalWrite(DirPin2_1,0);
        digitalWrite(DirPin2_2,1);

        digitalWrite(DirPin3_1,0);
        digitalWrite(DirPin3_2,1);

        digitalWrite(DirPin4_1,0);
        digitalWrite(DirPin4_2,1);
      }
    }
  }

}
