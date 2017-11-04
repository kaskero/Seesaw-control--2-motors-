#include <LSM6.h>
#include <IMU_v5.h>
#include <CF_lib.h>

LSM6 imu;
IMU_v5 imu_lib;
CF_lib cf_lib;

const int ppm_R_pin = 9, 
          ppm_L_pin = 10;
          
volatile float setpoint = 0.0;
volatile double input = 0.0;

volatile int timer2_overflow = 0;
volatile float t = 0.0;

struct PID_data {
  float Ts = 3;
  double Kp = 1.50,
         Ki = 0.50,
         Kd = 0.75;
  double e_1 = 0.0;
  double iError = 0.0,
         dError = 0.0;
  int outputMin = -50, // OCR1 min = 1100
      outputMax = 50;  // OCR1 max = 1200
};

volatile PID_data myPID;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Entrando en setup()");

  // Configuración e inicialización de los Timers
  Serial.println("1. Configurando la PPM... ");
  long t1 = micros();
  
  pinMode(ppm_R_pin, OUTPUT);  //OC1A como salida del módulo OC para la PWM
  pinMode(ppm_L_pin, OUTPUT);  //OC1B como salida del módulo OC para la PWM
  configureT1_PPM();
  
  long t2 = micros();
  Serial.println("   ...PPM configurada");
  Serial.print("   ...tiempo empleado: "); Serial.print((t2 - t1)/1000.0f, 3); Serial.println("ms");

  // Pulso mínimo para que los motores empiecen a moverse
  delay(1000);
  OCR1A = 1050;
  OCR1B = 1050;

  // Configuración de la IMU
  Serial.println("2. Configurando la IMU... ");
  t1 = micros();
  
  imu_lib.begin(&imu);
  delay(100);
  imu_lib.getOffsetNoise();
  delay(100);
  
  int* accel_data = imu_lib.Read_Acc();
  cf_lib.begin(accel_data);

  t2 = micros();
  Serial.println("   ...IMU configurada");
  Serial.print("   ...tiempo empleado: "); Serial.print((t2 - t1)/1000000.0f, 3); Serial.println("s");

  Serial.println("Introduce la letra 'S' para continuar");
  while(Serial.read() != 'S') {}

  configureT2_Temp();
  
  Serial.println("Saliendo de setup()");
}

void loop() {
  int* accel_data = imu_lib.Read_Acc();
  int* gyro_data = imu_lib.Read_Gyro();
  int* noise_data = imu_lib.Get_Noise();

  input = cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));  
}

ISR(TIMER2_COMPA_vect) {
  if(timer2_overflow == myPID.Ts) {
    //Serial.println((micros()-t)/1000.0, 3);
    //t = micros();

    //long t1 = micros();
    double error = setpoint - input;
    Serial.print(error, 1);
  
    int output = compute_PID(&myPID, error); 
    Serial.print(" "); Serial.println(output);
    // motor offset = 1150
    OCR1A = 1150 + output; 
    OCR1B = 1150 - output; 
    //Serial.print(" "); Serial.print(OCR1A);
    //Serial.print(" "); Serial.println(OCR1B);
  
    myPID.e_1 = error;
    //long t2 = micros();
    //Serial.println((t2-t1)/1000.0, 5);
    
    timer2_overflow = 0;
  } else {
    timer2_overflow++;
  }
}

double compute_PID(PID_data* pid, double e) {
  double pTerm, dTerm, iTerm;
  float Ts = pid->Ts / 1000;

  pTerm = pid->Kp * e;   
  Serial.print(" "); Serial.print(pTerm, 1);

  //pid->iError += e*Ts; // aproximacion rectangular o de Euler
  pid->iError += ((e + pid->e_1) / 2) * Ts; // aproximación trapezoidal o de Tustin
  iTerm = pid->Ki * pid->iError;
  Serial.print(" "); Serial.print(iTerm, 1);

  float dError = e - pid->e_1;
  // EMA LPF to reduce the effect of input noise on dError
  float alpha = 0.75;
  pid->dError = (alpha * dError) + ((1 - alpha) * pid->dError);
  dTerm = pid->Kd * (pid->dError / Ts);
  Serial.print(" "); Serial.print(dTerm, 1);

  double u = pTerm + iTerm + dTerm;
  if(u > pid->outputMax) return pid->outputMax;
  else if(u < pid->outputMin) return pid->outputMin;
  else return round(u);
}

void configureT1_PPM(){
  // CONFIGURACIÓN DEL TIMER1 PARA LA GENERACIÓN DE UNA SEÑAL PWM 
  // CON CORRECCIÓN DE FASE Y FRECUENCIA DE 50Hz
  
  // Table 16-4. Waveform Generation Mode Bit Description
  // Mode||WGM13||WGM12||WGM11||WGM10||Timer/Counter_Mode_of_Operation ||TOP
  //  8  ||  1  ||  0  ||  0  ||  0  ||PWM, Phase and Frequency Correct||ICR1
    
  // 16.11.1 TCCR1A – Timer/Counter1 Control Register A
  // COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
  // COM1A1:0 and COM1B1:0 control the Output Compare pins (OC1A and OC1B) behavior.
  // Table 16-3. Compare Output Mode, Phase Correct and Phase and Frequency Correct PWM
  // COM1A1/COM1B1 COM1A0/COM1B0 Description
  //       1             0       Clear OC1A/OC1B on Compare Match when upcounting.
  //                             Set OC1A/OC1B on Compare Match when downcounting.
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1));
  TCCR1A &= (~(_BV(COM1A0)) & ~(_BV(COM1B0)));
  TCCR1A &= (~(_BV(WGM11)) & ~(_BV(WGM10)));
  
  // 16.11.2 TCCR1B – Timer/Counter1 Control Register B
  // X X - WGM13 WGM12 CS12 CS11 CS10
  TCCR1B |= _BV(WGM13);
  TCCR1B &= ~(_BV(WGM12));
  
  // Table 16-5. Clock Select Bit Description
  // CS12 CS11 CS10 Description
  // 0    1    0    clk/8
  TCCR1B |= _BV(CS11);
  TCCR1B &= (~(_BV(CS12)) & ~(_BV(CS10)));

  ICR1 = 20000; // t_PWM: 2 * (20000 * 0.5us) = 20ms --> 50Hz
  //OCR1A = 1000; // 2 * (1000 * 0.5us) = 1ms
                // hay que multiplicar x2 porque es phase correct

  // Inicializar los motores a cero desde un principio para armarlos: 0 -> 1000ms
  OCR1A = 950;
  OCR1B = 950;
}

void configureT2_Temp(){
  cli(); // Inhabilitar interrupciones

  // CONFIGURACIÓN DEL TIMER2 COMO TEMPORIZADOR 
  // PARA GENERAR UNA INTERRUPCIÓN CADA 1ms

  // Preescalado del reloj 1/1024, con frecuencia del reloj 16MHz
  // t_TCNT2 = 1 / (16MHz / 1024) = 64us
  // Se quiere una interrupción cada 1ms
  // OCR2A · 64us = 1ms -> OCR2A = 15.625 ~ 16
  
  // Table 18-8. Waveform Generation Mode Bit Description
  // Mode||WGM22||WGM21||WGM20||Timer/Counter_Mode_of_Operation||TOP
  //  2  ||  0  ||  1  ||  0  ||            CTC                ||OCR2A

  // 18.11.1 TCCR2A – Timer/Counter Control Register A
  // COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
  // Table 18-2. Compare Output Mode, non-PWM Mode
  // COM2A1 COM2A0 Description
  //    0      0   Normal port operation, OC2A disconnected
  TCCR2A &= (~(_BV(COM2A1)) & ~(_BV(COM2A0)));
  TCCR2A &= ~(_BV(WGM20));
  TCCR2A |= (_BV(WGM21));

  // 18.11.2 TCCR2B – Timer/Counter1 Control Register B
  // X X - - WGM22 CS22 CS21 CS20
  // Table 18-9. Clock Select Bit Description
  // CS22 CS21 CS20 Description
  //  1    1    1   clk/1024
  TCCR2B &= ~(_BV(WGM22));
  TCCR2B |= (_BV(CS22) | _BV(CS21) | _BV(CS20));

  OCR2A = 16; // OCR2A como TOP del contador para interrupción cada 1ms
  TCNT2 = 0;  // Reseteo del registro del contador

  // 18.11.6 TIMSK2 - Timer/Counter2 Interrupt Mask Register
  // - - - - - OCIE2B OCIE2A TOIE2
  TIMSK2 |= 1<<OCIE2A; // Habilitar la interrupción
  
  sei(); // Habilitar interrupciones
}


