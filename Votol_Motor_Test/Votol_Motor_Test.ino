int pot_value ;
int Pot_Pin_num = 14   ;
int PWM_Thr_pin = 2;
void setup() {
  // put your setup code here, to run once:


  pinMode(Pot_Pin_num, INPUT);
  pinMode(PWM_Thr_pin, OUTPUT);


}

void loop() {

  pot_value = analogRead(Pot_Pin_num);
  pot_value = map(pot_value, 0, 600, 60, 200);
  analogWrite(PWM_Thr_pin, pot_value);

}
