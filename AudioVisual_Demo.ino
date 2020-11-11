int pin = 6; //Pin outputting tone
unsigned int freq = 3750; //Tone frequency, Hz
unsigned long dur = 5000; //Tone duration, ms


void setup() {
  

}

void loop() {
  alert();
  delay(7000);

}

void alert(){
  for(int i=0;i<3;i++){
    tone(pin, freq, 150);
    delay(200);
  } 
}
