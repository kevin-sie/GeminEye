int audioPin = 11; //Pin outputting tone
int visPin_L = 10; //Pin driving left visual alert
int visPin_R = 9; //Pin driving right visual alert

unsigned int freq = 3750; //Tone frequency, Hz
unsigned long dur = 150; //Tone duration, ms



void setup() {
  //Configure visual driver pins
  pinMode(visPin_L,OUTPUT);
  pinMode(visPin_R,OUTPUT);

  //For debug
  Serial.begin(9600);

}

void loop() {
  //Left blindspot alert
  leftBlindspot();

  delay(2000);
  
  //Right blindspot alert
  rightBlindspot();

  delay(2000);
  
  //Left blindspot alert, fast approaching car
  leftBlindspot_fast();

  delay(2000);

  //Right blindspot alert, fast approaching car
  rightBlindspot_fast();

  delay(2000);

  //Fast approaching car in-lane
  inLane_fast();

  delay(2000);
}

void leftBlindspot(){
  for(int i=0;i<3;i++){
    digitalWrite(visPin_L,HIGH);
    delay(250);
    digitalWrite(visPin_L,LOW);
    delay(250);
  }
  digitalWrite(visPin_L,HIGH);
  delay(1500);
  digitalWrite(visPin_L,LOW);
}

void rightBlindspot(){
  for(int i=0;i<3;i++){
    digitalWrite(visPin_R,HIGH);
    delay(250);
    digitalWrite(visPin_R,LOW);
    delay(250);
  }
  digitalWrite(visPin_R,HIGH);
  delay(1500);
  digitalWrite(visPin_R,LOW);
}

void leftBlindspot_fast(){
  for(int i=0;i<3;i++){
    digitalWrite(visPin_L,HIGH);
    tone(audioPin, freq, dur);
    delay(200);
    digitalWrite(visPin_L,LOW);
    delay(200);
  }
  digitalWrite(visPin_L,HIGH);
  delay(1500);
  digitalWrite(visPin_L,LOW);
}

void rightBlindspot_fast(){
  for(int i=0;i<3;i++){
    digitalWrite(visPin_R,HIGH);
    tone(audioPin, freq, dur);
    delay(200);
    digitalWrite(visPin_R,LOW);
    delay(200);
  }
  digitalWrite(visPin_R,HIGH);
  delay(1500);
  digitalWrite(visPin_R,LOW);
}

void inLane_fast(){
  digitalWrite(visPin_R,HIGH);
  digitalWrite(visPin_L,HIGH);
  
  for(int i=0;i<3;i++){
    tone(audioPin, freq, dur);
    delay(200);
  }
  
  digitalWrite(visPin_R,LOW);
  digitalWrite(visPin_L,LOW);
}

void audioAlert(){
  for(int i=0;i<3;i++){
    tone(audioPin, freq, dur);
    delay(200);
  } 
}
