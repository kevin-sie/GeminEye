//This may honestly be some of the worst code I have ever written in my life

int incomingByte = 0; //Stores each incoming byte
double speedMPH = 0;
char readSpeed[4] = "0000"; //Stores the latest speed reading, knots

void setup() 
{
    Serial.begin(4800); // Set the baud rate to 9600
     
}

void receiveByte()
{
  int recvd = 0;

  while(!recvd){
    //Check for incoming data
    if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    recvd = 1;
    }
  }
}

void loop() 
{
  receiveByte();
  
   //Check if incoming byte is a '$'
   if (incomingByte == 36){
    //If the incoming byte is a '$' print it and look at the next one
    //Serial.println(incomingByte);
    receiveByte();

    //Check if incoming byte is a 'G'
    if(incomingByte == 71){
      //If the incoming byte is a 'G' print it and look at the next one
      //Serial.println(incomingByte);
      receiveByte();

      //Check if the incoming byte is a 'P'
      if(incomingByte == 80){
      //If the incoming byte is a 'P' print it and look at the next one
      //Serial.println(incomingByte);
      receiveByte();

        //Check if the incoming byte is a 'R'
        if(incomingByte == 82){
        //If the incoming byte is a 'R' print it and look at the next one
        //Serial.println(incomingByte);
        receiveByte();

          //Check if the incoming byte is a 'M'
          if(incomingByte == 77){
          //If the incoming byte is a 'M' print it and look at the next one
          //Serial.println(incomingByte);
          receiveByte();

            //Check if the incoming byte is a 'C'
           if(incomingByte == 67){
            //If the incoming byte is a 'C' print it and look at the next one
            //Serial.println(incomingByte);
            
            
            //At this point we know we have "$GPRMC", so now throw away 38 bytes
            int j = 0;
            while(j < 39){
              j++;
              receiveByte();
            }

            //The next 4 bytes are the speed
            int x = Serial.readBytes(readSpeed,4);
            String readSpeedStr(readSpeed);
            Serial.print(readSpeedStr.toFloat()*1.15077945);
            Serial.println(" MPH");

           }
          }
        }
      }
    }
  } 
} 
