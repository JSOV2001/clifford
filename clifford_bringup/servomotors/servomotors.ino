#include <AX12A.h>
#define DirectionPin  (10u)
#define BaudRate    (1000000ul)
#define ID    (3u)
#define ID2    (13u)
#define ID3   (6u)
#define ID4    (3u)
#define ID5    (11u)
#define ID6   (4u)

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(10); 
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  ax12a.setEndless(ID, OFF);
  ax12a.setEndless(ID2, OFF);
  ax12a.setEndless(ID3, OFF);
  ax12a.setEndless(ID4, OFF);
  ax12a.setEndless(ID5, OFF);
  ax12a.setEndless(ID6, OFF);
}

void loop() {
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil(';');
    Serial.println("Recibido: " + input);
    
    int startIdx = input.indexOf('[') + 1;
    int endIdx = input.indexOf(']');
    String valuesStr = input.substring(startIdx, endIdx);
    Serial.println("Valores: " + valuesStr);

    String valuesArray[6];
    int count = 0;
    while (valuesStr.length() > 0) {
      int commaIdx = valuesStr.indexOf(',');
      if (commaIdx == -1) {
        valuesArray[count] = valuesStr;
        valuesStr = "";
      } else {
        valuesArray[count] = valuesStr.substring(0, commaIdx);
        valuesStr = valuesStr.substring(commaIdx + 1);
      }
      count++;        
    }
    
    //Dynamixel
    int g1 = valuesArray[0].toInt();
    int g2 = valuesArray[1].toInt();
    int g3 = valuesArray[2].toInt();
    int g4 = valuesArray[3].toInt();
    int g5 = valuesArray[4].toInt();
    int g6 = valuesArray[5].toInt();
    ax12a.move(ID, g1);
    ax12a.move(ID2, g2);
    ax12a.move(ID3, g3);
    ax12a.move(ID4, g4);
    ax12a.move(ID5, g5);
    ax12a.move(ID6, g6);
    delay(100);
   }
 }
 
