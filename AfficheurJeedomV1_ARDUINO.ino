#include <FastLED.h>
#define NUM_LEDS  100
#define LED_PIN   7
CRGB leds[NUM_LEDS];

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2; 
const int butgarage = 3;// the number of the pushbutton pin
const int butalarme = 4;
const int butporte = 5;
const int butpetit = 6;


const int ledPin =  13;      // the number of the LED pin


// variables will change:
int bgarage = 0;
int balarme =0;
int buttonState = 0;
int Tportail = 0;// variable for reading the pushbutton status
int TFportail = 502;
int bporte = 0;
int bpetit = 0;
int talarme = 0;
int tporte = 1;
int tgarage=0;

void setup() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(butgarage, INPUT);
  pinMode(butalarme, INPUT);
  pinMode (butporte, INPUT);
  pinMode (butpetit, INPUT);
  leds[4] = CRGB::Black; 
  leds[12] = CRGB::Black; 
  leds[20] = CRGB::Black; 
  leds[91] = CRGB::Black; leds[89] = CRGB::Black;leds[87] = CRGB::Black; leds[85] = CRGB::Black; FastLED.show();
  leds[28] = CRGB::Black; FastLED.show(); 
  Serial.begin(9600);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  bgarage = digitalRead(butgarage);
  balarme = digitalRead(butalarme);
  bporte = digitalRead(butporte);
  bpetit = digitalRead(butpetit);
  

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH && Tportail < 100) {
    FastLED.setBrightness(50);

leds[90] = CRGB::Red; FastLED.show(); delay(100);
leds[81] = CRGB::Red; FastLED.show(); delay(100);
leds[68] = CRGB::Red; FastLED.show(); delay(100);
leds[50] = CRGB::Red; FastLED.show(); delay(100);
leds[24] = CRGB::Red; FastLED.show(); delay(100);
leds[25] = CRGB::Red; FastLED.show(); 
leds[23] = CRGB::Red; FastLED.show(); delay(100);
leds[22] = CRGB::Red; FastLED.show(); 
leds[26] = CRGB::Red; FastLED.show();




  }

  if (buttonState == HIGH && Tportail < 100) {
         Serial.print(" ici ");
     Tportail = Tportail+1;
    Serial.print(Tportail);
    TFportail = 500;
  }
  
  if (buttonState == HIGH && Tportail >= 100) {
    Tportail = Tportail+1;
    Serial.print(Tportail);
    
    digitalWrite(ledPin, LOW);
leds[90] = CRGB::Black; 
leds[81] = CRGB::Black; 
leds[68] = CRGB::Black; 
leds[50] = CRGB::Black; 
leds[24] = CRGB::Black; 
leds[25] = CRGB::Black; 
leds[23] = CRGB::Black; 
leds[22] = CRGB::Black; 
leds[26] = CRGB::Black; 
    FastLED.show();
    delay(500);
    digitalWrite(ledPin, HIGH);
leds[90] = CRGB::Red; 
leds[81] = CRGB::Red; 
leds[68] = CRGB::Red; 
leds[50] = CRGB::Red; 
leds[24] = CRGB::Red; 
leds[25] = CRGB::Red; 
leds[23] = CRGB::Red; 
leds[22] = CRGB::Red; 
leds[26] = CRGB::Red; 
    FastLED.show();
    delay(500);
        digitalWrite(ledPin, LOW);
leds[90] = CRGB::Black; 
leds[81] = CRGB::Black; 
leds[68] = CRGB::Black; 
leds[50] = CRGB::Black; 
leds[24] = CRGB::Black; 
leds[25] = CRGB::Black; 
leds[23] = CRGB::Black; 
leds[22] = CRGB::Black; 
leds[26] = CRGB::Black; 
    FastLED.show();
    delay(500);
    digitalWrite(ledPin, HIGH);
leds[90] = CRGB::Red; 
leds[81] = CRGB::Red; 
leds[68] = CRGB::Red; 
leds[50] = CRGB::Red; 
leds[24] = CRGB::Red; 
leds[25] = CRGB::Red; 
leds[23] = CRGB::Red; 
leds[22] = CRGB::Red; 
leds[26] = CRGB::Red; 
    FastLED.show();
    delay(500);
     TFportail = 500;

  
  }
 
 if (buttonState == LOW && TFportail == 500) {
  TFportail = 100;
  Tportail = 0;

    FastLED.show();
leds[90] = CRGB::Green; FastLED.show(); delay(100);
leds[81] = CRGB::Green; FastLED.show(); delay(100);
leds[68] = CRGB::Green; FastLED.show(); delay(100);
leds[50] = CRGB::Green; FastLED.show(); delay(100);
leds[24] = CRGB::Green; FastLED.show(); delay(100);
leds[25] = CRGB::Green; FastLED.show(); 
leds[23] = CRGB::Green; FastLED.show(); delay(100);
leds[22] = CRGB::Green; FastLED.show(); 
leds[26] = CRGB::Green; FastLED.show(); 
Tportail = 0;
  
 }
 if (buttonState == LOW && TFportail < 499) {


  TFportail = TFportail-1;
  Serial.print(" ici 456 ");
 
 }
 
 if (buttonState == LOW && TFportail < 1) {
  
  leds[90] = CRGB::Black; FastLED.show(); delay(100);
leds[81] = CRGB::Black; FastLED.show(); delay(100);
leds[68] = CRGB::Black; FastLED.show(); delay(100);
leds[50] = CRGB::Black; FastLED.show(); delay(100);
leds[24] = CRGB::Black; FastLED.show(); delay(100);
leds[25] = CRGB::Black; FastLED.show(); 
leds[23] = CRGB::Black; FastLED.show(); delay(100);
leds[22] = CRGB::Black; FastLED.show(); 
leds[26] = CRGB::Black; FastLED.show(); 
TFportail = 501;

  
 }

 
if (bgarage == HIGH) {
  FastLED.setBrightness(50);
leds[86] = CRGB::Red; FastLED.show(); delay(100);
leds[75] = CRGB::Red; FastLED.show(); delay(100);
leds[60] = CRGB::Red; FastLED.show(); delay(100);
leds[38] = CRGB::Red; FastLED.show(); delay(100);
leds[8] = CRGB::Red; FastLED.show(); delay(100);
leds[7] = CRGB::Red; FastLED.show(); 
leds[9] = CRGB::Red; FastLED.show(); delay(100);
leds[6] = CRGB::Red; FastLED.show(); 
leds[10] = CRGB::Red; FastLED.show(); 

tgarage=1;

   
     
}
if (bgarage == LOW && tgarage == 1) {

  tgarage=0;
  
  
leds[86] = CRGB::Green; FastLED.show(); delay(100);
leds[75] = CRGB::Green; FastLED.show(); delay(100);
leds[60] = CRGB::Green; FastLED.show(); delay(100);
leds[38] = CRGB::Green; FastLED.show(); delay(100);
leds[8] = CRGB::Green; FastLED.show(); delay(100);
leds[7] = CRGB::Green; FastLED.show(); 
leds[9] = CRGB::Green; FastLED.show(); delay(100);
leds[6] = CRGB::Green; FastLED.show(); 
leds[10] = CRGB::Green; FastLED.show(); delay(5000);


  
leds[86] = CRGB::Black; FastLED.show(); delay(100);
leds[75] = CRGB::Black; FastLED.show(); delay(100);
leds[60] = CRGB::Black; FastLED.show(); delay(100);
leds[38] = CRGB::Black; FastLED.show(); delay(100);
leds[8] = CRGB::Black; FastLED.show(); delay(100);
leds[7] = CRGB::Black; FastLED.show(); 
leds[9] = CRGB::Black; FastLED.show(); delay(100);
leds[6] = CRGB::Black; FastLED.show(); 
leds[10] = CRGB::Black; FastLED.show(); 


}    

if (balarme == HIGH) {

leds[84] = CRGB::Blue; FastLED.show(); delay(100);
leds[72] = CRGB::Blue; FastLED.show(); delay(100);
leds[56] = CRGB::Blue; FastLED.show(); delay(100);
leds[32] = CRGB::Blue; FastLED.show(); delay(100);
leds[0] = CRGB::Blue; FastLED.show(); delay(100);
leds[1] = CRGB::Blue; FastLED.show(); 
leds[31] = CRGB::Blue; FastLED.show(); delay(100);
leds[2] = CRGB::Blue; FastLED.show(); 
leds[30] = CRGB::Blue; FastLED.show(); 
talarme= 35;


}

if (balarme == LOW && talarme > 1) {
  leds[84] = CRGB::Green; FastLED.show(); delay(100);
leds[72] = CRGB::Green; FastLED.show(); delay(100);
leds[56] = CRGB::Green; FastLED.show(); delay(100);
leds[32] = CRGB::Green; FastLED.show(); delay(100);
leds[0] = CRGB::Green; FastLED.show(); delay(100);
leds[1] = CRGB::Green; FastLED.show(); 
leds[31] = CRGB::Green; FastLED.show(); delay(100);
leds[2] = CRGB::Green; FastLED.show(); 
leds[30] = CRGB::Green; FastLED.show(); 

talarme= talarme-1;

}


if (balarme == LOW && talarme <= 1) {
  leds[84] = CRGB::Black; FastLED.show(); delay(100);
leds[72] = CRGB::Black; FastLED.show(); delay(100);
leds[56] = CRGB::Black; FastLED.show(); delay(100);
leds[32] = CRGB::Black; FastLED.show(); delay(100);
leds[0] = CRGB::Black; FastLED.show(); delay(100);
leds[1] = CRGB::Black; FastLED.show(); 
leds[31] = CRGB::Black; FastLED.show(); delay(100);
leds[2] = CRGB::Black; FastLED.show(); 
leds[30] = CRGB::Black; FastLED.show(); 

}

if (bporte == HIGH) {
  tporte = 1; 
  FastLED.setBrightness(50);
leds[88] = CRGB::Red; FastLED.show(); delay(100);
leds[78] = CRGB::Red; FastLED.show(); delay(100);
leds[64] = CRGB::Red; FastLED.show(); delay(100);
leds[44] = CRGB::Red; FastLED.show(); delay(100);
leds[16] = CRGB::Red; FastLED.show(); delay(100);
leds[15] = CRGB::Red; FastLED.show(); 
leds[17] = CRGB::Red; FastLED.show(); delay(100);
leds[14] = CRGB::Red; FastLED.show(); 
leds[18] = CRGB::Red; FastLED.show(); 


  
}

if (bporte == LOW && tporte == 1) {

tporte = 0; 
leds[88] = CRGB::Green; FastLED.show(); delay(100);
leds[78] = CRGB::Green; FastLED.show(); delay(100);
leds[64] = CRGB::Green; FastLED.show(); delay(100);
leds[44] = CRGB::Green; FastLED.show(); delay(100);
leds[16] = CRGB::Green; FastLED.show(); delay(100);
leds[15] = CRGB::Green; FastLED.show(); 
leds[17] = CRGB::Green; FastLED.show(); delay(100);
leds[14] = CRGB::Green; FastLED.show(); 
leds[18] = CRGB::Green; FastLED.show(); delay(1000);



  
leds[88] = CRGB::Black; FastLED.show(); delay(100);
leds[78] = CRGB::Black; FastLED.show(); delay(100);
leds[64] = CRGB::Black; FastLED.show(); delay(100);
leds[44] = CRGB::Black; FastLED.show(); delay(100);
leds[16] = CRGB::Black; FastLED.show(); delay(100);
leds[15] = CRGB::Black; FastLED.show(); 
leds[17] = CRGB::Black; FastLED.show(); delay(100);
leds[14] = CRGB::Black; FastLED.show(); 
leds[18] = CRGB::Black; FastLED.show(); 
 
  }
if (balarme == HIGH && bporte == HIGH){
leds[92] = CRGB::Yellow;



leds[27] = CRGB::Yellow; 
leds[28] = CRGB::Yellow; 
leds[29] = CRGB::Yellow; 
leds[91] = CRGB::Yellow; 

leds[83] = CRGB::Yellow; 

leds[70] = CRGB::Yellow; 
leds[71] = CRGB::Yellow; 
leds[51] = CRGB::Yellow; 
leds[52] = CRGB::Yellow; 
leds[53] = CRGB::Yellow; 
leds[54] = CRGB::Yellow; 
leds[55] = CRGB::Yellow; FastLED.show(); delay(500);


leds[27] = CRGB::Black; 
leds[28] = CRGB::Black; 
leds[29] = CRGB::Black; 
leds[91] = CRGB::Black; 

leds[83] = CRGB::Black; 

leds[70] = CRGB::Black; 
leds[71] = CRGB::Black; 
leds[51] = CRGB::Black; 
leds[52] = CRGB::Black; 
leds[53] = CRGB::Black; 
leds[54] = CRGB::Black; 
leds[55] = CRGB::Black; 



leds[3] = CRGB::Yellow; 
leds[4] = CRGB::Yellow; 
leds[5] = CRGB::Yellow;
leds[85] = CRGB::Yellow; 
leds[73] = CRGB::Yellow; 
leds[74] = CRGB::Yellow; 
leds[59] = CRGB::Yellow; 
leds[57] = CRGB::Yellow; 
leds[58] = CRGB::Yellow; 
leds[33] = CRGB::Yellow; 
leds[34] = CRGB::Yellow; 
leds[35] = CRGB::Yellow; 
leds[36] = CRGB::Yellow; 
leds[37] = CRGB::Yellow; FastLED.show(); delay(500);


leds[11] = CRGB::Yellow; 
leds[12] = CRGB::Yellow; 
leds[13] = CRGB::Yellow; 
leds[3] = CRGB::Black; 
leds[4] = CRGB::Black; 
leds[5] = CRGB::Black;
leds[85] = CRGB::Black; 
leds[73] = CRGB::Black; 
leds[74] = CRGB::Black; 
leds[59] = CRGB::Black; 
leds[57] = CRGB::Black; 
leds[58] = CRGB::Black; 
leds[33] = CRGB::Black; 
leds[34] = CRGB::Black; 
leds[35] = CRGB::Black; 
leds[36] = CRGB::Black; 
leds[37] = CRGB::Black; 
leds[87] = CRGB::Yellow; 
leds[76] = CRGB::Yellow; 
leds[77] = CRGB::Yellow; 
leds[61] = CRGB::Yellow; 
leds[62] = CRGB::Yellow; 
leds[63] = CRGB::Yellow; 
leds[39] = CRGB::Yellow; 
leds[40] = CRGB::Yellow; 
leds[41] = CRGB::Yellow; 
leds[42] = CRGB::Yellow; 
leds[43] = CRGB::Yellow; FastLED.show(); delay(500);


leds[11] = CRGB::Black; 
leds[12] = CRGB::Black; 
leds[13] = CRGB::Black;


leds[21] = CRGB::Yellow; 
leds[19] = CRGB::Yellow; 
leds[20] = CRGB::Yellow;



leds[89] = CRGB::Yellow; 
leds[79] = CRGB::Yellow; 

leds[65] = CRGB::Yellow; 
leds[66] = CRGB::Yellow; 

leds[45] = CRGB::Yellow; 
leds[46] = CRGB::Yellow; 
leds[47] = CRGB::Yellow; 

leds[49] = CRGB::Yellow; 
leds[87] = CRGB::Black; 
leds[76] = CRGB::Black; 
leds[77] = CRGB::Black; 
leds[61] = CRGB::Black; 
leds[62] = CRGB::Black; 
leds[63] = CRGB::Black; 
leds[39] = CRGB::Black; 
leds[40] = CRGB::Black; 
leds[41] = CRGB::Black; 
leds[42] = CRGB::Black; 
leds[43] = CRGB::Black; FastLED.show(); delay(500);



leds[21] = CRGB::Black; 
leds[19] = CRGB::Black; 
leds[20] = CRGB::Black;
leds[89] = CRGB::Black ; 
leds[79] = CRGB::Black; 

leds[65] = CRGB::Black; 
leds[66] = CRGB::Black; 

leds[45] = CRGB::Black; 
leds[46] = CRGB::Black; 
leds[47] = CRGB::Black; 

leds[49] = CRGB::Black; FastLED.show(); delay(500);





}
  if (bpetit == HIGH && buttonState == HIGH && bpetit == LOW ) {
leds[69] = CRGB::Magenta; leds[67] = CRGB::Magenta; leds[52] = CRGB::Magenta; leds[48] = CRGB::Magenta; leds[82] = CRGB::Magenta; leds[80] = CRGB::Magenta; FastLED.show();
  
}else {

  leds[69] = CRGB::Black; leds[67] = CRGB::Black; leds[52] = CRGB::Black; leds[48] = CRGB::Black; leds[82] = CRGB::Black; leds[80] = CRGB::Black; FastLED.show();
}
  
  

if (balarme == LOW && buttonState == LOW && bgarage == LOW &&  bporte == LOW) {
 FastLED.setBrightness(5);
 leds[91] = CRGB::Black;  leds[92] = CRGB::Black; leds[89] = CRGB::Black;leds[87] = CRGB::Black; leds[85] = CRGB::Black; leds[92] = CRGB::Black;  FastLED.show(); 
}


}
