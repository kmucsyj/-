#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  // put your setup code here, to run once:
  while (!Serial) {
    ;
  }
  count = toggle = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  while(1){
    if(count<=10){
      toggle = toggle_state(toggle);
      digitalWrite(PIN_LED, toggle);
      delay(100);
      count++;
    }
  }
}

int toggle_state(int toggle) {
  return toggle==0 ? 1:0;
}
