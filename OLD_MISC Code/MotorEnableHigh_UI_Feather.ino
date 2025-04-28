
#define M_ENABLE 33     //These pins are on UI feather but pins are correct
#define BatteryMonitor 34

void setup() {
  // put your setup code here, to run once:

pinMode(M_ENABLE, OUTPUT);
pinMode (BatteryMonitor, INPUT);


 Serial.begin(115200);
 delay(1000);


}

void loop() {
  // put your main code here, to run repeatedly:

  // Writing Digital ENABLE High (User Controlled Later On)

  digitalWrite(M_ENABLE, HIGH);

  int raw_battery = analogRead(BatteryMonitor);
  float battery_conv = (raw_battery / 4095.0) * 3.3;  // Convert to volts (3.3V reference)


  int percent = mapFloat(battery_conv, 1.55, 1.197, 100.0, 0.0);
  percent = clamp(percent, 0.0, 100.0);

  Serial.print("Raw ADC: ");
  Serial.print(raw_battery);
  Serial.print(" | Voltage: ");
  Serial.print(battery_conv, 3);
  Serial.println(" V");
  Serial.print(percent, 1);
  Serial.println(" %");

  delay(500);



}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

float clamp(float val, float minVal, float maxVal) {
  return max(min(val, maxVal), minVal);
}


