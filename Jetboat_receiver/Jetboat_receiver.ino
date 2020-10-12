#include <Servo.h>

#include <SoftwareSerial.h>

///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////////////
void turnLed(bool on);

//Channels
int x[15], ch1[15], ch[7], i;

int ledPin = 10;
long ledBlink = 0;
bool ledIsOn = false;

SoftwareSerial BTserial(10, 11); // RX | TX
Servo wheel, gas;
#define gas_min  1000
#define gas_max  2000
#define gas_middle 1500
#define steer_min 0
#define steer_max 180
int wheel_pin = 9, gas_pin = 4;

int data_receive[7];
byte  throttle_active, brake_active;
int battery_voltage, receive_watchdog, throttle_out, brake_out, steering_out;
long motor_current;
byte f1 = 150, f2 = 151, f3 = 152;

#define MY_PIN 3 // we could choose any pin

void setup() {
  pinMode(MY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  Serial.begin(9600);  //Start the serial port @ 9600bps.
  Serial.println("BB");
  BTserial.begin(9600);
  //Serial.println("BTserial started at 34800");
  wheel.attach(wheel_pin);
  gas.attach(gas_pin);
  wheel.write((steer_min + steer_max) / 2);

  /*gas.write(gas_max);
    delay(6000);
    gas.write(gas_min);
    delay(2000);
    gas.write(gas_middle);
    //delay(4000);*/


  //OCR0A = 0; //Set PWM duty cycle to 0%.
  //TCCR0A |= (1 << WGM01) | (1 << WGM00); //Enable the PWM fuction.
}

void loop() {
  read_rc();

  Serial.print(ch[1]); Serial.print("\t");
  //Serial.print(ch[2]); Serial.print("\t");
  Serial.print(ch[3]); Serial.print("\t");
  //Serial.print(ch[4]); Serial.print("\t");
  //Serial.print(ch[5]); Serial.print("\t");
  //Serial.print(ch[6]); 
  Serial.print("\n");

  battery_voltage = 100;//analogRead(0) >> 3;                                               //Read the battery voltage and divide by 8.
  if (true || battery_voltage > 100)battery_voltage = 100;                                    //Limit battery voltage to 100.


  /*while (BTserial.available() > 0) { //Read the serial port.

    data_receive[6] = data_receive[5];
    data_receive[5] = data_receive[4];
    data_receive[4] = data_receive[3];                                                //Shift the new data in the data_receive array.
    data_receive[3] = data_receive[2];
    data_receive[2] = data_receive[1];
    data_receive[1] = data_receive[0];
    data_receive[0] = (byte)BTserial.read();
    //Serial.println(String(data_receive[0]));

    //Check if new data has arrived.
    if (data_receive[6] == 250 && data_receive[3] == 251 && data_receive[0] == 252) {
      if (true || data_receive[3] == data_receive[1]) {
        //Send the battery voltage and include check bytes.
        /*BTserial.write((byte*)&f1, 1);
          BTserial.write((byte*)&battery_voltage, 1);
          BTserial.write((byte*)&f2, 1);
          BTserial.write((byte*)&battery_voltage, 1);
          BTserial.write((byte*)&f3, 1);*/
  /*BTserial.write((byte*)&battery_voltage, 1);
    receive_watchdog = 100;
  */
  //Serial.println("======================");
  //delay(120);//Reset the watchdog timer.

  //Calculate the throttle value.
  throttle_out = ch[3];//(data_receive[2] << 8) | data_receive[1];
  steering_out = ch[1];//(data_receive[5] << 8) | data_receive[4];
  /*Serial.print("Gas:");
    Serial.println(throttle_out);
    Serial.print("Wheel:");
    Serial.println(steering_out);
  */
  /*}
    }
    }*/

  if (true || receive_watchdog > 0) {                                                         //If the watchdog timer is still active.
    wheel.write(map(steering_out, 1000, 2000, steer_min, steer_max));
    
    Serial.print("Steer: ");
    Serial.println(map(steering_out, 1000, 2000, steer_min, steer_max));
    if (throttle_out < 1500)
      throttle_out = 1000;
    gas.write(constrain(throttle_out, gas_min, gas_max));
  }

  if (receive_watchdog > 0) {
    receive_watchdog --;                                       //If the watchdog variable is larger then 0, decrease the watchdog variable by 1.
    turnLed(true);  //Turn on the switch led
  }
  else {
    //Failsafe
    wheel.write((steer_min + steer_max) / 2);
    gas.write(gas_middle);
    ///
    if (micros() - ledBlink >= 1000) {
      if (ledIsOn == true)
      {
        turnLed(false);
      }
      else {
        turnLed(true);
      }
      ledBlink = micros();
    }


    //If the watchdog timer is 0.
    //OCR0A = 0;                                                                        //Set the PWM duty cycle to 0.
    if (throttle_active == 1) {                                                       //If throttle active flag is set.
      //Set PWM dut cycle to zero.
    }
    if (brake_active == 1) {                                                          //If break active flag is set.
      //Set break active flag to 0.
    }
  }
  delay(100);
}

void turnLed(bool on) {
  if (on) {
    digitalWrite(ledPin, HIGH);
    ledIsOn = true;
  }
  else {
    digitalWrite(ledPin, LOW);
    ledIsOn = false;
  }
}
