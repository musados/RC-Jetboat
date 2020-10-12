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

unsigned long timer;
int bat_voltage, throttle, LRThrottle, watchdog;
int last_throttle = 1000, last_LRThrottle = 1500;
int data_receive[5];
byte first_receive, first_receive_counter;
int f1 = 250, f2 = 251, f3 = 252;


void setup() {
  Serial.begin(9600); //Start serial port @ 9600bps

  pinMode(4, OUTPUT); //LED Yellow
  pinMode(5, OUTPUT); //LED Red
  pinMode(6, OUTPUT); //LEG Green

  //Startup LED blink.
  digitalWrite(6, HIGH);
  delay(300);
  digitalWrite(6, LOW);
  digitalWrite(4, HIGH);
  delay(300);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  delay(300);
  digitalWrite(5, LOW);
  delay(300);
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);

  //Check the battery voltage and display.
  bat_voltage = analogRead(2);
  if (bat_voltage > 240)digitalWrite(5, HIGH);
  if (bat_voltage > 260)digitalWrite(4, HIGH);
  if (bat_voltage > 270)digitalWrite(6, HIGH);

  //Wait 3 seconds and turn off the LED's.
  delay(3000);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

}
void loop() {
  //Blink the red LED when there is no connection with the receiver.
  if (first_receive == 0) {
    if (first_receive_counter < 20) {
      first_receive_counter ++;
      if (first_receive_counter == 19)digitalWrite(5, HIGH);
    }
    else {
      first_receive_counter = 0;
      digitalWrite(5, LOW);
    }
  }
  //Read the incomming data.
  if (Serial.available() > 0) {
    /*data_receive[4] = data_receive[3];
    data_receive[3] = data_receive[2];
    data_receive[2] = data_receive[1];
    data_receive[1] = data_receive[0];
    data_receive[0] =*/ 
    byte bat = (byte)Serial.read();
    /*Serial.println("\n\n");
    Serial.println(bat);
    Serial.println("\n\n");*/
    //Serial.println((byte)Serial.read());
    //Check serial data for valid battery voltage.
    if (true){// || data_receive[4] == 150 && data_receive[2] == 151 && data_receive[0] == 152) {
      if (true){// || data_receive[3] == data_receive[1]) {
        bat_voltage = bat;//data_receive[3];
      }
      first_receive = 1;  //Set first receive flag to 1.
      watchdog = 0;       //Reset the watchdog timer.
      //Display the battery voltage of the longboard.
      digitalWrite(4, LOW);
      if (bat_voltage > 86)digitalWrite(5, LOW);
      digitalWrite(6, LOW);
      if (bat_voltage > 96)digitalWrite(6, HIGH);
      else if (bat_voltage > 91)digitalWrite(4, HIGH);
      else if (bat_voltage > 86)digitalWrite(5, HIGH);
      else digitalWrite(5, !digitalRead(5));
    }
  }
  //Read the throttle potentiometer.
  throttle = analogRead(A1);
  LRThrottle = analogRead(A0);

  throttle = map(throttle, 0, 1023, 995, 2000);

  /*Serial.print("throttle origin: ");
  Serial.println(throttle);
  Serial.print("Last throttle: ");
  Serial.println(last_throttle);
  */
  
  if (!(abs(throttle - last_throttle) > 2))
    throttle = last_throttle;
  last_throttle = throttle;
  
  /*Serial.print("throttle current: ");
  Serial.println(throttle);
*/

  byte thr_high = throttle >> 8;
  byte thr_low = (byte)throttle;
  //Serial.println("back");
  //Serial.println((thr_high<<8) | thr_low);
  LRThrottle = map(LRThrottle, 0, 1023, 2000, 1000);

  /*Serial.print("LR throttle origin: ");
  Serial.println(LRThrottle);
  Serial.print("Last LR throttle: ");
  Serial.println(last_LRThrottle);
  */

  if (!(abs(LRThrottle - last_LRThrottle) > 18))
    LRThrottle = last_LRThrottle;
  last_LRThrottle = LRThrottle;
  
  /*Serial.print("LR throttle current: ");
  Serial.println(LRThrottle);
  Serial.println("==========================================");
  */

  byte thrLR_high = LRThrottle >> 8;
  byte thrLR_low = (byte)LRThrottle;


  //Send the throttle value to the longboard.
  Serial.write((byte*)&f1, 1);
  Serial.write((byte*)&thrLR_high, 1);
  Serial.write((byte*)&thrLR_low, 1);
  Serial.write((byte*)&f2, 1);
  Serial.write((byte*)&thr_high, 1);
  Serial.write((byte*)&thr_low, 1);
  Serial.write((byte*)&f3, 1);

  //Increment the watchdof timer.
  if (watchdog < 10)watchdog ++;
  //After one second of no valid data reset the first_receive flag and blink the red LED.
  if (watchdog == 10 && first_receive == 1) {
    first_receive = 0;
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
  }
  //Wait 100ms for the next loop.
  delay(100);
}


