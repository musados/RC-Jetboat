unsigned long int a, b, c;

void read_me()  {
  //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
  //this code gives channel values from 0-1000 values
  //    -: ABHILASH :-    //
  a = micros(); //store time value a when pin value falling
  c = a - b;  //calculating time inbetween two peaks
  b = a;      //
  x[i] = c;   //storing 15 value in array
  i = i + 1;       if (i == 15) {
    for (int j = 0; j < 15; j++) {
      ch1[j] = x[j];
    }
    i = 0;
  }
}
//copy store all values from temporary array another array after 15 reading
void read_rc() {
  int i, j, k = 0;
  for (k = 14; k > -1; k--) {
    if (ch1[k] > 3000) {
      j = k; //detecting separation space 10000us in that another array
      receive_watchdog = 100;
    }
  }
  for (i = 1; i <= 6; i++) {
    ch[i] = (ch1[i + j]);
  }
}     //assign 6 channel values after separation space
