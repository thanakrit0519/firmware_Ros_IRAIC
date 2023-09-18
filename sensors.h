/* Functions for various sensor types */
#define inPin 22
#define outPin 23

float microsecondsToCm(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per cm.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long Ping() {
  long duration, range;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(outPin, OUTPUT);
  pinMode(inPin, INPUT);
  digitalWrite(outPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(outPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(inPin, HIGH);

  // convert the time into meters
  range = microsecondsToCm(duration);
  
  return(range);
}
