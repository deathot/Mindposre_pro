float microsecondsToCm(long microseconds)
{
  return microseconds / 29 / 2;
}

long Ping(int pin) {
  long duration, range;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  range = microsecondsToCm(duration);
  
  return(range);
}

