char GP2D12;
char a, b;
bool far = false;
bool fire = false;
const int sensorMin = 0;
const int sensorMax = 1024;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(13, LOW);
}


void loop()
{
  int val;

  // read rangefinder
  GP2D12 = read_gp2d12_range(0);
  a = GP2D12 / 10;
  b = GP2D12 % 10;
  val = a * 10 + b;
  Serial.println(GP2D12);

  // set far variable
  if (val > 50) {
    far = true;
  }
  else {
    far = false;
  }
  Serial.print(far);

  if (far) {
    digitalWrite(13, HIGH);   //cliff
    delay(1000);
    digitalWrite(13, LOW);
    far = false;
  }
  else {
    digitalWrite(13, LOW);
  }

  // read flame sensor
  int sensorReading = analogRead(A7);

  if (sensorReading < 900 && sensorReading > 600) { //Distant Fire
    //turn right and drive 20 inches
    //turn right
  }

  else if (sensorReading < 600 && sensorReading > 300) { //close fire
    //turn right and drive 10 inches
    //turn right
  }

  else if (sensorReading < 300) { //point blank fire
    //turn right
    digitalWrite(4, HIGH);
    //side to side/up down
    digitalWrite(4, LOW);
    //turn right
  }
}

float read_gp2d12_range(byte pin)
{
  int tmp;
  tmp = analogRead(pin);
  if (tmp < 3)return -1;
  return (6787.0 / ((float)tmp - 3.0)) - 4.0;
}

