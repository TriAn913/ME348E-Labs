#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
int Steps = 0;
boolean Direction;
float commandedSteps;

bool toggle = true;

bool initialToggle = false;

void setup()
{
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop()
{
  if (toggle)
  {
    char serialArray[32];
    bool newData = false;
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    Serial.print("Read Chars:");
    while (newData == false)
    {
      if (Serial.available() > 0)
      {
        rc = Serial.read();


        if (rc != endMarker)
        {
          serialArray[ndx] = rc;
          ndx++;
          if (ndx >= 32)
          {
            ndx = 32 - 1;
          }
        }
        else
        {
          serialArray[ndx] = '\0'; // terminate the string
          ndx = 0;
          newData = true;
        }
      }
    }
    
    while(Serial.available())
    {
      Serial.read();
    }

    commandedSteps = atof(serialArray);
    Serial.print("\nCommanded Steps: ");
    Serial.print(commandedSteps);

    Direction = commandedSteps >= 0 ? true : false;
    Serial.print("\nDirection:");
    Serial.println(Direction);
    toggle = false;
  }
  else
  {

    if (commandedSteps != 0)
    {
      for (int i = 0; i < abs(commandedSteps); i++)
      {
        // stepperFULL(1);
        stepperHALF(1);
        delayMicroseconds(1500); // delay between steps
      }
      Serial.print("Finished rotating by ");
      Serial.print(commandedSteps);
      Serial.print("counts.\n");
      toggle = true;
    }
    else
    {
      for (int i = 0; i < 4096; i++) // given from the template code
      {
        stepperFULL(1);
        delayMicroseconds(1500); // delay between steps
      }
      Direction = !Direction; // after 4096 steps, change direction
    }
  }
}

void stepperFULL(int xw)
{
  for (int x = 0; x < xw; x++)
  {
    switch (Steps)
    {
      case 0:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        break;
      case 1:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        break;
      case 2:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
      case 3:
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
      default:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
    }
    SetDirectionFULL();
  }
} // END StepperFull()

void SetDirectionFULL()
{
  if (Direction == 1)
  {
    Steps++;
  }
  if (Direction == 0)
  {
    Steps--;
  }
  if (Steps > 4)
  {
    Steps = 0;
  }
  if (Steps < 0)
  {
    Steps = 4;
  }
}

/* FILL OUT YOUR CODE HERE FOR HALF STEPS!!*/

void stepperHALF(int xw)
{
  for (int x = 0; x < xw; x++)
  {
    switch (Steps)
    {
      case 0:
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        break;
      case 1:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        break;
      case 2:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, HIGH);
        break;
      case 3:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        break;
      case 4:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        break;
      case 5:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
      case 6:
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
      case 7:
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
      default:
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        break;
    }
    SetDirection();
  }
}
void SetDirection()
{
  if (Direction == 1)
  {
    Steps++;
  }
  else
  {
    Steps--;
  }

  if (Steps > 7)
  {
    Steps = 0;
  }
  if (Steps < 0)
  {
    Steps = 7;
  }
}
