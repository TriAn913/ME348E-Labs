#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
int Steps = 0;
boolean Direction = false;
float commandedSteps;

bool toggle = true;

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
    // Serial.end();   /*end serial communication*/
    // Serial.begin(9600);  /*clear serial buffer*/
    // Serial.println("Enter desired rotation and type. Press enter with no input to use default");
    // while (Serial.available()== 0)
    // {
    // }

    // commandedSteps = Serial.parseInt();
    // Serial.println(commandedSteps);
    // toggle = false;

    char serialArray[32];
    bool newData = false;
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    while (Serial.available() > 0 && newData == false)
    {
      rc = Serial.read();

      if (rc != endMarker && Serial.available() > 0)
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
    
    commandedSteps = atof(serialArray);
    Serial.println(commandedSteps);
    toggle = false;
  }
  else
  {

    if (commandedSteps != 0)
    {
      for (int i = 0; i < commandedSteps; i++)
      {
        // stepperFULL(1);
        stepperHALF(1);
        delayMicroseconds(1500); // delay between steps
      }
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
