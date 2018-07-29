/*
  Control Servos and Motors using JSON to parse structures contaning:
  MR: motor on the right
  ML: motor on the left
  SR: servo on the right
  SL: servo on the left

  Control sequence example: {"mr":70, "ml":60, "sr":-20, "sl":-30}\n
  The "\n" at the end is necessary so that serial knows when to stop reading.

  This file also continuously obtain GPS data.


  TODO: fix min/max to motor


  ##############################
  CREATED BY: Lucas Malta
  DATE: 26 August 2016

*/


#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <aJSON.h>

#define MOTOR_L_PIN 10  // LEFT MOTOR control pin
#define MOTOR_R_PIN 9   // RIGHT MOTOR control pin
#define SERVO_L_PIN 6   // LEFT SERVO control pin
#define SERVO_R_PIN 5   // RIGHT SERVO control pin
#define GPS_RX_PIN  11  // GPS RX pin
#define GPS_TX_PIN  3   // GPS TX pin
#define MIN_MOTOR_SPD 700     // Minimum motor speed
#define MAX_MOTOR_SPD 2200    // Maximum motor speed
#define MIN_SERVO_POS 700     // Minimum servo angle
#define MAX_SERVO_POS 2200    // Maximum servo angle

Servo myservoL; // Left servo object
Servo myservoR; // Right servo object
Servo mymotorL; // Left motor object
Servo mymotorR; // Right motor object

String inputString = "";         // A string to hold incoming control sequence
boolean stringComplete = false;  // Whether the string is complete
const int sentenceSize = 200;    // Max control sequence size


SoftwareSerial gpsSerial(GPS_RX_PIN,GPS_TX_PIN); // RX, TX (TX not used)
TinyGPS gps; // Create gps object

int valSL = 10;
int valSR = 10;
int valML = 10;
int valMR = 10;
long latS = -1;
long longW = -1;



void setup() {
	// Attach pins to motors -- servos should NOT be attached here
	mymotorR.attach(MOTOR_R_PIN); // Attach pin to right motor
	mymotorL.attach(MOTOR_L_PIN); // Attach pin to left motor

        // Connect to servos/motors
	myservoR.attach(SERVO_R_PIN); // attach right servo
	myservoL.attach(SERVO_L_PIN); // attach left servo

        // Checking attachment
        if (!mymotorR.attached() | !mymotorL.attached() | !myservoR.attached() | !myservoL.attached()) {
          Serial.println("ERROR: Attaching problem.");
          return; 
        }
        
	// Initialize serial:
	Serial.begin(9600);
	gpsSerial.begin(9600);

	// Reserve sentenceSize bytes for the inputString:
	inputString.reserve(sentenceSize);
}


void loop() {
	// Print the string when a newline arrives:
	if (stringComplete) {
                //Serial.println("input: " );  
		//Serial.println(inputString);

		// Reads data and execute command
		displayParams();

		// Clear the string:
		inputString = "";
		stringComplete = false;
	}


	if(gpsSerial.available()){ // Check for gps data
		if(gps.encode(gpsSerial.read())){ // Encode gps data
			gps.get_position(&latS,&longW); // Get latitude and longitude
		}
	}

        else {
          // Set lat/long back to -1 when GPS is not available
          latS = -2;
          longW = -2;
        }        
}


void displayParams()
{
	// Parse JSON data
	char* value;
	char jsonString[sentenceSize];
	inputString.toCharArray(jsonString,sentenceSize);

	aJsonObject* root = aJson.parse(jsonString);
	if(!root) {
   		Serial.println(jsonString);
 		Serial.println("ERROR: No control data");
		return;
	}

	aJsonObject* MR = aJson.getObjectItem(root, "mr");  
	aJsonObject* ML = aJson.getObjectItem(root, "ml");
	aJsonObject* SR = aJson.getObjectItem(root, "sr"); 
	aJsonObject* SL = aJson.getObjectItem(root, "sl"); 


        if (!MR | !ML | !SR | !SL ) {
		Serial.println("ERROR: Invalid command format. Missing input?");
		return;
	}
       
        if (MR->type != aJson_Int | ML->type != aJson_Int | SR->type != aJson_Int  | SL->type != aJson_Int){
		Serial.println("ERROR: Invalid type. Expecting integer..");
		return;
	}

	// Connect to servos/motors
	//myservoR.attach(SERVO_R_PIN); // attach right servo
	//myservoL.attach(SERVO_L_PIN); // attach left servo

	//Leftservo position   
	valSL = constrain( SL->valueint, -100, 100 );
	myservoL.writeMicroseconds( map(valSL, -100, 100, MIN_SERVO_POS, MAX_SERVO_POS ) );

	//Right servo position
	valSR = constrain( SR->valueint, -100, 100 );
	myservoR.writeMicroseconds( map(valSR, -100, 100, MIN_SERVO_POS, MAX_SERVO_POS ) );

	//Left motor position
	valML = constrain( ML->valueint, 0, 100 );
	mymotorL.writeMicroseconds( map(valML, 0, 100, MIN_MOTOR_SPD, MAX_MOTOR_SPD ) );

	//Right motor position
	valMR = constrain( MR->valueint, 0, 100 );
	mymotorR.writeMicroseconds( map(valMR, 0, 100, MIN_MOTOR_SPD, MAX_MOTOR_SPD ) );


	// Avoid noise on servos for trying to hold position
	// Motors should NOT be dettached though...
	//delay(1000); // wait for servo to get there
	//myservoL.detach();
	//myservoR.detach();       

        Serial.print("{\"Lat\":");
	Serial.print(latS);
	Serial.print(", \"Long\":");
	Serial.print(longW);
	Serial.print(", \"RServo\":");
	Serial.print(valSR);
	Serial.print(", \"LServo\":");
	Serial.print(valSL);
	Serial.print(", \"RMotor\":");
	Serial.print(valMR);
	Serial.print(", \"LMotor\":");
	Serial.print(valML);
	Serial.print("}");
	Serial.print("\n");

	Serial.flush(); // Wait till outgoing tx is done
        aJson.deleteItem(root);
}



/*
   SerialEvent occurs whenever a new data comes in the
   hardware serial RX.  This routine is run between each
   time loop() runs, so using delay inside loop can delay
   response.  Multiple bytes of data may be available.
 */
void serialEvent() {
	while (Serial.available()) {
		// Get the new byte:
		char inChar = (char)Serial.read();
		// Add it to the inputString:
		inputString += inChar;
		// If the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			stringComplete = true;

		}
	}
}

