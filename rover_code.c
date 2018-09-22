//Group 67
//Functions written by Amaar Quadri unless stated otherwise

#include "PC_FileIO.c"

//COORDINATE SYSTEM
//the x direction is forwards from the perspective of the rover when it starts
//the y direction is left from the perspective of the rover when it starts
//angles are measured counterclockwise from the positive x axis in the range [0, 360)
//distances are measured in cm, and angles are measured in degrees
//the gyro angles are expected to increase when it is rotating counterclockwise (this required mounting the gyro upside down)

//CONFIGURABLE CONSTANTS

//the power that is sent to the 2 motors while driving
const int VELOCITY = 30;
//the magnitude of the power that is sent to each motor while rotating in place
const int ANGULAR_VELOCITY = 20;
//minimum distance used to determine whether a new point is a different obstacle or part of the same obstacle
const float MIN_OBSTACLE_DISTANCE = 50;
//the minimum distance returned by the ultrasonic sensor for consideration as an actual target as opposed to a sensor error/unwanted data
const int MIN_ULTRASONIC_DISTANCE = 100;
//the power sent to the clamp when closing and openning
const int CLAMP_VELOCITY = 20;
//the amount of time the clamp spends openning and closing
const int CLAMP_TIME = 500;
//the magnitude of the power sent to the motors when lifting the arm
const int ARM_RAISE_VELOCITY = 100;
//the amount of time the motors spend lifting the arm
const int ARM_RAISE_TIME = 3000;
//the magnitude of the power sent to the motors when lowering the arm
const int ARM_LOWER_VELOCITY = 50;
//the amount of time the motors spend lowering the arm
const int ARM_LOWER_TIME = 1500;
//the number of targets to pick up
const int TARGETS_COUNT = 3;
//the number of objects to examine
const int OBJECTS_COUNT = 3;
//if true the rover will search until it finds TARGETS_COUNT targets, if false the rover will search until it finds OBJECTS_COUNT objects
const bool USE_TARGETS_COUNT = false;

//UTILITY FUNCTIONS

//converts the given angle to the equivalent angle in [0, 360)
void normalize(float &angle) {
	while (angle < 0) angle += 360;
	while (angle >= 360) angle -= 360;
}

//returns the euclidean distance between the given start and end points
float hypot(float startX, float startY, float endX, float endY) {
	float deltaX = endX - startX, deltaY = endY - startY;
	return sqrt(deltaX * deltaX + deltaY * deltaY);
}

//converts the given angle in radians to degrees
float toDegrees(float radians) {
	return radians * 180 / PI;
}

//converts the given angle in degrees to radians
float toRadians(float degrees) {
	return degrees * PI / 180;
}

//returns whether or not the given target point is within MIN_OBSTACLE_DISTANCE of any previously found obstacle
bool alreadyFound(float targetX, float targetY, float *obstaclesX, float *obstaclesY, int obstaclesSize) {
	for (int j = 0; j < obstaclesSize; j++) if (hypot(targetX, targetY, obstaclesX[j], obstaclesY[j]) < MIN_OBSTACLE_DISTANCE) return true;
	return false;
}

//writes the rovers x, y position and angle to the log file
void logPosition(TFileHandle logFile, float currentLocationX, float currentLocationY, float currentAngle) {
	writeTextPC(logFile, "(x = ");
	writeFloatPC(logFile, "%.2f" , currentLocationX);
	writeTextPC(logFile, ", y = ");
	writeFloatPC(logFile, "%.2f" , currentLocationY);
	writeTextPC(logFile, ", theta = ");
	writeFloatPC(logFile, "%.2f" , currentAngle);
	writeTextPC(logFile, ")");
}

//LIFETIME FUNCTIONS

//sets up the sensors and creates and returns the log file
TFileHandle setup() {
	SensorType[S3] = sensorEV3_Ultrasonic;
	SensorType[S4] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[S4] = modeEV3Color_Color;
	wait1Msec(50);
	SensorType[S1] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S1] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	while (getGyroRate(S1) != 0) {}
	resetGyro(S1);

	TFileHandle logFile;
	word fileSize = 1000;
	openWritePC(logFile, "rover_log.txt", fileSize);
	return logFile;
}

//waits for the user to press the enter button and displays a message to the user
void waitForConfirmation(char* message) {
	displayString(3, message);
	while (!getButtonPress(buttonEnter)) {}
	while (getButtonPress(buttonEnter)) {}
	wait1Msec(1000);
	displayString(3, "                              ");
}

//drives the rover in its current direction for the specified distance
void driveDistance(float distance, float &currentLocationX, float &currentLocationY, float &currentAngle, TFileHandle logFile) {
	if (distance < 0) {
		writeTextPC(logFile, "Driving from ");
		logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
		writeTextPC(logFile, " but no need to drive.");
		writeEndlPC(logFile);
		return;
	}
	writeTextPC(logFile, "Driving from ");
	logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
	writeTextPC(logFile, " for ");
	writeFloatPC(logFile, "%.2f" , distance);
	writeTextPC(logFile, " cm.");
	writeEndlPC(logFile);

	displayString(2, "DRIVING TOWARDS TARGET");

	//convert distance into encoder counts
	float encoderDistance = distance * 360 / (3.0 * PI);

	nMotorEncoder[motorA] = 0;
	motor[motorA] = motor[motorD] = VELOCITY;
	while (nMotorEncoder[motorA] < encoderDistance) {}
	motor[motorA] = motor[motorD] = 0;

	currentLocationX += distance * cos(toRadians(currentAngle));
	currentLocationY += distance * sin(toRadians(currentAngle));

	displayString(2, "                      ");

	writeTextPC(logFile, "Done driving. Now at ");
	logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
	writeEndlPC(logFile);
	return;
}

//rotates the rover a full 360 degrees and returns the first object that is found by the ultrasonic sensor
//if nothing is found, it drives 100 cm and tries again recursively
//any object that is found is rejected if it is within a specified (hard-coded) distance of any previously found obstalce
void search(float *obstaclesX, float *obstaclesY, int obstaclesSize, float &currentLocationX, 
	float &currentLocationY, float &currentAngle, float &targetX, float &targetY, TFileHandle logFile) {
	writeTextPC(logFile, "Starting search at ");
	logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
	writeEndlPC(logFile);

	displayString(2, "SEARCHING");
	resetGyro(S1);
	motor[motorA] = ANGULAR_VELOCITY;
	motor[motorD] = -ANGULAR_VELOCITY;
	while (getGyroDegrees(S1) < 360) {
		if (SensorValue(S3) < MIN_ULTRASONIC_DISTANCE) {
			motor[motorA] = motor[motorD] = 0;
			currentAngle += getGyroDegrees(S1);
			normalize(currentAngle);

			float distanceToTarget = SensorValue(S3);

			//calculate the x and y position of the target
			targetX = currentLocationX + distanceToTarget * cos(toRadians(currentAngle));
			targetY = currentLocationY + distanceToTarget * sin(toRadians(currentAngle));
			if (!alreadyFound(targetX, targetY, obstaclesX, obstaclesY, obstaclesSize)) {
				writeTextPC(logFile, "Done searching now at ");
				logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
				writeEndlPC(logFile);

				displayString(2, "         ");
				return;
			}
		}
	}
	motor[motorA] = motor[motorD] = 0;
	//currentAngle is unaffected due to full rotation
	driveDistance(100, currentLocationX, currentLocationY, currentAngle, logFile);
	search(obstaclesX, obstaclesY, obstaclesSize, currentLocationX, currentLocationY, currentAngle, targetX, targetY, logFile);
}

//uses the color sensor to determine if it has reached a target
bool isTarget() {
	return SensorValue[S4] == 5;
}

//FUNCTION WRITTEN BY HADI KHADRA
//picks up a target directly in front of it using the robotic arm
void pickUpTarget() {
	waitForConfirmation("PLEASE SWITCH TO CLAMP");

	//close clamp
	time1[T1] = 0;
	motor[motorA] = CLAMP_VELOCITY;
	while (time1[T1] < CLAMP_TIME) {}
	motor[motorA] = 0;

	wait1Msec(500);

	//raise arm
	time1[T1] = 0;
	motor[motorC] = -ARM_RAISE_VELOCITY;
	motor[motorB] = ARM_RAISE_VELOCITY;
	while (time1[T1] < ARM_RAISE_TIME) {}
	motor[motorC] = -ARM_RAISE_VELOCITY / 10;
	motor[motorB] = ARM_RAISE_VELOCITY / 10;

	wait1Msec(500);

	//open clamp
	time1[T1] = 0;
	motor[motorA] = -CLAMP_VELOCITY;
	while (time1[T1] < CLAMP_TIME) {}
	motor[motorA] = 0;

	wait1Msec(2000);

	//lower arm
	time1[T1] = 0;
	motor[motorC] = ARM_LOWER_VELOCITY / 4;
	motor[motorB] = -ARM_LOWER_VELOCITY / 4;
	while (time1[T1] < ARM_LOWER_TIME) {}
	motor[motorC] = motor[motorB] = 0;

	waitForConfirmation("PLEASE SWITCH TO WHEELS");
}

//FUNCTION WRITTEN BY FAHMID KADER
//drives the rover back to its base at (0, 0)
void returnToBase(float &currentLocationX, float &currentLocationY, float &currentAngle, TFileHandle logFile) {
	writeTextPC(logFile, "Returning to base from ");
	logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
	writeEndlPC(logFile);

	displayString(0, "RETURNING TO BASE");

	float rotateCounterClockwise = toDegrees(atan2(currentLocationY, currentLocationX)) - currentAngle - 180;
	normalize(rotateCounterClockwise);

	resetGyro(S1);
	motor[motorA] = ANGULAR_VELOCITY;
	motor[motorD] = -ANGULAR_VELOCITY;
	while (getGyroDegrees(S1) < rotateCounterClockwise) {}

	driveDistance(hypot(currentLocationX, currentLocationY, 0, 0), currentLocationX, currentLocationY, currentAngle, logFile);
}

//FUNCTION WRITTEN BY SEPEHR TALEBI
//prints the coordinates of the targets and obstacles that have been found to the log file
void printOutput(TFileHandle logFile, float *obstaclesX, float *obstaclesY, int obstaclesSize, float *targetsX, float *targetY, float targetsSize) {
	if (targetsSize > 0) {
		writeTextPC(logFile, "Coordinates of targets:");
		writeEndlPC(logFile);

		//write each target's coordinates on one line, seperated by spaces
		for (int count = 0; count < targetsSize; count++) {
			writeFloatPC(logFile, "%.2f" , obstaclesX[count]);
			writeTextPC(logFile, " ");
			writeFloatPC(logFile, "%.2f" , obstaclesY[count]);
			writeEndlPC(logFile);
		}
	}
	else {
		writeTextPC(logFile, "No targets found!");
		writeEndlPC(logFile);			
	}

	if (obstaclesSize > 0) {
		writeTextPC(logFile, "Coordinates of obstacles:");
		writeEndlPC(logFile);

		//write each obstacle's coordinates on one line, seperated by spaces
		for (int count = 0; count < obstaclesSize; count++) {
			writeFloatPC(logFile, "%.2f" , obstaclesX[count]);
			writeTextPC(logFile, " ");
			writeFloatPC(logFile, "%.2f" , obstaclesY[count]);
			writeEndlPC(logFile);
		}
	}
	else {
		writeTextPC(logFile, "No obstacles found!");
		writeEndlPC(logFile);					
	}
}

task main() {
	TFileHandle logFile = setup();

	float currentLocationX = 0;
	float currentLocationY = 0;
	float currentAngle = 0;

	//increase size as required
	float obstaclesX[50];
	float obstaclesY[50];
	int obstaclesSize = 0;

	//increase size as required
	float targetsX[50];
	float targetsY[50];
	int targetsCollected = 0;

	displayString(0, "OBSTACLES FOUND: 0");
	displayString(1, "TARGETS FOUND: 0");

	//the end condition in the for loop is dependant on USE_TARGETS_COUNT
	//if USE_TARGETS_COUNT is true, then the for loop will behave like a while loop 
	for (int j = 0; USE_TARGETS_COUNT ?  targetsCollected < TARGETS_COUNT : j < OBJECTS_COUNT; j++) {
		float targetX, targetY;
		search(obstaclesX, obstaclesY, obstaclesSize, currentLocationX, currentLocationY, currentAngle, targetX, targetY, logFile);
		//subtract 20 cm from distance so that rover doesn't bump into its destination
		driveDistance(hypot(currentLocationX, currentLocationY, targetX, targetY) - 20, currentLocationX, currentLocationY, currentAngle, logFile);
		waitForConfirmation("PRESS TO CONTINUE");
		if (isTarget()) {
			writeTextPC(logFile, "Picking up target at ");
			logPosition(logFile, currentLocationX, currentLocationY, currentAngle);
			writeEndlPC(logFile);

			pickUpTarget();
			targetsX[targetsCollected] = currentLocationX;
			targetsY[targetsCollected] = currentLocationY;

			targetsCollected++;
			displayString(1, "TARGETS FOUND: %d", targetsCollected);
		}
		else {
			obstaclesX[obstaclesSize] = targetX;
			obstaclesY[obstaclesSize++] = targetY;
			displayString(0, "OBSTACLES FOUND: %d", obstaclesSize);
		}
		waitForConfirmation("PRESS TO CONTINUE");
	}
	returnToBase(currentLocationX, currentLocationY, currentAngle, logFile);
	printOutput(logFile, obstaclesX, obstaclesY, obstaclesSize, targetsX, targetsY, targetsCollected);
}