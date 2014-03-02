import processing.serial.*;
Serial serial;

String stringGyroX, stringGyroY;
String stringAccX, stringAccY;
String stringCompX, stringCompY;
String stringKalmanX, stringKalmanY;

final int width = 800;
final int height = 600;

float[] gyroX = new float[width];
float[] gyroY = new float[width];

float[] accX = new float[width];
float[] accY = new float[width];

float[] compX = new float[width];
float[] compY = new float[width];

float[] kalmanX = new float[width];
float[] kalmanY = new float[width];

boolean drawValues  = false;

void setup() {
  size(width, height);
  println(Serial.list()); // Use this to print connected serial devices
  serial = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
  serial.bufferUntil('\n'); // Buffer until line feed

  for (int i = 0; i < width; i++) { // center all variables
    gyroX[i] = height/2;
    gyroY[i] = height/2;
    accX[i] = height/2;
    accY[i] = height/2;
    compX[i] = height/2;
    compY[i] = height/2;
    kalmanX[i] = height/2;
    kalmanY[i] = height/2;
  }

  drawGraph(); // Draw graph at startup
}

void draw() {
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    drawGraph();
  }
}

void drawGraph() {
  background(255); // White
  for (int i = 0; i < width; i++) {
    stroke(200); // Grey
    line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(0); // Black
  for (int i = 1; i <= 3; i++)
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
}

void serialEvent (Serial serial) {
  // Get the ASCII strings:
  stringAccX = serial.readStringUntil('\t');
  stringGyroX = serial.readStringUntil('\t');
  stringCompX = serial.readStringUntil('\t');
  stringKalmanX = serial.readStringUntil('\t');

  serial.readStringUntil('\t'); // Ignore extra tab

  stringAccY = serial.readStringUntil('\t');
  stringGyroY = serial.readStringUntil('\t');
  stringCompY = serial.readStringUntil('\t');
  stringKalmanY = serial.readStringUntil('\t');

  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph

  //printAxis(); // Used for debugging
}

void printAxis() {
  print(stringGyroX);
  print(stringAccX);
  print(stringCompX);
  print(stringKalmanX);

  print('\t');

  print(stringGyroY);
  print(stringAccY);
  print(stringCompY);
  print(stringKalmanY);

  println();
}
