import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Graph extends PApplet {

 
Serial serial; 

String stringGyroX, stringGyroY;
String stringAccX, stringAccY;
String stringCompX, stringCompY;
String stringKalmanX, stringKalmanY;

final int width = 1200;
final int height = 800;

float[] gyroX = new float[width];
float[] gyroY = new float[width];

float[] accX = new float[width];
float[] accY = new float[width];

float[] compX = new float[width];
float[] compY = new float[width];

float[] kalmanX = new float[width];
float[] kalmanY = new float[width];

boolean drawValues;

public void setup() {  
  size(width, height);
  println(Serial.list()); // Use this to print connected serial devices
  serial = new Serial(this, Serial.list()[8], 115200);
  serial.bufferUntil('\n'); // Buffer until line feed

  for (int i=0;i<width;i++) { // center all variables
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

public void draw() {
  /* Draw Graph */
  if(drawValues) {
    drawValues = false;
    drawGraph();
  }
}

public void drawGraph() {
  background(255); // white  
  for (int i = 0;i<width;i++) {
    stroke(200); // gray
    line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }
  
  stroke(0); // black
  for (int i = 1; i <= 3; i++)
    line(0, height/4*i, width, height/4*i); // Draw line, indicating 90 deg, 180 deg, and 270 deg
  
  convert();
  drawAxisX();
  //drawAxisY();
}

public void serialEvent (Serial serial) {
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

public void printAxis() {
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
//convert all axis
final int minAngle = 0;
final int maxAngle = 360;

public void convert() {
  /* Convert the gyro x-axis */
  if (stringGyroX != null) {
    // Trim off any whitespace:
    stringGyroX = trim(stringGyroX);
    // Convert to an float and map to the screen height, then save in buffer:    
    gyroX[gyroX.length-1] = map(PApplet.parseFloat(stringGyroX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the gyro y-axis */
  if (stringGyroY != null) {    
    // Trim off any whitespace:
    stringGyroY = trim(stringGyroY);
    // Convert to an float and map to the screen height, then save in buffer:   
    gyroY[gyroY.length-1] = map(PApplet.parseFloat(stringGyroY), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the accelerometer x-axis */
  if (stringAccX != null) {
    // Trim off any whitespace:
    stringAccX = trim(stringAccX);
    // Convert to an float and map to the screen height, then save in buffer:    
    accX[accX.length-1] = map(PApplet.parseFloat(stringAccX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the accelerometer y-axis */
  if (stringAccY != null) {
    // Trim off any whitespace:
    stringAccY = trim(stringAccY);
    // Convert to an float and map to the screen height, then save in buffer:        
    accY[accY.length-1] = map(PApplet.parseFloat(stringAccY), minAngle, maxAngle, 0, height);
  }

  /* Convert the complementary filter x-axis */
  if (stringCompX != null) {
    // Trim off any whitespace:
    stringCompX = trim(stringCompX);
    // Convert to an float and map to the screen height, then save in buffer:    
    compX[compX.length-1] = map(PApplet.parseFloat(stringCompX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the complementary filter x-axis */
  if (stringCompY != null) {
    // Trim off any whitespace:
    stringCompY = trim(stringCompY);
    // Convert to an float and map to the screen height, then save in buffer:    
    compY[compY.length-1] = map(PApplet.parseFloat(stringCompY), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the kalman filter x-axis */
  if (stringKalmanX != null) {
    // Trim off any whitespace:
    stringKalmanX = trim(stringKalmanX);
    // Convert to an float and map to the screen height, then save in buffer:    
    kalmanX[kalmanX.length-1] = map(PApplet.parseFloat(stringKalmanX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the kalman filter y-axis */
  if (stringKalmanY != null) {
    // Trim off any whitespace:
    stringKalmanY = trim(stringKalmanY);
    // Convert to an float and map to the screen height, then save in buffer:    
    kalmanY[kalmanY.length-1] = map(PApplet.parseFloat(stringKalmanY), minAngle, maxAngle, 0, height);
  }
}
public void drawAxisX() {
  /* Draw gyro x-axis */
  noFill();
  stroke(255,255,0); // Yellow
  // Redraw everything
  beginShape();
  for(int i = 0; i<gyroX.length;i++)
    vertex(i,gyroX[i]);
  endShape();
  // Put all data one array back
  for(int i = 1; i<gyroX.length;i++)
    gyroX[i-1] = gyroX[i];   
   
  /* Draw acceleromter x-axis */
  noFill();
  stroke(0,255,0); // Green
  // Redraw everything
  beginShape();
  for(int i = 0; i<accX.length;i++)
    vertex(i,accX[i]);  
  endShape();
  // Put all data one array back
  for(int i = 1; i<accX.length;i++)
    accX[i-1] = accX[i];   
   
  /* Draw complementary filter x-axis */
  noFill();
  stroke(0,0,255); // Blue
  // Redraw everything
  beginShape();
  for(int i = 0; i<compX.length;i++)
    vertex(i,compX[i]);
  endShape();
  // Put all data one array back
  for(int i = 1; i<compX.length;i++)
    compX[i-1] = compX[i];  
   
  /* Draw kalman filter x-axis */
  noFill();
  stroke(255,0,0);// Red
  // Redraw everything
  beginShape();
  for(int i = 0; i<kalmanX.length;i++)
    vertex(i,kalmanX[i]);  
  endShape();
  // Put all data one array back
  for(int i = 1; i<kalmanX.length;i++)
    kalmanX[i-1] = kalmanX[i];
}

public void drawAxisY() {
  /* Draw gyro y-axis */
  noFill();
  stroke(255,255,0); // Yellow
  // Redraw everything
  beginShape();
  for(int i = 0; i<gyroY.length;i++)
    vertex(i,gyroY[i]);
  endShape();
  // Put all data one array back
  for(int i = 1; i<gyroY.length;i++)
   gyroY[i-1] = gyroY[i];
   
  /* Draw acceleromter y-axis */
  noFill();
  stroke(0,255,0); // Green
  // Redraw everything
  beginShape();
  for(int i = 0; i<accY.length;i++)
    vertex(i,accY[i]);
  endShape();
  // Put all data one array back
  for(int i = 1; i<accY.length;i++)
    accY[i-1] = accY[i];
   
  /* Draw complementary filter y-axis */
  noFill();
  stroke(0,0,255); // Blue
  // Redraw everything
  beginShape();
  for(int i = 0; i<compY.length;i++)
    vertex(i,compY[i]);
  endShape();
  // Put all data one array back
  for(int i = 1; i<compY.length;i++)
    compY[i-1] = compY[i];
  
  /* Draw kalman filter y-axis */
  noFill();
  stroke(255,0,0); // Red
  // Redraw everything
  beginShape();
  for(int i = 0; i<kalmanY.length;i++)
    vertex(i,kalmanY[i]);
  endShape();
  // Put all data one array back
  for(int i = 1; i<kalmanY.length;i++)
    kalmanY[i-1] = kalmanY[i];
}    

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Graph" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
