//convert all axis
final int minAngle = 0;
final int maxAngle = 360;

void convert() {
  /* Convert the gyro x-axis */
  if (stringGyroX != null) {
    // Trim off any whitespace:
    stringGyroX = trim(stringGyroX);
    // Convert to an float and map to the screen height, then save in buffer:    
    gyroX[gyroX.length-1] = map(float(stringGyroX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the gyro y-axis */
  if (stringGyroY != null) {    
    // Trim off any whitespace:
    stringGyroY = trim(stringGyroY);
    // Convert to an float and map to the screen height, then save in buffer:   
    gyroY[gyroY.length-1] = map(float(stringGyroY), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the accelerometer x-axis */
  if (stringAccX != null) {
    // Trim off any whitespace:
    stringAccX = trim(stringAccX);
    // Convert to an float and map to the screen height, then save in buffer:    
    accX[accX.length-1] = map(float(stringAccX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the accelerometer y-axis */
  if (stringAccY != null) {
    // Trim off any whitespace:
    stringAccY = trim(stringAccY);
    // Convert to an float and map to the screen height, then save in buffer:        
    accY[accY.length-1] = map(float(stringAccY), minAngle, maxAngle, 0, height);
  }

  /* Convert the complementary filter x-axis */
  if (stringCompX != null) {
    // Trim off any whitespace:
    stringCompX = trim(stringCompX);
    // Convert to an float and map to the screen height, then save in buffer:    
    compX[compX.length-1] = map(float(stringCompX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the complementary filter x-axis */
  if (stringCompY != null) {
    // Trim off any whitespace:
    stringCompY = trim(stringCompY);
    // Convert to an float and map to the screen height, then save in buffer:    
    compY[compY.length-1] = map(float(stringCompY), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the kalman filter x-axis */
  if (stringKalmanX != null) {
    // Trim off any whitespace:
    stringKalmanX = trim(stringKalmanX);
    // Convert to an float and map to the screen height, then save in buffer:    
    kalmanX[kalmanX.length-1] = map(float(stringKalmanX), minAngle, maxAngle, 0, height);
  }
  
  /* Convert the kalman filter y-axis */
  if (stringKalmanY != null) {
    // Trim off any whitespace:
    stringKalmanY = trim(stringKalmanY);
    // Convert to an float and map to the screen height, then save in buffer:    
    kalmanY[kalmanY.length-1] = map(float(stringKalmanY), minAngle, maxAngle, 0, height);
  }
}
