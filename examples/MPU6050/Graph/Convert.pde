//convert all axis
final int minAngle = -180;
final int maxAngle = 180;

void convert() {
  /* Convert the gyro x-axis */
  if (stringGyroX != null) {
    stringGyroX = trim(stringGyroX); // Trim off any whitespace
    gyroX[gyroX.length - 1] = map(float(stringGyroX), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the gyro y-axis */
  if (stringGyroY != null) {
    stringGyroY = trim(stringGyroY); // Trim off any whitespace
    gyroY[gyroY.length - 1] = map(float(stringGyroY), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the accelerometer x-axis */
  if (stringAccX != null) {
    stringAccX = trim(stringAccX); // Trim off any whitespace
    accX[accX.length - 1] = map(float(stringAccX), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the accelerometer y-axis */
  if (stringAccY != null) {
    stringAccY = trim(stringAccY); // Trim off any whitespace
    accY[accY.length - 1] = map(float(stringAccY), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the complementary filter x-axis */
  if (stringCompX != null) {
    stringCompX = trim(stringCompX); // Trim off any whitespace
    compX[compX.length - 1] = map(float(stringCompX), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the complementary filter x-axis */
  if (stringCompY != null) {
    stringCompY = trim(stringCompY); // Trim off any whitespace
    compY[compY.length - 1] = map(float(stringCompY), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the kalman filter x-axis */
  if (stringKalmanX != null) {
    stringKalmanX = trim(stringKalmanX); // Trim off any whitespace
    kalmanX[kalmanX.length - 1] = map(float(stringKalmanX), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the kalman filter y-axis */
  if (stringKalmanY != null) {
    stringKalmanY = trim(stringKalmanY); // Trim off any whitespace
    kalmanY[kalmanY.length - 1] = map(float(stringKalmanY), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }
}
