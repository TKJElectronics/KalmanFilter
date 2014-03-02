void drawAxisX() {
  /* Draw gyro x-axis */
  noFill();
  stroke(255, 255, 0); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, gyroX[0]);
  for (int i = 1; i < gyroX.length; i++) {
    if ((gyroX[i] < height/4 && gyroX[i - 1] > height/4*3) || (gyroX[i] > height/4*3 && gyroX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, gyroX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < gyroX.length;i++)
    gyroX[i-1] = gyroX[i];

  /* Draw acceleromter x-axis */
  noFill();
  stroke(0, 255, 0); // Green
  // Redraw everything
  beginShape();
  vertex(0, accX[0]);
  for (int i = 1; i < accX.length; i++) {
    if ((accX[i] < height/4 && accX[i - 1] > height/4*3) || (accX[i] > height/4*3 && accX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, accX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < accX.length;i++)
    accX[i-1] = accX[i];

  /* Draw complementary filter x-axis */
  noFill();
  stroke(0, 0, 255); // Blue
  // Redraw everything
  beginShape();
  vertex(0, compX[0]);
  for (int i = 1; i < compX.length; i++) {
    if ((compX[i] < height/4 && compX[i - 1] > height/4*3) || (compX[i] > height/4*3 && compX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, compX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < compX.length; i++)
    compX[i-1] = compX[i];

  /* Draw kalman filter x-axis */
  noFill();
  stroke(255, 0, 0);// Red
  // Redraw everything
  beginShape();
  vertex(0, kalmanX[0]);
  for (int i = 1; i < kalmanX.length; i++) {
    if ((kalmanX[i] < height/4 && kalmanX[i - 1] > height/4*3) || (kalmanX[i] > height/4*3 && kalmanX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, kalmanX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < kalmanX.length; i++)
    kalmanX[i-1] = kalmanX[i];
}

void drawAxisY() {
  /* Draw gyro y-axis */
  noFill();
  stroke(255, 0, 255); // Purble
  // Redraw everything
  beginShape();
  vertex(0, gyroY[0]);
  for (int i = 1; i < gyroY.length; i++) {
    if ((gyroY[i] < height/4 && gyroY[i - 1] > height/4*3) || (gyroY[i] > height/4*3 && gyroY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, gyroY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < gyroY.length;i++)
   gyroY[i-1] = gyroY[i];

  /* Draw acceleromter y-axis */
  noFill();
  stroke(0, 255, 255); // Light blue
  // Redraw everything
  beginShape();
  vertex(0, accY[0]);
  for (int i = 1; i < accY.length; i++) {
    if ((accY[i] < height/4 && accY[i - 1] > height/4*3) || (accY[i] > height/4*3 && accY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, accY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < accY.length;i++)
    accY[i-1] = accY[i];

  /* Draw complementary filter y-axis */
  noFill();
  stroke(124, 252, 0); // Lawn Green
  // Redraw everything
  beginShape();
  vertex(0, compY[0]);
  for (int i = 1; i < compY.length; i++) {
    if ((compY[i] < height/4 && compY[i - 1] > height/4*3) || (compY[i] > height/4*3 && compY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, compY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < compY.length;i++)
    compY[i-1] = compY[i];

  /* Draw kalman filter y-axis */
  noFill();
  stroke(0, 0, 0); // Black
  // Redraw everything
  beginShape();
  vertex(0, kalmanY[0]);
  for (int i = 1; i < kalmanY.length; i++) {
    if ((kalmanY[i] < height/4 && kalmanY[i - 1] > height/4*3) || (kalmanY[i] > height/4*3 && kalmanY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, kalmanY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i<kalmanY.length;i++)
    kalmanY[i-1] = kalmanY[i];
}
