void drawAxisX() {
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

void drawAxisY() {
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

