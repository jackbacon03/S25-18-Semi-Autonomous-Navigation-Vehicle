import processing.serial.*;

color StopColor = color(255, 100, 100);   
color upColor = color(255, 165, 0);

color downColor = color(100, 255, 100); 
color leftColor = color(100, 100, 255);  
color rightColor = color(255, 255, 100); 
color centerColor = color(150, 150, 150); 
int selectedDirection = -1; 

void setup() {
  size(600, 600); // Canvas size
  noStroke();
  textAlign(CENTER, CENTER);
  textSize(16); // Set text size
}

void draw() {
  background(240);

  // Draw the D-pad with the updated colors
  drawDPad(200, 200, 200);
}

void drawDPad(int x, int y, int size) {
  int padSize = size / 3; // Size of each directional pad

  // Draw Up
  fill(selectedDirection == 0 ? lerpColor(upColor, color(255), 0.5) : upColor);
  rect(x + padSize, y, padSize, padSize);
  fill(0); // Text color
  text("UP", x + 1.5 * padSize, y + padSize / 2);

  // Draw Down
  fill(selectedDirection == 1 ? lerpColor(downColor, color(255), 0.5) : downColor);
  rect(x + padSize, y + 2 * padSize, padSize, padSize);
  fill(0); // Text color
  text("DOWN", x + 1.5 * padSize, y + 2.5 * padSize);

  // Draw Left
  fill(selectedDirection == 2 ? lerpColor(leftColor, color(255), 0.5) : leftColor);
  rect(x, y + padSize, padSize, padSize);
  fill(0); // Text color
  text("LEFT", x + padSize / 2, y + 1.5 * padSize);

  // Draw Right
  fill(selectedDirection == 3 ? lerpColor(rightColor, color(255), 0.5) : rightColor);
  rect(x + 2 * padSize, y + padSize, padSize, padSize);
  fill(0); // Text color
  text("RIGHT", x + 2.5 * padSize, y + 1.5 * padSize);

  // Draw Center
  fill(selectedDirection == 4 ? lerpColor(StopColor, color(255), 0.5) : StopColor);
  ellipse(x + 1.5 * padSize, y + 1.5 * padSize, padSize, padSize);
  fill(255); // Text color for center button
  text("STOP", x + 1.5 * padSize, y + 1.5 * padSize);
}

void keyPressed() {
  if (keyCode == UP) {
    selectedDirection = 0; // UP
  } else if (keyCode == DOWN) {
    selectedDirection = 1; // DOWN
  } else if (keyCode == LEFT) {
    selectedDirection = 2; // LEFT
  } else if (keyCode == RIGHT) {
    selectedDirection = 3; // RIGHT
  }else if (keyCode == 32) {
    selectedDirection = 4; // STOP
  }
}

void mousePressed() {
  int padSize = 200 / 3;
  int x = 200, y = 200; 

  // Check mouse position and set direction
  if (mouseX > x + padSize && mouseX < x + 2 * padSize && mouseY > y && mouseY < y + padSize) {
    selectedDirection = 0; // UP
  } else if (mouseX > x + padSize && mouseX < x + 2 * padSize && mouseY > y + 2 * padSize && mouseY < y + 3 * padSize) {
    selectedDirection = 1; // DOWN
  } else if (mouseX > x && mouseX < x + padSize && mouseY > y + padSize && mouseY < y + 2 * padSize) {
    selectedDirection = 2; // LEFT
  } else if (mouseX > x + 2 * padSize && mouseX < x + 3 * padSize && mouseY > y + padSize && mouseY < y + 2 * padSize) {
    selectedDirection = 3; // RIGHT
  } else if(dist(mouseX, mouseY, (x + 1.5 * padSize), (y + 1.5 * padSize)) <= (padSize / 2)){
      selectedDirection = 4;
    } else {
    selectedDirection = -1; // No direction
  }
}
