//Inverse Kinematics
//CSCI 5611 IK [Solution]
// Stephen J. Guy <sjguy@umn.edu>
IKArm arm1 = new IKArm();

Ball ball = new Ball(35);

PImage img;
PImage poster;

void setup(){
  size(1000,600,P3D);
  surface.setTitle("Inverse Kinematics [CSCI 5611]");
  img = loadImage("background.png");
  poster = loadImage("cat.png");
  arm1.root = new Vec2(100, 550);
}

boolean spaceDown = false;
char arm = 'o';
char arm1Key = 'o';

float value = 0;
void draw(){
  background(img);
  image(poster, 800, 100, 100,100);
  
  lights();
  
  // platform
  pushMatrix();
  fill(121, 68, 59);
  translate(500,575);
  box(900,10,250);
  popMatrix();
  
  // rails
  pushMatrix();
  fill(220,220,220);
  translate(500,565);
  box(800,5,5);
  popMatrix();
  
  arm1.fk();
  arm1.solve(arm, spaceDown, ball);
  arm1.updatePos(arm1Key, 1/frameRate);
  arm1.drawIKArm(color(0,0,204), color(255,51,51));
  
  ball.drawBall();

  
}

void mousePressed() {
  if (mouseButton == LEFT) {
    arm = 'l';  
  }
  if (mouseButton == RIGHT) {
    arm = 'r';  
  }
}

void mouseReleased() {
  if (mouseButton == LEFT) {
    arm = 'o';  
  }
  if (mouseButton == RIGHT) {
    arm = 'o';  
  }
}

void keyPressed() {
  if (key == 'a')
    arm1Key = 'a';
  if (key == 'd')
    arm1Key = 'd';
  if (key == ' ')
    spaceDown = true;
  if (key == 'q')
    value += 100;
  if (key == 'e')
    value -= 100;
}

void keyReleased() {
  if (key == 'a')
    arm1Key = 'o';
  if (key == 'd')
    arm1Key = 'o';
  if (key == ' ')
    spaceDown = false;
}

public class Ball {
  Vec2 pos = new Vec2(500, 200);
  float radius = 0;
  
  public Ball(float radius) {
    this.radius = radius;
  }
  
  void drawBall() {
    pushMatrix();
    fill(0,155,119);
    translate(pos.x, pos.y);
    sphere(radius);
    popMatrix();
  }
}
