Vec2[] points = new Vec2[5];
float[] lengths = {100,100,100,100};
int MAX_ITERATIONS = 20;

void setup(){
  size(640,480,P3D);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  strokeWeight(2);
  points[0] = new Vec2(0,0);
  points[1] = new Vec2(100,0);
  points[2] = new Vec2(100,100);
  points[3] = new Vec2(200,100);
  points[4] = new Vec2(200,200);
  
  for (Vec2 point : points) {
    point.add(new Vec2(320,480));  
  }
  
  
}

void forward_reach(Vec2[] points, float[] lengths, Vec2 goal) {
  for (int i = 0; i < points.length - 1; i++) {
    Vec2 v_ps = points[i+1].minus(points[i]);
    goal = points[i+1];
    points[i+1] = points[i].plus(v_ps.normalized().times(lengths[i]));
  }
}

void backwards_reach(Vec2[] points, float[] lengths, Vec2 goal) {
  for (int i = points.length-1; i > 0; i--) {
    Vec2 v_pt = points[i-1];
    points[i] = goal;
    goal = points[i].minus(v_pt.normalized().times(lengths[i-1]));
  }
}

void fabrik(Vec2[] points, float[] lengths, Vec2 endpoint, float delta) {
  int i = 0;
  while (points[points.length-1].distanceTo(endpoint) > delta && i < MAX_ITERATIONS) {
    Vec2 goal = endpoint;
    
    backwards_reach(points, lengths, goal);
    forward_reach(points, lengths, goal);
    
    i++;
  }
  
  for (Vec2 point : points) {
    println(point);  
  }
}

float armW = 20;
boolean paused = true;
void draw(){
  background(255,255,255);
  
  Vec2 goal = new Vec2(mouseX, mouseY);
  if (!paused) {
    fabrik(points, lengths, goal, 10); 
  }
  
  pushMatrix();
  stroke(0);
  strokeWeight(10);
  circle(points[points.length-1].x, points[points.length-1].y, 5);
  for (int i = 0; i < points.length-1; i++) {
    circle(points[i].x, points[i].y, 5);
    line(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
  }
  popMatrix();
  
}

void keyPressed() {
  if (key == ' ') {
    paused = !paused;  
  }
}




// Vector Library

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
