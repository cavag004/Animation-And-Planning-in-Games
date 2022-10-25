//Create Window
String windowTitle = "Swinging Rope";

Camera camera;
//OrbitCamera cam;
Obstacle obstacle;
ArrayList<Rope> ropesList = new ArrayList();

void setup() {
  size(400, 500, P3D);
  smooth();
  surface.setTitle(windowTitle);
  camera = new Camera(200,200,700);
  //cam = new OrbitCamera(200,250,0);
  obstacle = new Obstacle(new Vec2(200,250), 50);
  
  for (int i = 0; i < 10; i++) {
    float ropePosOffset = i * 10;
    Rope rope = new Rope(new Vec2(0,400), new Vec2(200+ropePosOffset,50), 10, 2.5, 10, 1, 200, 100);
    rope = initScene(rope);
    ropesList.add(rope);
  } 
}

void updateRopes(float dt, ArrayList<Rope> ropesList) {
  //Reset accelerations each timestep (momenum only applies to velocity)
  for (Rope rope : ropesList) {
    for (int i = 0; i < rope.numNodes; i++){
      rope.acc[i] = new Vec2(0,0);
      rope.acc[i].add(rope.gravity);
      rope.acc[i].add(rope.drag[i]);
      rope.drag[i] = new Vec2(0,0);

    }
  
    //Compute (damped) Hooke's law for each spring (Vertical)
    for (int i = 0; i < rope.numNodes-1; i++) {
      if (rope.pos[i] == null || rope.pos[i+1] == null)
        continue;
      Vec2 diff = rope.pos[i+1].minus(rope.pos[i]);
      float stringF = -rope.k*(diff.length() - rope.restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec2 stringDir = diff.normalized();
      float projVbot = dot(rope.vel[i], stringDir);
      float projVtop = dot(rope.vel[i+1], stringDir);
      float dampF = -rope.kv*(projVtop - projVbot);
      
      Vec2 force = stringDir.times(stringF+dampF);
      rope.acc[i].add(force.times(-1.0/rope.mass)); 
      rope.acc[i+1].add(force.times(1.0/rope.mass));
    }
  }
  
  //Compute (damped) Hooke's law for each spring (Horizontal)
  for (int i = 0; i < ropesList.size()-1; i++) {
    Rope r1 = ropesList.get(i);
    Rope r2 = ropesList.get(i+1);
    for (int j = 0; j < r1.numNodes; j++) {
      if ((r1.pos[j] == null || r1.pos[j+1] == null) ||
         (r2.pos[j] == null || r2.pos[j+1] == null))
        continue;
      Vec2 diff = r2.pos[j].minus(r1.pos[j]);
      float stringF = -r1.k*(diff.length() - r1.restLen);
      
      Vec2 stringDir = diff.normalized();
      float projVbot = dot(r1.vel[j], stringDir);
      float projVtop = dot(r2.vel[j], stringDir);
      float dampF = -r1.kv*(projVtop - projVbot);
      
      Vec2 force = stringDir.times(stringF+dampF);
      r1.acc[j].add(force.times(-1.0/r1.mass)); 
      r2.acc[j].add(force.times(1.0/r1.mass));
    }
  }

  //Eulerian integration
  for (Rope rope : ropesList) {
    for (int i = 1; i < rope.numNodes; i++){
      if (rope.pos[i] == null)
        continue;
      rope.vel[i].add(rope.acc[i].times(dt));
      rope.pos[i].add(rope.vel[i].times(dt));
    }
    
    for (int i = 1; i < rope.numNodes; i++) {
      if (rope.pos[i] == null || rope.pos[i-1] == null)
        continue;
      if (rope.pos[i].distanceTo(rope.pos[i-1]) > 50) {
        rope.pos[i] = null;
        rope.acc[i].x = 0;
        rope.acc[i].y = 0;
      }
    }
  }
  
  //Collision detection and response
  for (Rope rope : ropesList) {
    for (int i = 0; i < rope.numNodes; i++){
      if (rope.pos[i] == null)
        continue;
      if (rope.pos[i].y+rope.radius > rope.floor){
        rope.vel[i].y *= -.9;
        rope.pos[i].y = rope.floor - rope.radius;
      }
      if (rope.pos[i].distanceTo(obstacle.pos) < (obstacle.radius + rope.radius)) {
        Vec2 normal = (rope.pos[i].minus(obstacle.pos)).normalized();
        rope.pos[i] = obstacle.pos.plus(normal.times(obstacle.radius + rope.radius).times(1.01));
        Vec2 velNormal = normal.times(dot(rope.vel[i], normal));
        rope.vel[i].subtract(velNormal.times(1+ .7));
      }
    }
  }
  
  for (Rope rope : ropesList) {
    for (int i = 0; i < rope.numNodes; i++){
      // add drag to the simulation
      Vec2 dragForce = rope.vel[i].times(-1.25);
      rope.drag[i].add(dragForce);
      
      //if (rope.wind > 0) {
      //  Vec2 windForce = new Vec2(abs(rope.acc[i].x * rope.wind * 0.01), 0);
      //  rope.drag[i].add(windForce);
      //}
      
    }
  }
}

Rope initScene(Rope rope){
  rope.floor = 500;
  rope.obstacle = obstacle;
  
  for (int i = 0; i < rope.numNodes; i++){
    rope.pos[i] = new Vec2(0,0);
    rope.pos[i].x = rope.stringTop.x + 18*i; //Make each node a little more to the right
    rope.pos[i].y = rope.stringTop.y + 8*i; //Make each node a little lower
    rope.vel[i] = new Vec2(0,0);
  }
  return rope;
}

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
boolean rightMousePressed = false;
boolean deleteNode = false;
void draw() {
  lights();
  camera.updateCamera(1.0/frameRate);
  //cam.updateCamera();
  //beginCamera();
  //camera(200,200,700,200,250,0, 0, 0, 1);
  //endCamera();
  
  background(255,255,255);
  for (int i = 0; i < 20; i++)
    if (!paused) updateRopes(1/(20*frameRate), ropesList);

  noStroke();
  for (int i = 0; i < ropesList.size()-1; i++) {
    Rope r1 = ropesList.get(i);
    Rope r2 = ropesList.get(i+1);
    for (int j = 0; j < r1.numNodes-1; j++) {
      if ((r1.pos[j] == null || r1.pos[j+1] == null) ||
         (r2.pos[j] == null || r2.pos[j+1] == null))
         continue;
       beginShape();
       fill(121, 208, 232);
       if (j%2==0) fill(99, 68, 171);
       vertex(r1.pos[j].x, r1.pos[j].y);
       vertex(r2.pos[j].x, r2.pos[j].y);
       vertex(r2.pos[j+1].x, r2.pos[j+1].y);
       vertex(r1.pos[j+1].x, r1.pos[j+1].y);
       endShape();
    }
  }

  // draw the obstacle
  pushMatrix();
  fill(54,220,154);
  translate(obstacle.pos.x, obstacle.pos.y, 0);
  sphere(obstacle.radius);
  popMatrix();
  if (rightMousePressed)
    obstacle.updateObstacle();
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  if (key == ' ')
    paused = !paused;
  if (key == 'r') {
    paused = true;
    for (Rope rope : ropesList) 
      initScene(rope);
  }
  if (key =='z') {
    for (Rope rope : ropesList) {
      if (rope.wind > 0)
        rope.wind -= 10; 
    }
  }
  if (key =='x') {
    for (Rope rope : ropesList) {
      rope.wind += 10;
      println(rope.wind);
    }
  }
  camera.HandleKeyPressed();
}

void keyReleased() {
  camera.HandleKeyReleased();
}

void mousePressed() {
  if (mouseButton == LEFT) {
    Vec2 mousePos = new Vec2(mouseX, mouseY);
    for (Rope rope : ropesList) {
      for (int i = 0; i < rope.numNodes; i++) {
        if (rope.pos[i] == null)
          continue;
        if (mousePos.distanceTo(rope.pos[i]) < rope.radius + 5) {
          rope.pos[i] = null;
          rope.acc[i].x = 0;
          rope.acc[i].y = 0;
          continue;
        }
      }
    }
  }
  if (mouseButton == RIGHT) {
    rightMousePressed = true;
  }
}

void mouseReleased() {
  if (mouseButton == RIGHT) {
    rightMousePressed = false;
  }
}

public class Obstacle {
  Vec2 pos = new Vec2(0,0);
  Vec2 vel = new Vec2(0,0);
  float radius;
  float speed = 5;
  
  public Obstacle(Vec2 pos, float radius) {
    this.pos = pos;
    this.radius = radius;
  }
  
  void updateObstacle() {
    Vec2 mousePos = new Vec2(mouseX, mouseY);
    Vec2 dir = mousePos.minus(pos);
    
    if (dir.length() < speed/2) { // Makes to circle stop jittering
      dir.x = 0;
      dir.y = 0;
    }
    
    if (dir.length() > 0)
      dir.normalize();
    vel = dir.times(speed);
    pos.add(vel);
  }
}
