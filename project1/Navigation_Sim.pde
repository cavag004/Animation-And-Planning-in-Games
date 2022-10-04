//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//This is a test harness designed to help you test & debug your PRM.

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests

PImage tennisball = null;

//Change the below parameters to change the scenario/roadmap size
int numObstacles = 50;
int numNodes  = 100;

int numAgents = 1;
float agentRad = 10;
float goalSpeed = 50;
  
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii
ArrayList<Circle> circlesList = new ArrayList();

Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++) {
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
    circlesList.add(new Circle(circlePos[i], circleRad[i]));
  }
  circleRad[0] = 30; //Make the first obstacle big
  circlesList.get(0).radius = 30;
}

//ArrayList<Integer> curPath;
Agent[] agentsList = new Agent[numAgents];

int strokeWidth = 2;
void setup(){
  size(1024,768,P3D);
  tennisball = loadImage("ball.png");
  
  placeRandomObstacles(numObstacles);
  
  for (int i = 0; i < numAgents; i++) {
    startPos = sampleFreePos();
    goalPos = sampleFreePos();
  
    generateRandomNodes(numNodes, circlePos, circleRad);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
    
    ArrayList<Integer> path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
    //agentPaths.add(path);
    
    // set agent velocity
    Vec2 vel = goalPos.minus(startPos);
    if (vel.length() > 0)
      vel.setToLength(goalSpeed);

    agentsList[i] = new Agent(i, startPos, goalPos, startPos.plus(new Vec2(1,1)), vel, new Vec2(0,0));
    agentsList[i].radius = agentRad;
    agentsList[i].endGoalPos = goalPos;
    agentsList[i].path = path;
  }
  
}

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  }
  return randPos;
}

boolean paused = true;
void draw(){
  lights();
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(200); //Grey background
  stroke(0,0,0);
  fill(255,255,255);
  
  translate(0, -50, -600);
  rotateX(PI/5);
  
  PShape cube = null;
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    //circle(c.x,c.y,r*2);
    cube = createShape(BOX,r*sqrt(2),r*sqrt(2),r*sqrt(2));
    cube.noFill();
    shapeMode(CENTER);
    shape(cube,c.x + ((r*sqrt(2))/2), c.y + ((r*sqrt(2))/2));
    
  }
  
  //Draw the first circle a little special b/c the user controls it
  fill(240);
  strokeWeight(2);
  circle(circlePos[0].x,circlePos[0].y,circleRad[0]*2);
  strokeWeight(1);
  
  //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++){
    //circle(nodePos[i].x,nodePos[i].y,5);
    //text(i,nodePos[i].x + 10,nodePos[i].y);
  }
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate, agentsList);
  }
  
  //Draw the green agents
  fill(20,200,150);
  for (Agent agent : agentsList) {
    //circle(agent.pos.x, agent.pos.y, agentRad*2);
    image(tennisball, agent.pos.x - agentRad, agent.pos.y - agentRad, agentRad*2, agentRad*2);
  }
  
  //Draw Mouse Pos
  //String mousePos = mouseX + ", " + mouseY; 
  //text(mousePos, mouseX, mouseY);
  
  //Draw graph
  //stroke(100,100,100,75);
  //strokeWeight(1);
  //for (int i = 0; i < numNodes; i++){
  //  for (int j : neighbors[i]){
  //    line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
  //  }
  //}
  
  //Draw Start and Goal
  fill(20,60,250);
  for (Agent agent : agentsList) {
    circle(agent.startPos.x, agent.startPos.y, 20);
  }
  //circle(nodePos[startNode].x,nodePos[startNode].y,20);
  //circle(startPos.x,startPos.y,20);
  fill(250,30,50);
  for (Agent agent : agentsList) {
    circle(agent.endGoalPos.x, agent.endGoalPos.y, 20);
  }
  //circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
  //circle(goalPos.x,goalPos.y,20);
  
  //if (curPath.size() >0 && curPath.get(0) == -1) return; //No path found
  for (Agent agent : agentsList) {
    if (agent.path.size() >0 && agent.path.get(0) == -1) return;
  }
  
  //Draw Planned Path
  stroke(20,255,40);
  strokeWeight(5);
  for (Agent agent : agentsList) {
    if (agent.path.size() == 0) {
      line(agent.startPos.x, agent.startPos.y, agent.goalPos.x, agent.goalPos.y);
    }
  }
  
  for (Agent agent : agentsList) {
    line(agent.startPos.x, agent.startPos.y, nodePos[agent.path.get(0)].x, nodePos[agent.path.get(0)].y);
    for (int i = 0; i < agent.path.size()-1; i++) {
      int curNode = agent.path.get(i);
      int nextNode = agent.path.get(i+1);
      line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
    }
    line(agent.endGoalPos.x, agent.endGoalPos.y, nodePos[agent.path.get(agent.path.size()-1)].x, nodePos[agent.path.get(agent.path.size()-1)].y);
  }
  
}

boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    setup();
    return;
  }
  
  if (key == ' ') {
    paused = !paused;
    return;
  }
  
  if (keyCode == SHIFT){
    shiftDown = true;
  }
  
  float speed = 10;
  if (shiftDown) speed = 30;
  if (keyCode == RIGHT){
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[0].x -= speed;
  }
  if (keyCode == UP){
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[0].y += speed;
  }
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  //for (Agent agent : agentsList) {
  //  agent.path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  //}
  //curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
}

void mousePressed(){
  if (mouseButton == RIGHT){
    startPos = new Vec2(mouseX, mouseY);
    //println("New Start is",startPos.x, startPos.y);
  }
  else{
    goalPos = new Vec2(mouseX, mouseY);
    //println("New Goal is",goalPos.x, goalPos.y);
  }
  //curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  //for (Agent agent : agentsList) {
  //  agent.path = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  //}
}
