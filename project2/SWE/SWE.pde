import java.lang.Math;

//Create Window
String windowTitle = "SPH Fluid";
int windowX = 800;
int windowY = 500;
PImage bg;
PImage boat;

float scale = 1;
int n = 40; // number of cells
float dx = windowX / n; // width of each cell
float h[] = new float[n]; // height of each cell
float hu[] = new float[n]; // momentum of each cell

void setup() {
  size(800, 500);
  smooth();
  surface.setTitle(windowTitle);
  initScene();
  bg = loadImage("sky.png");
  boat = loadImage("boat.png");
}

void initScene() {
  for (int i = 0; i < n; i++) {
    h[i] = 300 + i*2;
  }
}

void simulateSWE(float dt) {
  float g = 1.0; // gravity
  float damp = 0.9;
  
  float dhdt[] = new float[n]; // height derivative
  float dhudt[] = new float[n]; // momentum derivative
  
  float h_mid[] = new float[n]; // height (midpoint)
  float hu_mid[] = new float[n]; // momentum (midpoint)
  
  float dhdt_mid[] = new float[n]; // height derivative (midpoint)
  float dhudt_mid[] = new float[n]; // momentum derivative (midpoint)
  
  // compute midpoint heights and momentums
  for (int i = 0; i < n-1; i++) {
    h_mid[i] = (h[i] + h[i+1])/2;
    hu_mid[i] = (hu[i] + hu[i+1])/2;
  }
  
  // compute derivates at midpoints
  for (int i = 0; i < n-1; i++) {
    // compute dh/dt (mid)
    float dhudx_mid = (hu[i+1] - hu[i])/dx;
    dhdt_mid[i] = -dhudx_mid;
      
    // compute dhu/dt (mid)   
    float dhu2dx_mid = ((hu[i+1]*hu[i+1])/h[i+1] - (hu[i]*hu[i])/h[i])/dx;
    float dgh2dx_mid = (g*h[i+1]*h[i+1] - h[i]*h[i])/dx;
    dhudt_mid[i] = -(dhu2dx_mid + 0.5*dgh2dx_mid);
  }
  
  // update midpoints for 1/2 a timestep based on midpoint derivatives
  for (int i = 0; i < n; i++) {
    h_mid[i] += dhdt_mid[i]*dt/2;
    hu_mid[i] += dhudt_mid[i]*dt/2;
  }
  
  // compute height and momentum updates (non-midpoint)
  for (int i = 1; i < n-1; i++) {
    // compute dh/dt
    float dhudx = (hu_mid[i] - hu_mid[i-1])/dx;
    dhdt[i] = -dhudx;
         
    // Compute dhu/dt
    float dhu2dx = ((hu_mid[i]*hu_mid[i])/h_mid[i] - (hu_mid[i-1]*hu_mid[i-1])/h_mid[i-1])/dx;
    float dgh2dx = g*((h_mid[i]*h_mid[i]) - (h_mid[i-1]*h_mid[i-1]))/dx;
    dhudt[i] = -(dhu2dx + 0.5*dgh2dx);
  }
  
  // update values (non-midpoint) based on full timestep
  for (int i = 0; i < n; i++) {
    h[i] += damp*dhdt[i]*dt;
    hu[i] += damp*dhudt[i]*dt;
  }
  
  // reflecting boundary conditions
  h[0] = h[1];
  h[n-1] = h[n-2];
  hu[0] = -hu[1];
  hu[n-1] = -hu[n-2];
}

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
boolean grabbed = false;
void draw() {
  //lights();
  background(bg);
  
  for (int i = 0; i < 20; i++)
    if (!paused) simulateSWE(1/frameRate);
  
  // info for displaying the boat
  float boatX = 0;
  float boatY = 0;
  noStroke();
  
  for (int j = 0; j < 3; j++) {
    if (j==0) fill(1, 72, 156);
    if (j==1) fill(0, 106, 178);
    if (j==2) fill(51, 139, 197);
    for (int i = 0; i < n-1; i++) {
      float x1 = i*dx;
      float x2 = (i+1)*dx;
      float y1 = (h[i]-1 + (j*40))*scale;
      float y2 = (h[i+1]-1 + (j*40))*scale;
      quad(x1,y1,x2,y2,x2,windowY,x1,windowY);
      if (i+1 == n-1) {
        quad(x1,y1,windowX,y2,windowX,windowY,x1,windowY); 
      }
      if (i == 20) {
        boatX = x1;
        boatY = y1;
      }
    }
  }
  
  image(boat, boatX, boatY-110);
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  if (key == ' ')
    paused = !paused;
  if (key == 'r') {
    //initScene(rope);
  }
  if (keyCode == RIGHT) {
    h[1] += 15;
    hu[1] += 0;
  }
  if (keyCode == LEFT) {
    h[n-2] += 15;
    hu[n-2] += 0;
  }
}

void mousePressed() {
  if (mouseButton == LEFT)
    grabbed = true;
}

void mouseReleased() {
  if (mouseButton == LEFT)
    grabbed = false;
}
