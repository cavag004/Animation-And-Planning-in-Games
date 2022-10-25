public class Rope {
  int maxNodes = 100;
  int numNodes = 0;
  
  float pos_z;
  Vec2 pos[] = new Vec2[maxNodes];
  Vec2 vel[] = new Vec2[maxNodes];
  Vec2 acc[] = new Vec2[maxNodes];
  Vec2 drag[] = new Vec2[maxNodes];
  int wind = 0;
  
  Obstacle obstacle;
  float floor;
  
  Vec2 gravity;
  float radius;
  Vec2 stringTop;
  float restLen;
  float mass;
  float k;
  float kv;

   public Rope(Vec2 gravity, Vec2 stringTop, int numNodes, float radius, float restLen, float mass, float k, float kv) {
     this.gravity = gravity;
     this.stringTop = stringTop;
     this.numNodes = numNodes;
     this.radius = radius;
     this.restLen = restLen;
     this.mass = mass;
     this.k = k;
     this.kv = kv;
     
     for (int i = 0; i < maxNodes; i++) {
       drag[i] = new Vec2(0,0);  
     }
   }
   
   void updateRope(float dt){
    //Reset accelerations each timestep (momenum only applies to velocity)
    for (int i = 0; i < numNodes; i++){
      acc[i] = new Vec2(0,0);
      acc[i].add(gravity);
    }
    
    //Compute (damped) Hooke's law for each spring
    for (int i = 0; i < numNodes-1; i++) {
      Vec2 diff = pos[i+1].minus(pos[i]);
      float stringF = -k*(diff.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec2 stringDir = diff.normalized();
      float projVbot = dot(vel[i], stringDir);
      float projVtop = dot(vel[i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      Vec2 force = stringDir.times(stringF+dampF);
      acc[i].add(force.times(-1.0/mass)); 
      acc[i+1].add(force.times(1.0/mass));
    }
  
    //Eulerian integration
    for (int i = 1; i < numNodes; i++){
      vel[i].add(acc[i].times(dt));
      pos[i].add(vel[i].times(dt));
    }
    
    //Collision detection and response
    for (int i = 0; i < numNodes; i++){
      if (pos[i].y+radius > floor){
        vel[i].y *= -.9;
        pos[i].y = floor - radius;
      }
      if (pos[i].distanceTo(obstacle.pos) < (obstacle.radius + radius)) {
        Vec2 normal = (pos[i].minus(obstacle.pos)).normalized();
        pos[i] = obstacle.pos.plus(normal.times(obstacle.radius + radius).times(1.01));
        Vec2 velNormal = normal.times(dot(vel[i], normal));
        vel[i].subtract(velNormal.times(1+ .7));
      }
    }
    
    //for (int i = 0; i < numNodes; i++){
    //  Vec2 dragForce = vel[i].times(-1);
    //  //dragForce.setToLength(10.0);
    //  pos[i].add(dragForce.times(dt));
    //}
  }
}
