public class IKArm {
  //Root
  Vec2 root = new Vec2(0,0);
  Vec2 goal = new Vec2(0,0);
  Vec2 vel = new Vec2(0,0);
  
  float armW = 20;
  
  //Upper Arm
  float l0 = 170; 
  float a0 = 0.3; //Shoulder joint
  
  //Lower Arm
  float l1 = 150;
  float a1 = 0.3; //Elbow joint
  float l1_2 = 150;
  float a1_2 = 0.3; //Elbow joint
  
  //Hand
  float l2 = 100;
  float a2 = 0.3; //Wrist joint
  float l2_2 = 100;
  float a2_2 = 0.3; //Wrist joint
  
  float l3 = 50;
  float a3 = 0.3;
  float l3_2 = 50;
  float a3_2 = 0.3;
  
  float l4 = 35;
  float a4 = 0.3;
  float l4_2 = 35;
  float a4_2 = 0.3;
  
  Vec2 start_l1,start_l2,start_l3,start_l4,endPoint;
  Vec2 start_l2_2,start_l3_2,start_l4_2,endPoint_2;
  
  void solve(char mouseDown, boolean spaceDown, Ball ball) {
    if (mouseDown != 'o')
      goal = new Vec2(mouseX, mouseY);
      
    
    if (endPoint.distanceTo(ball.pos) < ball.radius-15) {
      if (spaceDown) {
        ball.pos = endPoint;
      }
    }
    
    if (endPoint_2.distanceTo(ball.pos) < ball.radius-15) {
      if (spaceDown) {
        ball.pos = endPoint_2;
      }
    }
    
    if (ball.pos.y + ball.radius > 585)
      ball.pos.y = 585 - ball.radius;
      
    if (goal.y > 585)
      goal.y = 585;
   
    
    Vec2 startToGoal, startToEndEffector;
    float dotProd, angleDiff;
    
    
    if (mouseDown == 'l') {
      //Update grabber joint
      startToGoal = goal.minus(start_l4);
      startToEndEffector = endPoint.minus(start_l4);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a4 += angleDiff;
      else
        a4 -= angleDiff;
      a4 = 0;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update new joint
      startToGoal = goal.minus(start_l3);
      startToEndEffector = endPoint.minus(start_l3);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a3 += angleDiff;
      else
        a3 -= angleDiff;
      if (a3 > 1.57)
        a3 = 1.57;
      if (a3 < -1.57)
        a3 = -1.57;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update wrist joint
      startToGoal = goal.minus(start_l2);
      startToEndEffector = endPoint.minus(start_l2);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a2 += angleDiff;
      else
        a2 -= angleDiff;
      if (a2 > 1.57)
        a2 = 1.57;
      if (a2 < -1.57)
        a2 = -1.57;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update elbow joint
      startToGoal = goal.minus(start_l1);
      startToEndEffector = endPoint.minus(start_l1);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a1 += angleDiff;
      else
        a1 -= angleDiff;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update shoulder joint
      startToGoal = goal.minus(root);
      if (startToGoal.length() < .0001) return;
      startToEndEffector = endPoint.minus(root);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a0 += angleDiff;
      else
        a0 -= angleDiff;
      
      if (a0 > 0)
        a0 = 0;
      if (a0 < -3.14)
        a0 = -3.14;
      fk(); //Update link positions with fk (e.g. end effector changed)
    }
    
    if (mouseDown == 'r') {
      //Update grabber joint 2
      startToGoal = goal.minus(start_l4_2);
      startToEndEffector = endPoint_2.minus(start_l4_2);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a4_2 += angleDiff;
      else
        a4_2 -= angleDiff;
      a4_2 = 0;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update new joint
      startToGoal = goal.minus(start_l3_2);
      startToEndEffector = endPoint_2.minus(start_l3_2);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a3_2 += angleDiff;
      else
        a3_2 -= angleDiff;
      if (a3_2 > 1.57)
        a3_2 = 1.57;
      if (a3_2 < -1.57)
        a3_2 = -1.57;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update wrist joint
      startToGoal = goal.minus(start_l2_2);
      startToEndEffector = endPoint_2.minus(start_l2_2);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a2_2 += angleDiff;
      else
        a2_2 -= angleDiff;
      if (a2_2 > 1.57)
        a2_2 = 1.57;
      if (a2_2 < -1.57)
        a2_2 = -1.57;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update elbow joint
      startToGoal = goal.minus(start_l1);
      startToEndEffector = endPoint_2.minus(start_l1);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a1_2 += angleDiff;
      else
        a1_2 -= angleDiff;
      fk(); //Update link positions with fk (e.g. end effector changed)
      
      //Update shoulder joint
      startToGoal = goal.minus(root);
      if (startToGoal.length() < .0001) return;
      startToEndEffector = endPoint_2.minus(root);
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd,-1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        a0 += angleDiff;
      else
        a0 -= angleDiff;
      
      if (a0 > 0)
        a0 = 0;
      if (a0 < -3.14)
        a0 = -3.14;
      fk(); //Update link positions with fk (e.g. end effector changed)
    }

    //println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2);
  }
  
  void fk(){
    start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
    
    start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
    start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
    start_l4 = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
    endPoint = new Vec2(cos(a0+a1+a2+a3+a4)*l4,sin(a0+a1+a2+a3+a4)*l4).plus(start_l4);
    
    start_l2_2 = new Vec2(cos(a0+a1_2)*l1_2,sin(a0+a1_2)*l1_2).plus(start_l1);
    start_l3_2 = new Vec2(cos(a0+a1_2+a2_2)*l2_2,sin(a0+a1_2+a2_2)*l2_2).plus(start_l2_2);
    start_l4_2 = new Vec2(cos(a0+a1_2+a2_2+a3_2)*l3_2,sin(a0+a1_2+a2_2+a3_2)*l3_2).plus(start_l3_2);
    endPoint_2 = new Vec2(cos(a0+a1_2+a2_2+a3_2+a4_2)*l4_2,sin(a0+a1_2+a2_2+a3_2+a4_2)*l4_2).plus(start_l4_2);
  }
  
  void updatePos(char letter, float dt) {
    Vec2 newPos = root;
    if (letter == 'a')
      newPos = new Vec2(root.x-100, root.y);
    else if(letter == 'd')
      newPos = new Vec2(root.x+100, root.y);
    
    Vec2 dir = newPos.minus(root);
    if (dir.length() > 0)
      dir.normalize();
    vel = dir.times(2);
    root.add(vel);
    goal.add(vel);
    
  }
  
  void drawIKArm(color c1, color c2) {
    //lights();
    noStroke();
    
    pushMatrix();
    fill(128,128,128);
    translate(root.x,root.y+25);
    box(50);
    popMatrix();
    
    // first link
    pushMatrix();
    translate(root.x,root.y);
    fill(160, 32, 240);
    sphere(20);
    rotate(a0);
    translate(l0/2,0);
    fill(160, 32, 240);
    box(l0, armW, armW/2);
    translate(l0/2,0);
    fill(160, 32, 240);
    sphere(17);
    popMatrix();
    
    // second link
    pushMatrix();
    translate(start_l1.x,start_l1.y);
    rotate(a0+a1);
    translate(l1/2,0);
    fill(c1);
    box(l1, armW, armW/2);
    translate(l1/2,0);
    fill(c1);
    sphere(15);
    popMatrix();
    // 2
    pushMatrix();
    translate(start_l1.x,start_l1.y);
    rotate(a0+a1_2);
    translate(l1_2/2,0);
    fill(c2);
    box(l1_2, armW, armW/2);
    translate(l1_2/2,0);
    fill(c2);
    sphere(15);
    popMatrix();
    
    // third link
    pushMatrix();
    translate(start_l2.x,start_l2.y);
    rotate(a0+a1+a2);
    translate(l2/2,0);
    fill(c1);
    box(l2, armW, armW/2);
    translate(l2/2,0);
    fill(c1);
    sphere(13);
    popMatrix();
    // 2
    pushMatrix();
    translate(start_l2_2.x,start_l2_2.y);
    rotate(a0+a1_2+a2_2);
    translate(l2_2/2,0);
    fill(c2);
    box(l2_2, armW, armW/2);
    translate(l2_2/2,0);
    fill(c2);
    sphere(13);
    popMatrix();
    
    // fourth link
    pushMatrix();
    translate(start_l3.x,start_l3.y);
    rotate(a0+a1+a2+a3);
    translate(l3/2,0);
    fill(c1);
    box(l3, armW, armW/2);
    translate(l3/2,0);
    fill(c1);
    sphere(13);
    popMatrix(); 
    // 2
    pushMatrix();
    translate(start_l3_2.x,start_l3_2.y);
    rotate(a0+a1_2+a2_2+a3_2);
    translate(l3_2/2,0);
    fill(c2);
    box(l3_2, armW, armW/2);
    translate(l3_2/2,0);
    fill(c2);
    sphere(13);
    popMatrix();  
  }
}
