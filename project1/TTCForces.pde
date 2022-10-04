//static int maxNumAgents = 5;
//int numAgents = 5;

float k_goal = 25;  //TODO: Tune this parameter to agent stop naturally on their goals
float k_avoid = 100;
float k_avoid_obstacle = 200;

//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Agent primaryAgent, Agent secondaryAgent) {
  float combinedRadius = primaryAgent.radius + secondaryAgent.radius;
  Vec2 relativeVel = primaryAgent.vel.minus(secondaryAgent.vel);
  float ttc = rayCircleIntersectTime(secondaryAgent.pos, combinedRadius, primaryAgent.pos, relativeVel);
  
  return ttc;
}

float computeTTC(Agent primaryAgent, Circle circle) {
  float combinedRadius = primaryAgent.radius + circle.radius;
  Vec2 relativeVel = primaryAgent.vel.minus(new Vec2(0,0));
  float ttc = rayCircleIntersectTime(circle.pos, combinedRadius, primaryAgent.pos, relativeVel);
  
  return ttc;
}

void updateAgentGoal(Agent agent) {
  if (agent.path.size() > agent.pathID + 1) {
    if (agent.pos.distanceTo(nodePos[agent.path.get(agent.pathID)]) < 20) {
      agent.goalPos = nodePos[agent.path.get(agent.pathID + 1)];
      agent.pathID = agent.pathID + 1;
    } else {
      agent.goalPos = nodePos[agent.path.get(agent.pathID)];
    }
  } else {
    agent.goalPos = agent.endGoalPos;
  }
}

Vec2 computeAgentForces(Agent[] agentsList, Agent primaryAgent){
  Vec2 acc = new Vec2(0,0);
  
  updateAgentGoal(primaryAgent);
  
  Vec2 goalVel = primaryAgent.goalPos.minus(primaryAgent.pos);
  //if (goalVel.length() > goalSpeed)
  //  goalVel.setToLength(goalSpeed);
  Vec2 goalForce = goalVel.minus(primaryAgent.vel); 
  acc.add(goalForce.times(k_goal));
  
  for (Agent secondaryAgent : agentsList) {
    if (secondaryAgent.id == primaryAgent.id || primaryAgent.pos.distanceTo(secondaryAgent.pos) > 100)
      continue;
    float ttc = computeTTC(primaryAgent, secondaryAgent);
    Vec2 futurePos_id = primaryAgent.pos.plus(primaryAgent.vel.times(ttc));
    Vec2 futurePos_j = secondaryAgent.pos.plus(secondaryAgent.vel.times(ttc));
    Vec2 relativeVec = futurePos_id.minus(futurePos_j).normalized();
    
    Vec2 avoidForce = relativeVec.times((k_avoid * (1/ttc)));
    acc.add(avoidForce);
  }
  
  for (Circle circle : circlesList) {
    if (primaryAgent.pos.distanceTo(circle.pos) > 50)
      continue;
    float ttc = computeTTC(primaryAgent, circle);
    Vec2 futurePos_id = primaryAgent.pos.plus(primaryAgent.vel.times(ttc));
    Vec2 futurePos_j = circle.pos;
    Vec2 relativeVec = futurePos_id.minus(futurePos_j).normalized();
    
    Vec2 avoidForce = relativeVec.times((100 * (1/ttc)));

    acc.add(avoidForce);
  }
  
  return acc;
}


//Update agent positions & velocities based acceleration
void moveAgent(float dt, Agent[] agentsList){
  for (Agent agent : agentsList) {
    agent.acc = computeAgentForces(agentsList, agent);
    agent.vel.add(agent.acc.times(dt));
    agent.pos.add(agent.vel.times(dt));
  }
}

public class Agent {
  int id = -1;
  int pathID = 0;
  float radius = 0;
  Vec2 startPos = null;
  Vec2 goalPos = null;
  Vec2 endGoalPos = null;
  Vec2 pos = null;
  Vec2 vel = null;
  Vec2 acc = null;
  ArrayList<Integer> path;
  
  public Agent(int id, Vec2 startPos, Vec2 goalPos, Vec2 pos, Vec2 vel, Vec2 acc) {
    this.id = id;
    this.startPos = startPos;
    this.goalPos = goalPos;
    this.pos = pos;
    this.vel = vel;
    this.acc = acc;
  }
}

public class Circle {
  Vec2 pos = null;
  float radius = 0;
  
  public Circle(Vec2 pos, float radius) {
    this.pos = pos;
    this.radius = radius;
  }
}
