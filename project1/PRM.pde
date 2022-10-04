//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestNode(startPos, nodePos, numNodes);
  int goalID = closestNode(goalPos, nodePos, numNodes);
  
  
  path = runBFS(nodePos, numNodes, startID, goalID);
  //path = runUCS(nodePos, numNodes, startID, goalID, startPos, goalPos);  
  return path;
}

float getCost(Vec2 neighborNode, Vec2 startPos, Vec2 goalPos) {
  
  float distStartToCur = startPos.distanceTo(neighborNode);
  float distCurToGoal = neighborNode.distanceTo(goalPos);

  return distStartToCur + distCurToGoal; 
}

//BFS (Breadth First Search)
ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}


public class Node {
  public Vec2 nodePos;
  
  public Node(Vec2 nodePos) {
    this.nodePos = nodePos; 
  }
}
