/* Boids - v1.1 - 2013/12/29 */
   
// Global variables
ArrayList agents;
ArrayList objs;

// Setup the Processing Canvas
void setup() {
  size(900, 200);
  frameRate(40);
  
  initAgents(3, 0);
  initObj(3);
}

// Main draw loop
void draw() {
  background(255);
  update();
  render();
}

void initAgents(int numOfAgent, int numOfPrey) {
  agents = new ArrayList();
  // Add boids
  for (int i = 0; i < numOfAgent; i++) {
    Agent boid = new Agent(random(width), random(height), 1);
    agents.add(boid);
  }
  // Add predator
  for (int i = 0; i < numOfPrey; i++) {
    Agent predator = new Agent(random(width), random(height), 2);
    agents.add(predator);
  }
}

void initObj(int numOfObj) {
  objs = new ArrayList();
  // Add objects
  for (int i = 0; i < numOfObj; i++) {
    Obj obj = new Obj(random(1, width), random(1, height), random(50, 100), round(random(1, 2)));
    objs.add(obj);
  }
}

void update() {
  // Update agent state
  for (int i = 0; i < agents.size(); i++) {
    Agent boid = (Agent) agents.get(i);    
    boid.acc.set(0, 0, 0); // Reset accelertion to 0 each cycle
    steer(boid); // Update steering with approprate behavior
    boid.vel.add(boid.acc); // Update velocity
    /*switch (action) {
      case 1: vel.limit(maxpursue); break; // Limit pursue speed
      case 2: vel.limit(maxevade); break; // Limit evade speed
      default: vel.limit(maxspeed); break; // Limit speed      
    }*/
    boid.pos.add(boid.vel); // Move agent
    bounding(boid); // Wrap around the screen or else...
  }
}

void steer(Agent boid) {
    PVector wan = wander(boid);
    //PVector avo = avoid(boid);
    //PVector flo = new PVector();
    //PVector avo = new PVector();
    //PVector pur = new PVector();
    //PVector eva = new PVector();
    //PVector arr = new PVector();
    //PVector dep = new PVector();
    
    boid.acc.add(wan);
    boid.acc.limit(boid.maxforce); // Limit to maximum steering force
}

PVector seek(Agent boid, PVector target) {
  PVector steer; // The steering vector
  PVector desired = PVector.sub(target, boid.pos); // A vector pointing from current location to the target
  float distance = mag2(desired); // Distance from the target is the magnitude of the vector
  // If the distance is greater than 0, calc steering (otherwise return zero vector)
  if (distance > 0) {
    desired.normalize(); // Normalize desired

    desired.mult(boid.maxforce);

    steer = PVector.sub(desired, boid.vel); // Steering = Desired minus Velocity
  }
  else {
    steer = new PVector(0, 0);
  }
  return steer;
}

PVector wander(Agent boid) {
  float NRadius = 0.3; // Wander: radius of wander noise circle
  float WRadius = 40.0; // Wander: radius of wandering circle
  float KWander = 1.0;
  boid.wdelta += random(-NRadius, NRadius); // Determine noise ratio
  // Calculate the new location to steer towards on the wander circle
  PVector center = boid.vel.get(); // Get center of wander circle
  center.mult(60.0); // Multiply by distance
  center.add(boid.pos); // Make it relative to boid's location
  // Apply offset to get new target    
  PVector offset = new PVector(WRadius*cos(boid.wdelta), WRadius*sin(boid.wdelta));
  PVector target = PVector.add(center, offset); // Determine new target
  // Steer toward new target    
  PVector steer = seek(boid, target); // Steer towards it 
  steer.mult(KWander); // Weight  
  return steer;
}

void bounding(Agent boid) {
  if (boid.pos.x < -boid.mass) boid.pos.x = width + boid.mass;
  if (boid.pos.y < -boid.mass) boid.pos.y = height + boid.mass;
  if (boid.pos.x > width + boid.mass) boid.pos.x = -boid.mass;
  if (boid.pos.y > height + boid.mass) boid.pos.y = -boid.mass;
}

void render() {
  // Render agents
  for (int i = 0; i < agents.size(); i++) {
    Agent boid = (Agent) agents.get(i);    
    if (boid.type == 1) {
      fill(156, 206, 255);
      stroke(16, 16, 222);
      ellipse(boid.pos.x, boid.pos.y, boid.mass, boid.mass);
      PVector dir = boid.vel.get();
      dir.normalize();
      line(boid.pos.x, boid.pos.y, boid.pos.x + dir.x*10, boid.pos.y + dir.y*10);
      
      /***debug***/
      // Velocity
      stroke(16, 148, 16);
      line(boid.pos.x, boid.pos.y, boid.pos.x + boid.vel.x*4, boid.pos.y + boid.vel.y*4);
      // Steering
      stroke(255, 0, 0);
      line(boid.pos.x, boid.pos.y, boid.pos.x + boid.acc.x*20, boid.pos.y + boid.acc.y*20);
      // Neighborhood radius
      fill(239, 239, 239, 10);
      stroke(132, 132, 132);
      ellipse(boid.pos.x, boid.pos.y, 80.0*2, 80.0*2);
      fill(100, 100, 100, 30);
      noStroke();
      ellipse(boid.pos.x, boid.pos.y, 20.0*2, 20.0*2);
      stroke(255, 0, 0);
      noFill();
    }
    else if (boid.type == 2) {
      // Draw a triangle rotated in the direction of velocity        
      float theta = heading2D(boid.vel) + radians(90);
      pushMatrix();
      translate(boid.pos.x, boid.pos.y);
      rotate(theta);
      fill(220, 0, 0);
      noStroke();
      beginShape(TRIANGLES);
      vertex(0, -boid.mass);
      vertex(-3, boid.mass);
      vertex(3, boid.mass);
      endShape();
      popMatrix();
    }
   }
  
  // Render objects
  for (int i = 0; i < objs.size(); i++) {
    Obj obj = (Obj) objs.get(i);    
    if (obj.type == 1) {
      fill(200, 180, 160);
      stroke(50, 30, 20);
      ellipse(obj.pos.x, obj.pos.y, obj.mass, obj.mass);
    }
    else if (obj.type == 2) {
      fill(120, 190, 150);
      stroke(80, 70, 40);
      ellipse(obj.pos.x, obj.pos.y, obj.mass, obj.mass);
    }
  }
  // Render info
  fill(0);
  text("FPS: " + frameRate, 15, 20);
}

float heading2D(PVector vec) { return -1*((float) Math.atan2(-vec.y, vec.x)); }
float mag2(PVector vec) { return (vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); }

class Agent {
  float mass = 10.0;
  PVector pos; // Position
  PVector vel; // Velocity
  PVector acc; // Acceleration
  int type; // Agent type
  float wdelta; // Wander delta
  int action; // Current action
  float maxspeed = 5.0; // Maximum speed
  float maxforce = 3.0; // Maximum steering force
  float maxpursue; // Maximum steering force
  float maxevade; // Maximum steering force

  Agent(float px, float py, int t) {
    mass = 10.0;
    pos = new PVector(px, py);
    vel = new PVector(random(-5, 5), random(-5, 5));
    acc = new PVector();
    type = t;
    wdelta = 0.0;
    action = 0;
    maxspeed = 5.0;
    maxforce = 3.0;
    maxpursue = 20.0;
    maxevade = 10.0;
  }
}

class Obj {
  PVector pos;
  float mass;
  int type;

  Obj(float px, float py, float m, int t) {
    pos = new PVector(px, py);
    mass = m;
    type = t;
  }
}

