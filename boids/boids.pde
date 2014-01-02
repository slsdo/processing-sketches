/* Boids - v1.1 - 2013/12/29 */
   
// Global variables
ArrayList agents;
float rad = 40.0;
// Setup the Processing Canvas
void setup() {
  size(900, 200);
  frameRate(40);
  
  initAgents(3, 0);
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
    updateweight(boid); // Updates weights
  }
}

void steer(Agent boid) {
  PVector mouse = new PVector(mouseX, mouseY);
  
  PVector wan = wander(boid);
  PVector avo = avoid(boid, mouse);
  //PVector flo = new PVector();
  //PVector avo = new PVector();
  //PVector pur = new PVector();
  //PVector eva = new PVector();
  //PVector arr = new PVector();
  //PVector dep = new PVector();
  
  
  wan.mult(boid.KWander); // Weight
  avo.mult(boid.KAvoid); // Weight
  
  boid.acc.add(wan);
  boid.acc.add(avo);
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
  return steer;
}

PVector avoid(Agent boid, PVector obstacle) {
  PVector steer = new PVector();
  
  // Distance between object and avoidance sphere
  float distance = dist2(obstacle, boid.pos);
  // If distance is less than the sum of the two radius, there is collision
  float bound = rad + boid.BRadius;
  if (distance < bound*bound) {
    boid.KAvoid = 10.0;
    boid.KWander = 0.1;
    float collision = (rad*2 + boid.mass)*0.5;
    if (distance < collision*collision) {
      steer = PVector.sub(boid.pos, obstacle);
      steer.mult(boid.maxforce*0.1);
      return steer;
    }
    else {
      float direction = dist2(obstacle, PVector.add(boid.pos, boid.vel));
      // If is heading toward obstacle
      if (direction < distance) {
        // If steering in the verticle direction
        if (abs(boid.vel.x) <= abs(boid.vel.y)) {   
          steer = new PVector((boid.pos.x - obstacle.x), boid.vel.y);
          steer.mult(boid.maxforce*((bound*bound)/distance)*0.001);       
        }
        // If steering in the horizontal direction
        else {
          steer = new PVector(boid.vel.x, (boid.pos.y - obstacle.y));
          steer.mult(boid.maxforce*((bound*bound)/distance)*0.001);  
        }
      }
    }
  }
  return steer;
}

/*
PVector flocking(Agent boid) {
  // Get steering forces
  PVector steer = new PVector();
  PVector coh = new PVector(); // Perceived center
  PVector sep = new PVector(); // Displacement
  PVector ali = new PVector(); // Perceived velocity
  int count = 0;
  // Agents try to fly towards the centre of mass of neighbouring agents
  // Agents try to keep a small distance away from other objects (including other agents)
  // Agents try to match velocity with near agents
  for (int i = 0; i < boids.size(); i++) {
    Agent boid = (Agent) boids.get(i);
    float distance = Vector3D.dist2(pos, boid.pos);
    // Go through each agents
    if (this != boid && distance < Rn*Rn) {
      coh.add(boid.pos); // Cohesion
      ali.add(boid.vel); // Alignment
      count++;
    }      
    // Separation
    if (this != boid && distance < SDistance) {
      Vector3D diff = Vector3D.sub(boid.pos, pos); // (agent.position - bJ.position)
      diff.normalize();
      distance = (float) Math.sqrt(distance);
      diff.div(distance); // Weighed by distance
      sep.sub(diff); // c = c - (agent.position - bJ.position)
    }
  }
  if (count > 0) {
    // Cohesion - Step towards the center of mass
    coh.div((float)count); // cJ = pc / (N-1)
    coh.sub(pos); // (pcJ - bJ.position)
    coh.mult(CStep); // (pcJ - bJ.position) / 100
  // Alignment - Find average velocity
    ali.div((float)count); // pvJ = pvJ / N-1
    ali.sub(vel); // (pvJ - bJ.velocity)
    ali.mult(AVelocity); // (pvJ - bJ.velocity) / 8
  }
  // Apply weights
  coh.mult(wCoh);
  sep.mult(wSep);
  ali.mult(wAli);
  // Accumulate forces
  steer.add(coh);
  steer.add(sep);
  steer.add(ali);
  // Add speed
  steer.mult(maxspeed);
  return steer;
}
*/
void bounding(Agent boid) {
  if (boid.pos.x < -boid.mass) boid.pos.x = width + boid.mass;
  if (boid.pos.y < -boid.mass) boid.pos.y = height + boid.mass;
  if (boid.pos.x > width + boid.mass) boid.pos.x = -boid.mass;
  if (boid.pos.y > height + boid.mass) boid.pos.y = -boid.mass;
}

void updateweight(Agent boid) {
  boid.KWander = 1.0;
  boid.KAvoid = 5.0;
  boid.KFlock = 1.0;
  boid.KCohesion = 1.0;
  boid.KSeparate = 2.0;
  boid.KAlignment = 1.0;
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
      
      /***debug*************/
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
      /******************/
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
  // Render info
  fill(0);
  text("FPS: " + frameRate, 15, 20);  
  // Neighborhood radius
  fill(100, 100, 100, 30);
  noStroke();
  ellipse(mouseX, mouseY, rad, rad);
}

float heading2D(PVector vec) { return -1*((float) Math.atan2(-vec.y, vec.x)); }
float mag2(PVector vec) { return (vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); }
float dist2(PVector v1, PVector v2) { return ((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z)); }

class Agent {
  float mass = 10.0;
  PVector pos; // Position
  PVector vel; // Velocity
  PVector acc; // Acceleration
  int type; // Agent type
  float wdelta; // Wander delta
  int action; // Current action

  float maxspeed; // Maximum speed
  float maxforce; // Maximum steering force
  float maxpursue; // Maximum steering force
  float maxevade; // Maximum steering force
  float BRadius; // Avoid: radius of agent bounding sphere
  //float KNoise = 1.0;
  float KWander;
  float KAvoid;
  float KFlock;
  float KCohesion;
  float KSeparate;
  float KAlignment;

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
    BRadius = 20.0;
    KWander = 1.0;
    KAvoid = 5.0;
    KFlock = 1.0;
    KCohesion = 1.0;
    KSeparate = 2.0;
    KAlignment = 1.0;    
  }
}
