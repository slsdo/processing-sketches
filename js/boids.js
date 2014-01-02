/* Boids - v1.1.0 - 2014/01/02 */
   
// Global variables
ArrayList agents;
float rad = 60.0; // Mouse radius

// Setup the Processing Canvas
void setup() {
  size(900, 200);
  frameRate(40);
  
  initAgents(30, 1);
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
    switch (boid.action) {
      case 1: boid.vel.limit(boid.maxpursue); break; // Limit pursue speed
      case 2: boid.vel.limit(boid.maxevade); break; // Limit evade speed
      default: boid.vel.limit(boid.maxspeed); break; // Limit speed      
    }
    boid.pos.add(boid.vel); // Move agent
    bounding(boid); // Wrap around the screen or else...
    updateweight(boid); // Updates weights
  }
}

void steer(Agent boid) {
  PVector mouse = new PVector(mouseX, mouseY);
  PVector wan = new PVector();
  PVector flo = new PVector();
  PVector avo = new PVector();
  PVector pur = new PVector();
  PVector eva = new PVector();
  PVector arr = new PVector();
  
  // Predator behavior
  if (boid.type == 2) {
    if (boid.hunt > 0) boid.hunt -= random(0.5);
    if (boid.hunt < 0) boid.hunt = 10*ceil(random(6, 15));
    if (boid.hunt < 20 && boid.action == 0) {
      boid.action = 2;
       do {
        int pick = int(random(agents.size() - 1));
        boid.prey = (Agent) agents.get(pick);
      }
      while (boid.prey.type != 1);  
    }        
    if (boid.hunt > 20 && boid.action == 2) boid.action = 0;
  }
  
  // Calculate steering forces
  switch (boid.action) {
    // Evading
    case 1: {
      eva = evade(boid);
      avo = avoid(boid, mouse);  
      break; 
    }
    // Pursuing
    case 2: {
      pur = pursue(boid, boid.prey);
      avo = avoid(boid, mouse);  
      break;
    }
    // Wandering
    default: {
      if (boid.type == 1) {
        wan = wander(boid);
        avo = avoid(boid, mouse);
        flo = flocking(boid);
        eva = evade(boid);
        break;
      }
      if (boid.type == 2) {
        wan = wander(boid);
        if (mouse.x > 5 && mouse.x < width - 5 && mouse.y > 5 && mouse.y < height - 5) {
          arr = arrival(boid, mouse);
        }
        break;
      }
    }      
  }

  wan.mult(boid.KWander); // Weight
  avo.mult(boid.KAvoid); // Weight
  flo.mult(boid.KFlock); // Weight
  
  boid.acc.add(wan);
  boid.acc.add(avo);
  boid.acc.add(flo);
  boid.acc.add(pur);
  boid.acc.add(eva);
  boid.acc.add(arr);
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
  for (int i = 0; i < agents.size(); i++) {
    Agent neighbor = (Agent) agents.get(i);
    float distance = dist2(boid.pos, neighbor.pos);
    // Go through each agents
    if (boid != neighbor && distance < boid.Rn*boid.Rn) {
      coh.add(neighbor.pos); // Cohesion
      ali.add(neighbor.vel); // Alignment
      count++;
    }
    // Separation
    if (boid != neighbor && distance < boid.SDistance) {
      PVector diff = PVector.sub(neighbor.pos, boid.pos); // (agent.position - bJ.position)
      diff.normalize();
      distance = (float) Math.sqrt(distance);
      diff.div(distance); // Weighed by distance
      sep.sub(diff); // c = c - (agent.position - bJ.position)
    }
  }
  if (count > 0) {
    // Cohesion - Step towards the center of mass
    coh.div((float)count); // cJ = pc / (N-1)
    coh.sub(boid.pos); // (pcJ - bJ.position)
    coh.mult(boid.CStep); // (pcJ - bJ.position) / 100
  // Alignment - Find average velocity
    ali.div((float)count); // pvJ = pvJ / N-1
    ali.sub(boid.vel); // (pvJ - bJ.velocity)
    ali.mult(boid.AVelocity); // (pvJ - bJ.velocity) / 8
  }
  // Apply weights
  coh.mult(boid.KCohesion);
  sep.mult(boid.KSeparate);
  ali.mult(boid.KAlignment);
  // Accumulate forces
  steer.add(coh);
  steer.add(sep);
  steer.add(ali);
  // Add speed
  steer.mult(boid.maxspeed);
  return steer;
}

PVector pursue(Agent boid, Agent target) {
  PVector steer = new PVector();
  steer = PVector.sub(target.pos, boid.pos);
  steer.mult(boid.maxpursue);    
  return steer;
}

PVector evade(Agent boid) {
  PVector steer = new PVector();
  for (int i = 0; i < agents.size(); i++) {
    Agent predator = (Agent) agents.get(i);
    if (predator.type == 2) {
      float distance = dist2(boid.pos, predator.pos);
      if (distance < boid.ERadius*boid.ERadius) {
        boid.action = 1;
        steer = flee(boid, predator.pos);
        steer.mult(boid.maxevade);
        return steer;
      }
    }
  }
  boid.action = 0;
  return steer;
}

PVector flee(Agent boid, PVector target) {
  PVector steer; // The steering vector
  PVector desired = PVector.sub(target, boid.pos); // A vector pointing from current location to the target
  float distance = mag2(desired); // Distance from the target is the magnitude of the vector
  // If the distance is greater than 0, calc steering (otherwise return zero vector)
  if (distance > 0 && distance < boid.ARadius*100) {
    desired.normalize(); // Normalize desired
    desired.mult(boid.maxforce);
    steer = PVector.sub(boid.vel, desired); // Steering = Desired minus Velocity
  }
  else {
    steer = new PVector(0, 0);
  }
  return steer;
}

PVector arrival(Agent boid, PVector target) {
  PVector steer; // The steering vector
  PVector desired = PVector.sub(target, boid.pos); // A vector pointing from current location to the target
  float distance = mag2(desired); // Distance from the target is the magnitude of the vector
  // If the distance is greater than 0, calc steering (otherwise return zero vector)
  if (distance > 0) {
    desired.normalize(); // Normalize desired

    if (distance < boid.ARadius) desired.mult(boid.maxspeed*(distance/boid.ARadius)); // This damping is somewhat arbitrary
    else desired.mult(boid.maxforce);

    steer = PVector.sub(desired, boid.vel); // Steering = Desired minus Velocity
  }
  else {
    steer = new PVector();
  }
  return steer;
}

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
}

float heading2D(PVector v) { return -1*((float) Math.atan2(-v.y, v.x)); }
float mag2(PVector v) { return (v.x*v.x + v.y*v.y + v.z*v.z); }
float dist2(PVector v1, PVector v2) { return ((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z)); }

class Agent {
  float mass = 10.0;
  PVector pos; // Position
  PVector vel; // Velocity
  PVector acc; // Acceleration
  int type; // Agent type
  float wdelta; // Wander delta
  int action; // Current action
  float hunt;
  Agent prey; // Predator's target

  float maxspeed; // Maximum speed
  float maxforce; // Maximum steering force
  float maxpursue; // Maximum steering force
  float maxevade; // Maximum steering force
  
  float Rn; // R neighborhood
  float ARadius; // Arrival: max distance at which the agent may begin to slow down
  float ERadius; // Evade: radius of evade range
  float BRadius; // Avoid: radius of agent bounding sphere
  float CStep; // Cohesion: move it #% of the way towards the center
  float SDistance; // Separation: small separation distance
  float AVelocity; // Alignment: add a small portion to the velocity
  
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
    hunt = 10*ceil(random(5, 10));
    
    maxspeed = 5.0;
    maxforce = 3.0;
    maxpursue = 20.0;
    maxevade = 10.0;
    
    Rn = 80.0;
    ARadius = 100.0;
    ERadius = 50.0;
    BRadius = 20.0;
    CStep = 1.0/100.0;
    SDistance = 6000.0;
    AVelocity = 1.0/8.0;
    
    KWander = 1.0;
    KAvoid = 5.0;
    KFlock = 1.0;
    KCohesion = 1.0;
    KSeparate = 2.0;
    KAlignment = 1.0;    
  }
}
