/* Boids - v1.1 - 2013/12/29 */
   
// Global variables
int CWIDTH = 900;
int CHEIGHT = 200;

int bNum = 15;
int pNum = 1;
int oNum = 2;

// Variables
float Rn = 80.0; // R neighborhood
float maxspeed = 5.0; // Maximum speed
float maxforce = 3.0; // Maximum steering force
float maxpursue = 20.0; // Maximum steering force
float maxevade = 10.0; // Maximum steering force
boolean info = false; // Display information
boolean bounded = false; // Bounded within world
boolean debug = false; // Display viewing fields and more

float ARadius = 100.0; // Arrival: max distance at which the agent may begin to slow down
float ERadius = 50.0; // Evade: radius of evade range
float WRadius = 40.0; // Wander: radius of wandering circle
float NRadius = 0.3; // Wander: radius of wander noise circle
float BRadius = 20.0; // Avoid: radius of agent bounding sphere
float ORadius = 20.0; // Avoid: radius of object bounding sphere
float CStep = 1.0/100.0; // Cohesion: move it #% of the way towards the center
float SDistance = 100.0f; // Separation: small separation distance
float AVelocity = 1.0/8.0; // Alignment: add a small portion to the velocity

// Weight constants
float KNoise = 1.0;
float KWander = 1.0;
float KAvoid = 5.0;
float KFlock = 1.0;
float KCohesion = 1.0;
float KSeparate = 2.0;
float KAlignment = 1.0;

// Setup the Processing Canvas
void setup() {
  size(900, 200);  
  world = new World();
  frameRate(40);
}

// Main draw loop
void draw() {
  background(255);  
  world.run();
}

class Agent {
  float mass;
  float energy;
  PVector pos; // Position
  PVector vel; // Velocity
  PVector acc; // Acceleration
  int type; // Agent type
  float wdelta; // Wander delta
  int action; // Current action
  int prey; // Predator's target
  // Weights
  float wWan;
  float wAvo;
  float wFlo;
  float wCoh;
  float wSep;
  float wAli;

  Agent(float px, float py, int t) {
    mass = 10.0;
    energy = 10*ceil(random(5, 10));
    pos = new PVector(px, py);
    vel = new PVector(random(-5, 5), random(-5, 5));
    acc = new PVector();
    type = t;
    wdelta = 0.0;
    action = 0;
    updateweight();
  }

  void run(ArrayList boids, ArrayList predators, ArrayList objs) {
    acc.set(0, 0, 0); // Reset accelertion to 0 each cycle
    steer(boids, predators, objs); // Update steering with approprate behavior
    vel.add(acc); // Update velocity
    switch (action) {
      case 1: vel.limit(maxpursue); break; // Limit pursue speed
      case 2: vel.limit(maxevade); break; // Limit evade speed
      default: vel.limit(maxspeed); break; // Limit speed      
    }
    pos.add(vel); // Move agent
    bounding(); // Wrap around the screen or else...
    updateweight(); // Updates weights
    render();
  }

  void steer(ArrayList boids, ArrayList predators, ArrayList objs) { 
    if (type == 2) predator(boids); // Determine current action
    // Initialize steering forces
    PVector wan = new PVector();
    PVector flo = new PVector();
    PVector avo = new PVector();
    PVector pur = new PVector();
    PVector eva = new PVector();
    PVector arr = new PVector();
    PVector dep = new PVector();
    // Calculate steering forces
    switch (action) {
      // Evading
      case 1: {
        eva = evade(predators);
        avo = avoid(objs); 
        break; 
      }
      // Pursuing
      case 2: {
        pur = pursue(boids);
        avo = avoid(objs);  
        break;
      }
      // Wandering
      default: {
        if (type == 1) {
          wan = wander(); 
          avo = avoid(objs);
          flo = flocking(boids);
          eva = evade(predators);
          break;
        }
        if (type == 2) {
          wan = wander(); 
          avo = avoid(objs);   
          break;
        }
      }      
    }
    // User interaction
    if (mousePressed && keyPressed == false && type == 1) {
      // Left mouse button - Arrival
      if (mouseButton == LEFT) {
        PVector mouse = new PVector(mouseX, mouseY);
        arr = arrival(mouse);
      }
      // Right mouse button - Departure
      else if (mouseButton == RIGHT) {
        PVector mouse = new PVector(mouseX, mouseY);
        dep = departure(mouse);
        dep.mult(maxevade);
      }
    }
    // Apply weights
    wan.mult(wWan);
    avo.mult(wAvo);
    flo.mult(wFlo);
    // Accumulate steering force
    acc.add(wan);
    acc.add(avo);
    acc.add(flo);
    acc.add(pur);
    acc.add(eva);
    acc.add(arr);
    acc.add(dep);
    acc.limit(maxforce); // Limit to maximum steering force
  }
  
  void predator(ArrayList boids) {
    if (energy > 0) energy -= random(0.5);
    if (energy < 0) energy = 10*ceil(random(10, 20));
    if (energy < 10 && action == 0) {
      action = 2;
      prey = int(random(boids.size() - 1));
    }        
    if (energy > 10 && action == 2) action = 0;
  }

  PVector seek(PVector target) {
    PVector steer; // The steering vector
    PVector desired = PVector.sub(target, pos); // A vector pointing from current location to the target
    float distance = desired.magSq(); // Distance from the target is the magnitude of the vector
    // If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (distance > 0) {
      desired.normalize(); // Normalize desired

      desired.mult(maxforce);

      steer = PVector.sub(desired, vel); // Steering = Desired minus Velocity
    }
    else {
      steer = new PVector(0, 0);
    }
    return steer;
  }

  PVector flee(PVector target) {
    PVector steer; // The steering vector
    PVector desired = PVector.sub(target, pos); // A vector pointing from current location to the target
    float distance = desired.magSq(); // Distance from the target is the magnitude of the vector
    // If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (distance > 0 && distance < ARadius*100) {
      desired.normalize(); // Normalize desired

      desired.mult(maxforce);

      steer = PVector.sub(vel, desired); // Steering = Desired minus Velocity
    }
    else {
      steer = new PVector(0, 0);
    }
    return steer;
  }

  PVector arrival(PVector target) {
    PVector steer; // The steering vector
    PVector desired = PVector.sub(target, pos); // A vector pointing from current location to the target
    float distance = desired.magSq(); // Distance from the target is the magnitude of the vector
    // If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (distance > 0) {
      desired.normalize(); // Normalize desired

      if (distance < ARadius) desired.mult(maxspeed*(distance/ARadius)); // This damping is somewhat arbitrary
      else desired.mult(maxforce);

      steer = PVector.sub(desired, vel); // Steering = Desired minus Velocity
    }
    else {
      steer = new PVector();
    }
    return steer;
  }

  PVector departure(PVector target) {
    PVector steer; // The steering vector
    PVector desired = PVector.sub(target, pos); // A vector pointing from current location to the target
    float distance = desired.magSq(); // Distance from the target is the magnitude of the vector
    // If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (distance > 0 && distance < ARadius*100) {
      desired.normalize(); // Normalize desired

      if (distance < ARadius) desired.mult(maxspeed*(ARadius/distance)); // This damping is somewhat arbitrary
      else desired.mult(maxforce);

      steer = PVector.sub(vel, desired); // Steering = Desired minus Velocity
    }
    else {
      steer = new PVector();
    }
    return steer;
  }
  
  PVector pursue(ArrayList boids) {
    PVector steer = new PVector();
    if (prey < boids.size()) {
      Agent boid = (Agent) boids.get(prey);
      steer = PVector.sub(boid.pos, pos);
      steer.mult(maxpursue);
    }
    return steer;
  }
  
  PVector evade(ArrayList predators) {
    PVector steer = new PVector();
    for (int i = 0; i < predators.size(); i++) {
      Agent predator = (Agent) predators.get(i);
      float distance = PVector.dist(pos, predator.pos);
      if (distance < ERadius) {
        action = 1;
        steer = flee(predator.pos);
        steer.mult(maxevade);
        return steer;
      }
    }
    action = 0;
    return steer;
  }

  PVector wander() {
    wdelta += random(-NRadius, NRadius); // Determine noise ratio
    // Calculate the new location to steer towards on the wander circle
    PVector center = vel.get(); // Get center of wander circle
    center.mult(60.0); // Multiply by distance
    center.add(pos); // Make it relative to boid's location
    // Apply offset to get new target    
    PVector offset = new PVector(WRadius*cos(wdelta), WRadius*sin(wdelta));
    PVector target = PVector.add(center, offset); // Determine new target
    // Steer toward new target    
    PVector steer = seek(target); // Steer towards it    
    return steer;
  }
  
  PVector avoid(ArrayList objs) {
    PVector steer  = new PVector();    

    for (int i = 0; i < objs.size(); i++) {
      Obj obj = (Obj) objs.get(i);
      // Distance between object and avoidance sphere
      float distance = PVector.dist(obj.pos, pos);
      // If distance is less than the sum of the two radius, there is collision
      float bound = obj.mass*0.5 + BRadius + ORadius;
      if (distance < bound) {
        wAvo = 10.0;
        wWan = 0.1;
        float collision = (obj.mass + mass)*0.5;
        if (distance < collision) {
          steer = PVector.sub(pos, obj.pos);
          steer.mult(maxforce*0.1);
          return steer;
        }
        else {
          float direction = PVector.dist(obj.pos, PVector.add(pos, vel));
          // If is heading toward obstacle
          if (direction < distance) {
            // If steering in the verticle direction
            if (abs(vel.x) <= abs(vel.y)) {   
              steer = new PVector((pos.x - obj.pos.x), vel.y);
              steer.mult(maxforce*((bound)/distance)*0.001);       
            }
            // If steering in the horizontal direction
            else {
              steer = new PVector(vel.x, (pos.y - obj.pos.y));
              steer.mult(maxforce*((bound)/distance)*0.001);  
            }
          }
        }
      }
    }
    return steer;
  }

  PVector flocking(ArrayList boids) {
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
      float distance = PVector.dist(pos, boid.pos);
      // Go through each agents
      if (this != boid && distance < Rn) {
        coh.add(boid.pos); // Cohesion
        ali.add(boid.vel); // Alignment
        count++;
      }      
      // Separation
      if (this != boid && distance < SDistance) {
        PVector diff = PVector.sub(boid.pos, pos); // (agent.position - bJ.position)
        diff.normalize();
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
  
  // Wrap around or bounded 
  void bounding() {
    if (bounded) {
      if (pos.x <= BRadius) vel.x = BRadius - pos.x;
      else if (pos.x >= width - BRadius) vel.x = (width - BRadius) - pos.x;
      if (pos.y <= BRadius) vel.y = BRadius - pos.y;
      else if (pos.y >= height - BRadius) vel.y = (height - BRadius) - pos.y;
    }
    else {
      if (pos.x < -mass) pos.x = width + mass;
      if (pos.y < -mass) pos.y = height + mass;
      if (pos.x > width + mass) pos.x = -mass;
      if (pos.y > height + mass) pos.y = -mass;
    }
  }
  
  void updateweight() {
    wWan = KWander;
    wAvo = KAvoid;
    wFlo = KFlock;
    wCoh = KCohesion;
    wSep = KSeparate;
    wAli = KAlignment;
  }
  
  void render() {   
    if (type == 1) {
      fill(156, 206, 255);
      stroke(16, 16, 222);
      ellipse(pos.x, pos.y, mass, mass);
      PVector dir = vel.get();
      dir.normalize();
      line(pos.x, pos.y, pos.x + dir.x*10, pos.y + dir.y*10);
    }
    else if (type == 2) {
      // Draw a triangle rotated in the direction of velocity    
      float theta = (-1*((float) Math.atan2(-vel.y, vel.x))) + radians(90);
      pushMatrix();
      translate(pos.x, pos.y);
      rotate(theta);
      fill(220, 0, 0);
      noStroke();
      beginShape(TRIANGLES);
      vertex(0, -mass);
      vertex(-3, mass);
      vertex(3, mass);
      endShape();
      popMatrix();
    }
    // Debug mode
    if (debug) {
      // Velocity
      stroke(16, 148, 16);
      line(pos.x, pos.y, pos.x + vel.x*4, pos.y + vel.y*4);
      // Steering
      stroke(255, 0, 0);
      line(pos.x, pos.y, pos.x + acc.x*20, pos.y + acc.y*20);
      // Neighborhood radius
      fill(239, 239, 239, 10);
      stroke(132, 132, 132);
      ellipse(pos.x, pos.y, Rn*2, Rn*2);
      fill(100, 100, 100, 30);
      noStroke();
      ellipse(pos.x, pos.y, BRadius*2, BRadius*2);
      stroke(255, 0, 0);
      noFill();
    }
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

class World {
  ArrayList boids;
  ArrayList predators;
  ArrayList objs;

  World() {
    initAgents();
    initobjs();
  }

  void initAgents() {
    // Add boids
    boids = new ArrayList();
    for (int i = 0; i < bNum; i++) {
      Agent boid = new Agent(random(width), random(height), 1);
      boids.add(boid);
    }
    // Add predator
    predators = new ArrayList();
    for (int i = 0; i < pNum; i++) {
      Agent predator = new Agent(random(width), random(height), 2);
      predators.add(predator);
    }
  }

  void initobjs() {
    objs = new ArrayList();
    // Add objects
    for (int i = 0; i < oNum; i++) {
      Obj obj = new Obj(random(1, width), random(1, height), random(50, 100), round(random(1, 2)));
      objs.add(obj);
    }
  }

  void run() {
    update();
    render();
  }

  void update() {
    // Update agents
    for (int i = 0; i < boids.size(); i++) {
      Agent boid = (Agent) boids.get(i);
      boid.run(boids, predators, objs);
    }
    for (int i = 0; i < predators.size(); i++) {
      Agent predator = (Agent) predators.get(i);
      predator.run(boids, predators, objs);
    }
  }

  void render() {
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
      // Debug mode
      if (debug) {
        // Neighborhood radius
        fill(100, 100, 100, 30);
        noStroke();
        ellipse(obj.pos.x, obj.pos.y, obj.mass + ORadius*2, obj.mass + ORadius*2);
      }
    }
    // Render info
    if (info) {
      fill(0);
      text("FPS: " + frameRate, 15, 20);
      text("Boids: " + (world.boids.size()), 15, 35);
      text("Predators: " + (world.predators.size()), 15, 50);
      text("Objects: " + (world.objs.size()), 15, 65);
    }
  }
}
