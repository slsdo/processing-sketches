/* Boids - v1.1 - 2013/12/29 */
   
// Global variables
ArrayList agents;
ArrayList objs;

// Setup the Processing Canvas
void setup() {
  size(900, 200);
  frameRate(40);
  
  initAgents(10, 1);
  initObj(3);
}

// Main draw loop
void draw() {
  background(255);
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

class Agent {
  float mass = 10.0;
  PVector pos; // Position
  PVector vel; // Velocity
  PVector acc; // Acceleration
  int type; // Agent type
  float wdelta; // Wander delta
  int action; // Current action

  Agent(float px, float py, int t) {
    mass = 10.0;
    pos = new PVector(px, py);
    vel = new PVector(random(-5, 5), random(-5, 5));
    acc = new PVector();
    type = t;
    wdelta = 0.0;
    action = 0;
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

