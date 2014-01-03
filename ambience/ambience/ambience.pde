

int count = 20; // Set number of circles
Circle[] circles = new Circle[count];
color[] circleColors = new color[count];
float ds=2; // Set size of dot in circle center
boolean dragging = false;
//Colors taken from the IDE
color[] colors = { 
  #21759b, // Blue
  #669933, // Green
  #777777  // Gray
};

// Set up canvas
void setup() {
  frameRate(60); // Frame rate
  size(900, 900); // Size of canvas (width,height)
  strokeWeight(1); // Stroke/line/border thickness
  background(255, 255, 255);

  for (int i = 0; i < count; i++) {
    circles[i] = new Circle();
  }
}

void draw() {
  background(255, 255, 255);

  // Begin looping through circle array
  for (int i = 0; i < count; i++) {
    update(circles[i]);
    render(circles[i]);
  }
}

void update(Circle c) {
  c.acc.set(0, 0, 0);
  float damp = 0.99; // Damping force
  
  if (c.locked && c.parent == null) {
    // Move the particle's coordinates to the mouse's position, minus its original offset
    PVector m = new PVector(mouseX, mouseY);
    c.acc = PVector.sub(PVector.sub(m, c.offset), c.pos);
    c.acc.limit(1); // Limit to maximum force
    c.vel.add(c.acc); // Apply acceleration
    c.vel.limit(5); // Limit to maximum speed
    c.pos = PVector.sub(m, c.offset);
  }
  else if (c.locked && c.parent != null) {
    // Move the particle's coordinates to the parent's position, minus its original offset
    c.acc.set(c.parent.vel);
    c.acc.limit(1); // Limit to maximum force
    c.vel.add(c.acc); // Apply acceleration
    c.vel.limit(5); // Limit to maximum speed
    c.pos.add(c.vel); // Move circle
  }
  else {
    c.acc.limit(1); // Limit to maximum force
    c.vel.add(c.acc); // Apply acceleration
    if (mag2(c.vel) > 0.5*0.5) {
      c.vel.x *= damp;
      c.vel.y *= damp;
    }
    c.pos.add(c.vel); // Move circle
  }
  
  c.opa = calcOpacity(c.pos.x, c.pos.y); // Update opacity

  float dm = c.rad*2; // Cache diameter
  // Wrap around canvas edges
  if (c.pos.x < -dm) c.pos.x = width + dm;
  if (c.pos.x > width + dm) c.pos.x = -dm;
  if (c.pos.y < -dm) c.pos.y = height + dm;
  if (c.pos.y > height + dm) c.pos.y = -dm;
}

void render(Circle c) {
  PVector m = new PVector(mouseX, mouseY);
  if (dist2(c.pos, m) < c.rad2) fill(#c93b0e, c.opa*0.5);  // orange if mouseover
  else fill(c.c, c.opa); // regular
  noStroke(); // Disable shape stroke/border

  ellipse(c.pos.x, c.pos.y, c.rad*2, c.rad*2); // Draw circle

  // If current circle is selected...
  if ((c.locked && c.parent == null && dragging)) {
    fill(#CD6633, c.opa*2); // Center dot color orange
    stroke(#CD6633, c.opa*4); // Line color orange
  } 
  else {
    fill(#000000, c.opa*2); // Center dot color black
    stroke(#777777, c.opa*2); // Line color turquoise
  }

  // Loop through all circles
  for (int j = 0; j < count; j++) {
    // If the circles are close
    if (dist2(c.pos, circles[j].pos) < c.rad2) {
      // Stroke a line from current circle to adjacent circle
      line(c.pos.x, c.pos.y, circles[j].pos.x, circles[j].pos.y);
      
      // Attach it to parent
      if (c.locked == true && circles[j].locked == false) {
        circles[j].locked = true;
        // Assign parent and children
        circles[j].parent = c;
        c.children.add(circles[j]);
      }
    }
    else {/*
      if (circles[j].locked == true && circles[j].parent != null) {
        circles[j].offset.set(0, 0, 0);
        circles[j].locked = false;
        circles[j].parent = null; // Clear parent
        circles[j].children.clear(); // Clear all children
      }*/
    }
  }

  noStroke();  // Turn off stroke/border
  rect(c.pos.x - ds, c.pos.y - ds, ds * 2, ds * 2); // Draw dot in center of circle
}

float calcOpacity(float posx, float posy) {
  float w = width*0.5;
  float h = height*0.5;
  float opax = abs(w - posx);
  float opay = abs(h - posy);
  float opacity = opax*opax*h*h + opay*opay*w*w; // Eqn of ellipse: x^2/a^2 + y^2/b^2 = 1
  //opacity = map(opacity, w*w*h*h + h*h*w*w, 0, -70, 70); // Map to canvas
  opacity = constrain(opacity, 0, 100);
  return opacity;
}

float mag2(PVector v) { return (v.x*v.x + v.y*v.y + v.z*v.z); }
float dist2(PVector v1, PVector v2) { return ((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z)); }

void mousePressed () {
  // Look for a circle the mouse is in, then lock that circle to the mouse
  PVector m = new PVector(mouseX, mouseY);
  for (int i = 0; i < count; i++) {
    // If the circles are close...
    if (dist2(circles[i].pos, m) < circles[i].rad2) {
      circles[i].locked = true;
      circles[i].offset = PVector.sub(m, circles[i].pos);
      dragging = true; // Break out of the loop because we found our circle
      break;
    }
  }
}

void mouseReleased() {
  // User is no-longer dragging
  for (int i = 0; i < count; i++) {
    circles[i].offset.set(0, 0, 0);
    circles[i].locked = false;
    circles[i].parent = null; // Clear parent
    circles[i].children.clear(); // Clear all children
  }
  dragging = false;
}

class Circle
{
  float w;
  float h;
  float rad;
  float rad2;
  PVector pos;
  PVector vel;
  PVector acc;
  PVector offset;
  color c;
  float opa;
  boolean locked;
  Circle parent;
  ArrayList children;

  Circle() {
    w = random(width); // X
    h = random(height); // Y
    rad = random(10, 70); // Radius
    rad2 = rad*rad;
    pos = new PVector(random(width), random(height)); // Location
    vel = new PVector(random(-0.5, 0.5), random(-0.5, 0.5)); // Speed
    acc = new PVector(0, 0); // Acceleration
    offset = new PVector(0, 0); // Offset from mouse
    c = colors[int(random(colors.length))]; // Color
    opa = calcOpacity(pos.x, pos.y); // Calculate opacity
    locked = false;
    parent = null;
    children = new ArrayList();
  }
}

