import processing.serial.*;
String val;     // Data received from the serial port
Serial myPort;
int step = 50;
float x1,x2;
float y1,y2;
float diffx =0,diffy = 0;
float rotStateX = 0;
float rotStateY = 0;
boolean mouseState = false; 
boolean prevMouseState = false;


float roll, pitch,yaw;
void setup()
{
  // [3] for macbook
  // [1] for my pc
String portName = Serial.list()[1]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 115200);
    size(400, 400, P3D);
}


void draw()
{
   if ( myPort.available() > 0) 
  {  // If data is available,
  val = myPort.readStringUntil('\n');         // read it and store it in val
  } 
    if (val != null) {
      String[] list = split(val, ',');
      if (list.length == 4)
      {
      roll = float(list[1]);
      pitch = float(list[2]);
      yaw = float(list[3]);
      }
    }
    
  background(255);
  
  lights();
  
  // grids
  /*
  for (int i = 0; i < width/step; i++ ) {
    line(i*step, 0, i*step, height);
    line(0, i*step, width, i*step);
  } 
  */


  // main translate and rotate
  pushMatrix();
  translate(200, 200,0);
  
   mouseState = mousePressed;
// perform during transition
if (mouseState==true && prevMouseState ==false)
{
  x1 = mouseX;
  y1 = mouseY;



  prevMouseState = mouseState;
 
}
// perform during transition
if (mouseState==false && prevMouseState ==true)
{


   prevMouseState = mouseState;
   rotStateX = rotStateX + diffx;
   rotStateY = rotStateY - diffy;
   

}
// perform during state
if (mouseState == true)
{
  x2 = mouseX;
  y2 = mouseY;

 diffx = (x2 - x1)*TWO_PI/width/2;
 diffy = (y2 - y1)*TWO_PI/width/2;

}
  
  // rotate Z (yaw)
  rotateY(rotStateX-radians(yaw));
  
  // rotate Y (pitch)
  rotateX(-radians(pitch));

  // rotate X (roll)
  rotateZ(radians(roll));

  // MAIN BOX
  fill(255,255,0);
  box(100);
  
  // y axis (green)
  pushMatrix();
  translate(100,0,0);
  fill(0,255,0);
  box(100,10,10);
  popMatrix();
  
  // z axis (red)
  pushMatrix();
  translate(0,100,0);
  fill(255,0,0);
  box(10,100,10);
  popMatrix();
  
  // x axis (blue)
  pushMatrix();
  translate(0,0,-100);
  fill(0,0,255);
  box(10,10,100);
  popMatrix();
  
  popMatrix();
  


  String s1 = "roll: " + (roll) +" deg";
  String s2 = "pitch: " + (pitch) +" deg";
  String s3 = "yaw: " + (yaw) +" deg";
  textSize(18);
  text(s1, 10, 30); 
  text(s2, 10, 50); 
  text(s3, 10, 70); 

}
