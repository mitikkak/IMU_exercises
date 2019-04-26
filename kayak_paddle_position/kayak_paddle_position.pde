
import processing.net.*;
import websockets.*;

class AngularPosition
{
  AngularPosition(float p, float r, float y)
  {
    yaw = y;
    pitch = p;
    roll = r;
  }
  public float yaw;
  public float pitch;
  public float roll;
};

/*
AngularPosition inputPos[];
int positionIndex;
*/
WebsocketClient c;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

void setup()
{
  /*
  inputPos = new AngularPosition[15];
  inputPos[0] = new AngularPosition(0.0, 0.0, 0.0);
  inputPos[1] = new AngularPosition(90.0, 0.0, 0.0);
  inputPos[2] = new AngularPosition(180.0, 0.0, 0.0);
  inputPos[3] = new AngularPosition(270.0, 0.0, 0.0);
  inputPos[4] = new AngularPosition(350.0, 0.0, 0.0);
  
  inputPos[5] = new AngularPosition(0.0, 0.0, 0.0);
  inputPos[6] = new AngularPosition(0.0, 90.0, 0.0);
  inputPos[7] = new AngularPosition(0.0, 180.0, 0.0);
  inputPos[8] = new AngularPosition(0.0, 270.0, 0.0);
  inputPos[9] = new AngularPosition(0.0, 350.0, 0.0);
  
  inputPos[10] = new AngularPosition(0.0, 0.0, 0.0);
  inputPos[11] = new AngularPosition(0.0, 0.0, 90.0);
  inputPos[12] = new AngularPosition(0.0, 0.0, 180.0);
  inputPos[13] = new AngularPosition(0.0, 0.0, 270.0);
  inputPos[14] = new AngularPosition(0.0, 0.0, 350.0);

  positionIndex = 0;
  */
  c = new WebsocketClient(this, "ws://192.168.8.101:80");
  size(600, 500, P3D);

  // if you have only ONE serial port active
  //myPort = new Serial(this, Serial.list()[0], 9600); // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5:", 9600);                    // Windows
  //myPort = new Serial(this, "/dev/ttyACM0", 9600);             // Linux
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

String nextPositionMessage = new String("Orientation: 0.0 0.0 0.0");
void webSocketEvent(String msg){
 println(msg);
 nextPositionMessage = msg;
}

void draw()
{
  c.sendMessage("position?\n");
  
  serialEvent(nextPositionMessage);  // read and parse incoming serial message
  //positionIndex++;
  //if (positionIndex == inputPos.length) { positionIndex = 0; }
  background(255); // set background to white
  lights();

  translate(width/2, height/2); // set position to centre

  pushMatrix(); // begin object

  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch));
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawArduino();

  popMatrix(); // end of object

  // Print values to console
  //print(positionIndex); print(": "); print("\t");
  print("pitch"); print(pitch); print("\t");
  print("roll"); print(roll); print("\t");
  print("yaw"); print(yaw); println();
  //delay(1000);
}

void serialEvent(String message)
{
  
  String[] list = split(trim(message), " ");
      if (list.length >= 4 && list[0].equals("Orientation:")) {
        pitch = float(list[1]); // convert to float pitch
        roll = float(list[2]); // convert to float roll
        yaw = float(list[3]); // convert to float yaw
      }
}

void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 10); // draw Arduino board base shape
  translate(150, 0, 0); // set position to edge of Arduino box
  box(50, 10, 30); // draw Arduino board base shape
  translate(-300, 0, 0); // set position to edge of Arduino box
  rotateX(radians(60));
  box(50, 10, 30); // draw Arduino board base shape
  //stroke(0); // set outline colour to black
  //fill(80); // set fill colour to dark grey

  //translate(60, -10, 90); // set position to edge of Arduino box
  //box(170, 20, 10); // draw pin header as box

  //translate(-20, 0, -180); // set position to other edge of Arduino box
  //box(210, 20, 10); // draw other pin header as box
}
