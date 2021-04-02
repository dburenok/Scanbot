import processing.serial.*;
import peasy.*;
import java.time.format.DateTimeFormatter;
import java.time.LocalDateTime;
import java.io.File;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import javax.swing.JFileChooser;
import java.util.Scanner;

PeasyCam camera;

Serial myPort;

// string variable to collect serial data
String receivedString = null;

// ASCII code for carriage return in serial
int end = 10;

// total number of points expected to receive over serial port
final int NUM_POINTS = 1600 * 801;

// string array to store individual x, y, z components of points
String[] receivedCartesianPoints = new String[3];

// main array to store received xyz coordinates of points
float pointsArray[][] = new float[NUM_POINTS][3];

// number of points received over serial port
int receivedCount = 0;

// main render loop switch, flips true when serial data is first received
boolean started = true;

// file saving
//PrintWriter output;

int renderSkip = 1;
int scaleX = -1;
File scanOpenFile;
int sceneScaler = 3;

void setup() {
  myPort = new Serial(this, Serial.list()[1], 500000);  // Serial.list()[0]

  //size(1920, 1080, P3D);
  fullScreen(P3D, 1);
  pixelDensity(displayDensity());

  scale(scaleX, 1, 1);

  camera = new PeasyCam(this, 0, 0, 0, 200);

  // zero out the main points array
  for (int i = 0; i < NUM_POINTS; i++) {
    pointsArray[i][0] = 0;
    pointsArray[i][1] = 0;
    pointsArray[i][2] = 0;
  }

  printArray(Serial.list());
}

void draw() {
  // 3D part
  hint(ENABLE_DEPTH_TEST); 
  perspective(PI/3, float(width)/height, 1, 1000000);
  background(8);
  noFill();

  // trial-and-error scaling with F key
  scale(scaleX, 1, 1);

  if (started) {

    stroke(255, 255, 255); // white color vertices, can be any
    for (int i = 0; i < receivedCount; i += renderSkip) { // only rendering the number of points received
      beginShape(POINTS);
      vertex(pointsArray[i][0] * sceneScaler, pointsArray[i][1] * sceneScaler, pointsArray[i][2] * sceneScaler);
      endShape();
    }
  } 

  // 2D part / HUD
  pushMatrix();
  camera();
  hint(DISABLE_DEPTH_TEST);
  noLights();
  textSize(16);
  text("Scan opened: " + scanOpenFile, 10, 30);
  text("Resolution: " + 1.0 / renderSkip + 
    ", FPS: " + round(frameRate) + 
    ", Points: " + receivedCount / renderSkip + 
    ", Serial port: \"" + Serial.list()[1] + "\"" + 
    ", Serial available: " + myPort.available(), 10, 60);
  popMatrix();
}

void serialEvent (Serial myPort) {
  try {
    receivedString = myPort.readStringUntil('\n'); // read the serial string until newline charcater, looks like this "1.2323,3.5120,0.0122\n"
    if (receivedString != null) {  // if the string is not empty
      receivedCartesianPoints = split(receivedString, ',');  // array stores values into separate cells separated by commas
      pointsArray[receivedCount][0] = float(receivedCartesianPoints[0]); // convert the strings into floats as they should be
      pointsArray[receivedCount][1] = float(receivedCartesianPoints[1]);
      pointsArray[receivedCount][2] = float(receivedCartesianPoints[2]);

      receivedCount++; // increment number of received points

      println(receivedCount); // print total number of points just in case
    }
  }
  catch(RuntimeException e) {
    e.printStackTrace();
  }
}

void keyPressed() { // Press a key to save the data
  if (key == 's' || key == 'S') {
    // Create a new file in the sketch directory
    DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
    LocalDateTime now = LocalDateTime.now();

    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < receivedCount; i++) { //for each received point
      for (int j = 0; j < 3; j++) { //for each column
        builder.append(pointsArray[i][j] + ""); // append to the output string
        if (j < 2) // if this is not the last row element
          builder.append(","); // then add comma, csv
      }
      builder.append("\n"); //append new line at the end of the row
    }
    try {
      String filename = dtf.format(now).replaceAll(":", "_").replaceAll("/", "_") + ".csv";
      BufferedWriter writer = new BufferedWriter(new FileWriter(filename));
      writer.write(builder.toString()); //save the string representation of the 2D array
      writer.close();
      println("Scan saved as " + filename + ".");
    }
    catch (IOException e) {
      System.out.println(e);
    }
  } else if (key == 'o' || key == 'O') {

    selectInput("Select a scan to display:", "scanSelected");

    // zero out the main points array
    for (int i = 0; i < NUM_POINTS; i++) {
      pointsArray[i][0] = 0;
      pointsArray[i][1] = 0;
      pointsArray[i][2] = 0;
    }
    receivedCount = 0;
  } else if (key == 'r' || key == 'R') {
    if (renderSkip == 1) {
      renderSkip = 2;
    } else if (renderSkip == 2) {
      renderSkip = 4;
    } else if (renderSkip == 4) {
      renderSkip = 8;
    } else if (renderSkip == 8) {
      renderSkip = 1;
    }
  } else if (key == 'f' || key == 'F') {
    scaleX *= -1;
  } else if (key == 'z' || key == 'Z') {
    // zero out the main points array
    for (int i = 0; i < NUM_POINTS; i++) {
      pointsArray[i][0] = 0;
      pointsArray[i][1] = 0;
      pointsArray[i][2] = 0;
    }
    renderSkip = 1;
    receivedCount = 0;
    scanOpenFile = null;
  }
}

void scanSelected(File file) {
  if (file == null) {
    println("Scan open dialog cancelled.");
  } else {
    scanOpenFile = file;
    Scanner in = null;
    try {
      in = new Scanner(file);
      while (in.hasNextLine()) {
        receivedCartesianPoints = split(in.nextLine(), ',');
        pointsArray[receivedCount][0] = float(receivedCartesianPoints[0]); // convert the strings into floats as they should be
        pointsArray[receivedCount][1] = float(receivedCartesianPoints[1]);
        pointsArray[receivedCount][2] = float(receivedCartesianPoints[2]);
        receivedCount++;
      }
    }
    catch (IOException e) {
      System.out.println(e);
    }
  }
}
