import fsm.*;
import processing.net.*;
import SimpleOpenNI.*;  //middleware to execute skeleton tracking
SimpleOpenNI kinect;
import java.lang.Runtime;
import processing.video.*; //to run the intro video

Movie intro;
Movie outro;
Movie currentMovie;

int[] userMap;
PImage backgroundImage; //background image

PVector prevRightHandLocation;
PVector prevLeftHandLocation;
PVector prevRightKneeLocation;
PVector prevLeftKneeLocation;
PVector prevRightHipLocation;
PVector prevLeftHipLocation;
PVector prevRightFootLocation;
PVector prevLeftFootLocation;

int r = 255;
int g = 255;
int b = 255;

float finalScore = 0;

float testTime = 10000;
float timeLeft = 0;
float startTime = 0;

//no calibration
boolean autoCalib=true;

void setup() { 

  kinect = new SimpleOpenNI(this); //create kinect object
  kinect.enableDepth(); //enable the depth image
  kinect.enableRGB(); //display color video feed of user rather than depth image
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  kinect.alternativeViewPointDepthToImage(); //turn on depth color alignment

  //load the background image
  background(255);

  size(640, 480); 

  //initialize joint location objects as vectors
  prevRightHandLocation = new PVector(0, 0, 0);
  prevLeftHandLocation = new PVector(0, 0, 0);
  prevRightKneeLocation = new PVector(0, 0, 0);
  prevLeftKneeLocation = new PVector(0, 0, 0);
  prevRightHipLocation = new PVector(0, 0, 0);
  prevLeftHipLocation = new PVector(0, 0, 0);
  prevRightFootLocation = new PVector(0, 0, 0);
  prevLeftFootLocation = new PVector(0, 0, 0);
}


void draw() { 
  //set up the kinect part
  //scale(2);
  kinect.update(); 
  PImage rgbImage = kinect.rgbImage();
  //image(depthImage, 0, 0);
  //image(rgbImage, 0, 0);
  //image(kinect.depthImage(), 0, 0);

  IntVector userList = new IntVector(); 
  kinect.getUsers(userList);

  if (userList.size() > 0) { 
    int userId = userList.get(0);

    if (kinect.isTrackingSkeleton(userId)) { 
      //display the user's image
      //prepare the color pixels
      rgbImage.loadPixels();
      loadPixels();
      userMap = kinect.getUsersPixels(SimpleOpenNI.USERS_ALL);
      for (int i = 0; i < userMap.length; i++) {
        //if the pixel is part of the user
        if (userMap[i] !=0) {
          //set the sketch pixel to the color pixel
          pixels[i] = rgbImage.pixels[i];
        }
      }//end of pixel for loop
      updatePixels();

      //millis() start after calibration has happened
      if (startTime == 0) {
        startTime = millis();
        //startTime = millis();
        println("START TIME" + startTime);
      }

      drawSkeleton(userId);

      // make a vector to store the right hand
      PVector rightHandLocation = new PVector();
      // put the position of the left hand into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HAND, rightHandLocation); 
      // make a vector to store the right hand
      PVector leftHandLocation = new PVector();
      // put the position of the left hand into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HAND, leftHandLocation); 
      // make a vector to store the right hand
      PVector rightKneeLocation = new PVector();
      // put the position of the left hand into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_KNEE, rightKneeLocation); 
      // make a vector to store the left knee
      PVector leftKneeLocation = new PVector();
      // put the position of the left knee into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, leftKneeLocation); 
      //make a vector to store the left hip
      PVector leftHipLocation = new PVector();
      // put the position of the left hip into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HIP, leftHipLocation); 
      //right hip vector
      PVector rightHipLocation = new PVector();
      // put the position of the right hip into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HIP, rightHipLocation); 
      //vector for feet locations
      PVector leftFootLocation = new PVector();
      // put the position of the left hip into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_FOOT, leftFootLocation); 
      //right hip vector
      PVector rightFootLocation = new PVector();
      // put the position of the right hip into that vector
      kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_LEFT_FOOT, rightFootLocation); 

      //Look at hip to foot angle in lumbar lordosis - acute angle or obtuse angle

      //Hip to knee angle for crouched gait
      // reduce our joint vectors to two dimensions
      PVector rightHand2D = new PVector(rightHand.x, rightHand.y); 

      PVector rightElbow2D = new PVector(rightElbow.x, rightElbow.y);
      PVector rightShoulder2D = new PVector(rightShoulder.x, 
      rightShoulder.y);
      PVector rightHip2D = new PVector(rightHip.x, rightHip.y);
      // calculate the axes against which we want to measure our angles
      PVector torsoOrientation =
        PVector.sub(rightShoulder2D, rightHip2D); 

      PVector upperArmOrientation =
        PVector.sub(rightElbow2D, rightShoulder2D);

      // calculate the angles between our joints
      float shoulderAngle = angleOf(rightElbow2D, 4
        rightShoulder2D, 
      torsoOrientation);
      float elbowAngle = angleOf(rightHand2D, 
      rightElbow2D, 
      upperArmOrientation);
      // show the angles on the screen for debugging
      fill(255, 0, 0);
      scale(3);
      text("shoulder: " + int(shoulderAngle) + "\n" +
        " elbow: " + int(elbowAngle), 20, 20);
    }
  }
}
float angleOf(PVector one, PVector two, PVector axis) {
  PVector limb = PVector.sub(two, one);
  return degrees(PVector.angleBetween(limb, axis));
}

//figure out the distance between the current location and previous location of the knees
float currentScore = (rightKneeLocation.dist(prevRightKneeLocation) + leftKneeLocation.dist(prevLeftKneeLocation));


//float time = millis();

//add the distance moved to itself 
if ((millis() - startTime) < 10000) {
  finalScore += currentScore;
}

scale(3);
fill(255, 50, 70);
text("Total Score: " + finalScore, 10, 10);
//  println("TOTAL SCORE:" + finalScore);
//set the current location as the previous location (i.e. reset)
prevRightHandLocation = rightHandLocation;
prevLeftHandLocation = leftHandLocation;
prevRightKneeLocation = rightKneeLocation;
prevLeftKneeLocation = leftKneeLocation;

timeLeft = testTime - (millis() - startTime);


println("tl: " + timeLeft + " st: " + startTime + " m: " + millis());


if ((millis() - startTime) < testTime) {
  fill(200, 30, 255);
  //scale(2);
  text("Time left:" + timeLeft, 0, 20);
  //text("elapsed:" + (millis() - startTime), 10, 30);
  fill(200, 30, 255);
  text("Current Score: " + finalScore, 90, 20);
}
else {
  fill(5, 200, 100);
  //scale(2);

  text("Great job!", 15, 50);
  text("Your final score is:" + finalScore, 15, 65);
}

/*        else if (finalScore >= 36000 && finalScore <=80000) {
 fill(5, 200, 100);
 //scale(2);
 
 text("Great job!", 15, 50);
 text("Your final score is:" + "1", 15, 65);
 }
 
 else if (finalScore >= 36000 && finalScore <=80000) {
 fill(5, 200, 100);
 //scale(2);
 
 text("Great job!", 15, 50);
 text("Your final score is:" + "1", 15, 65);
 } */

//saveFrame("kinect####.png");
} // end TrackingSkeleton
} // end userDataAvaialable
}
}





void drawSkeleton(int userId) { 
  stroke(0); 
  strokeWeight(2);
  /*kinect.drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT); 
   kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_LEFT_HIP);
   
   noStroke();*/

  fill(r, g, b); 
  drawJoint(userId, SimpleOpenNI.SKEL_HEAD); 
  drawJoint(userId, SimpleOpenNI.SKEL_NECK); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_ELBOW); 
  drawJoint(userId, SimpleOpenNI.SKEL_NECK); 
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER); 
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW); 
  drawJoint(userId, SimpleOpenNI.SKEL_TORSO); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_HIP); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_KNEE); 
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_HIP); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_FOOT); 
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_KNEE); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_HIP); 
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_FOOT); 
  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_HAND); 
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_HAND);
}

void drawJoint(int userId, int jointID) { 
  PVector joint = new PVector(); 
  float confidence = kinect.getJointPositionSkeleton(userId, jointID, joint); 
  if (confidence < 0.5) {
    return;
  } 
  PVector convertedJoint = new PVector(); 
  kinect.convertRealWorldToProjective(joint, convertedJoint); 
  ellipse(convertedJoint.x, convertedJoint.y, 20, 20);
}

// user-tracking callbacks!
void onNewUser(int userId)
{
  println("onNewUser - userId: " + userId);
  println("  start pose detection");

  if (autoCalib)
    kinect.requestCalibrationSkeleton(userId, true);
  else    
    kinect.startPoseDetection("Psi", userId);
}

void onLostUser(int userId)
{
  println("onLostUser - userId: " + userId);
}


void onEndCalibration(int userId, boolean successful) {
  if (successful) { 
    println("  User calibrated !!!");
    kinect.startTrackingSkeleton(userId);
  } 
  else { 
    println("  Failed to calibrate user !!!");
    kinect.startPoseDetection("Psi", userId);
  }
}
void onStartPose(String pose, int userId) { 
  println("Started pose for user"); 
  kinect.stopPoseDetection(userId); 
  kinect.requestCalibrationSkeleton(userId, true);
}

