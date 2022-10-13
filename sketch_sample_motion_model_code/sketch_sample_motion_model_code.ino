// Motion Model - Dimensions
#define limbseg1length 310.0 // Upperarm
#define limbseg2length 255.0 // Forearm
#define wristthickness 35.0 // 
#define elbowthickness 75.0 //
#define s1ratio 0.5
#define s2ratio 0.8
const bool LEFT_HANDED = false;
// Controlling Motion Model:
const float wrist_marker_height = 13.0;
const float sensorheighabovelimbthickness = 20.0; // = 10 if sensor taped on, = 20 if velcro is used
const bool t_as_wrist = true;
const bool t_as_marker = false;
const bool recompute_IP_at_rest = true;
const float seconds_to_record = 2.0;
#define left_trigger_value 1.05 // keep as high as possible; may lower to 1.025 if participant struggling
int motion_num = 0; // to start at 1, set as 0. 

// Core Vector Variables
struct Vectors {
  float r = 0.0; // Uncomment if we need to use quaternions, in which case this is q0, hence, it should initialize as zero, in case it doesn't get set elsewhere and we're dealing with a vector in R^3
  float x = 0.0; // q1; if ever encoding quaternions, think of x/y/z as q1/q2/q3.  
  float y = 0.0; // q2; it doesn't matter what we initialize these values as, they all get overwritten. 0/1/0/0 is a holdover from previous programs that used quaternions.
  float z = 0.0; // q3
} Sensor1_Acc, Sensor1_Gyro, // Sensor1_Mag, 
  Sensor2_Acc, Sensor2_Gyro, // Sensor2_Mag,
  Sensor3_Acc, Sensor3_Gyro, // Sensor3_Mag,
  Sensor1_GyroSaved, Sensor2_GyroSaved, Sensor3_GyroSaved, // these are used when integrating the gyro into the ROT vector
  Sensor1_Rot, Sensor2_Rot, Sensor3_Rot,
  Sensor1_GravityIP, Sensor2_GravityIP, Sensor3_GravityIP,
  Sensor1_GravityLEFT, Sensor2_GravityLEFT, Sensor3_GravityLEFT,
  Sensor1_GravityBias, Sensor2_GravityBias, Sensor3_GravityBias,
  Sensor1_GyroBias, Sensor2_GyroBias, Sensor3_GyroBias,
  p1,p2,t,a1x,a1y,a1z,s1,a2x,a2y,a2z,s2,
  p1_IP,p2_IP,t_IP,a1x_IP,a1y_IP,a1z_IP,s1_IP,a2x_IP,a2y_IP,a2z_IP,s2_IP,
  zerov;

void setup() {
  // Set gravity biases: 
  Sensor1_GravityBias.x = -0.05; 
  Sensor1_GravityBias.y = 0.075; 
  Sensor1_GravityBias.z = 0.06; 
  Sensor2_GravityBias.x = 0.04; 
  Sensor2_GravityBias.y = 0.0; 
  Sensor2_GravityBias.z = 0.2; 
  Sensor3_GravityBias.x = 0.09; 
  Sensor3_GravityBias.y = 0.11; 
  Sensor3_GravityBias.z = -0.08; 
  // Make sure we set values for the saved gyro vectors
  Sensor1_GyroSaved = zerov;
  Sensor2_GyroSaved = zerov;
  Sensor3_GyroSaved = zerov;
  // Gyro bias vectors should start as zero:
  Sensor1_GyroBias = zerov;
  Sensor2_GyroBias = zerov;
  Sensor3_GyroBias = zerov;

  // Need to find the left direction from user's motion;
  // Want to subtract off directional changes due to rotation about gravity vector, since the upper arm will 
  // rotate appreciably in this way, giving a LEFT vector that points in a different direction than the one from the forearm. 
  Vectors G_start1;
  Vectors G_start3;
  Vectors xaxis; xaxis.x = 1.0, xaxis.y = 0, xaxis.z = 0;
  Vectors yaxis; yaxis.x = 0, yaxis.y = 1.0, yaxis.z = 0;
  Vectors zaxis; zaxis.x = 0, zaxis.y = 0, zaxis.z = 1.0;
  UpdateTF(initialpitch + 220);
  amplitude_update(MaxAmplitude);
  bool NEEDLEFT = true;
  int LEFTcounter = 1;
  int LEFTcounterMax = 100;
  int LEFTtimer1 = micros();
  int LEFTtimer2;
  float LEFTdt;
  float s1adj = 0; float s3adj = 0;
  float G_start3_mag;
  bool ready1; bool ready3;
  while(NEEDLEFT) {
    // Always fetch readings as soon as interrupts are noticed
    if (isrFired1) { // If our isr flag is set then clear the interrupts on the ICM
      isrFired1 = false;
      FetchReadingsSensor (myICM1, Sensor1_Acc, Sensor1_Gyro); //, Sensor1_Mag); // Not pulling Mag data
      Sensor1_Acc = vadd(Sensor1_Acc,Sensor1_GravityBias); // compensate for the accel bias; gyro bias handled in integratesensordata_advanced
      ready1 = true;
    }
    if (isrFired2) { // If our isr flag is set then clear the interrupts on the ICM
      isrFired2 = false;
      FetchReadingsSensor (myICM2, Sensor2_Acc, Sensor2_Gyro); //, Sensor2_Mag); 
      Sensor2_Acc = vadd(Sensor2_Acc,Sensor2_GravityBias);
    }
    if (isrFired3) { // If our isr flag is set then clear the interrupts on the ICM
      isrFired3 = false;
      FetchReadingsSensor (myICM3, Sensor3_Acc, Sensor3_Gyro); //, Sensor3_Mag);
      Sensor3_Acc = vadd(Sensor3_Acc,Sensor3_GravityBias);
      ready3 = true;
    }
    if (ready1 && ready3 && LEFTcounter == 1 && vmagSqrd(MOTIONCHECKSENSOR_Gyro) < motionthreshold_gyro) {
      // Subject should be at rest, so Acc should be the gravity vector pointing up; grab as reference point
      Sensor1_GravityLEFT = Sensor1_Acc; 
      Sensor2_GravityLEFT = Sensor2_Acc;
      Sensor3_GravityLEFT = Sensor3_Acc;
      G_start1 = Sensor1_Acc;
      G_start3 = Sensor3_Acc;
      G_start3_mag = vmag(G_start3);
      ready1 = false;
      ready3 = false;
      LEFTcounter++;
    } else if (ready1 && ready3 && (vmag(Sensor3_Acc) < LEFTtrigger * G_start3_mag)) { // if not moving significantly, just keep track of gravity
      // For first 250 ms or so, just keep track of gravity vector
      LEFTtimer2 = micros(); 
      LEFTdt = (((float)LEFTtimer2 - (float)LEFTtimer1) / 1000000.0);
      LEFTtimer1 = micros(); 
      Vectors rot1 = vscaler_mult(Sensor1_Gyro,LEFTdt); 
      Vectors rot3 = vscaler_mult(Sensor3_Gyro,LEFTdt);
      Vectors q1 = rot_quat(rot1,xaxis,yaxis,zaxis,zerov);
      Vectors q3 = rot_quat(rot3,xaxis,yaxis,zaxis,zerov);
      // If we rotated G_start with q, we'd be rotating it with the physical axis system (but G_start is fixed). 
      // The below equations are in the coordinates of that moving physical axis system
      // In that coordinate system, G_start appears to rotate in the direction opposite the physical axes' rotation
      // So, need to rotate with the conj of q.
      G_start1 = qvq(quat_conj(q1),G_start1);
      G_start3 = qvq(quat_conj(q3),G_start3); 
      ready1 = false;
      ready3 = false;
    } else if (ready1 && ready3) { // if moving, grab samples
      // As before, start by keeping track of gravity vector in our local physical axis system
      LEFTtimer2 = micros(); 
      LEFTdt = (((float)LEFTtimer2 - (float)LEFTtimer1) / 1000000.0);
      LEFTtimer1 = micros(); 
      Vectors rot1 = vscaler_mult(Sensor1_Gyro,LEFTdt); 
      Vectors rot3 = vscaler_mult(Sensor3_Gyro,LEFTdt);
      Vectors q1 = rot_quat(rot1,xaxis,yaxis,zaxis,zerov);
      Vectors q3 = rot_quat(rot3,xaxis,yaxis,zaxis,zerov);
      G_start1 = qvq(quat_conj(q1),G_start1);
      G_start3 = qvq(quat_conj(q3),G_start3); 
      // Now find quat reaigning the local coordinate system with gravity vector as zaxis
      Vectors qa1 = alignment_quat(G_start1,zaxis);
      Vectors qa3 = alignment_quat(G_start3,zaxis);
      // Transform coordinates using this and recompute q, the quat giving the felt rotation in these new coordinates
      q1 = rot_quat(rot1,qvq(qa1,xaxis),qvq(qa1,yaxis),qvq(qa1,zaxis),zerov);
      q3 = rot_quat(rot3,qvq(qa3,xaxis),qvq(qa3,yaxis),qvq(qa3,zaxis),zerov);
      // Extract the rotation felt about the zaxis = gravity, using atan2 to preserve the direction
      float phi1 = atan2(q1.z,q1.r); 
      float phi3 = atan2(q3.z,q3.r); 
      s1adj += phi1;
      s3adj += phi3;
      // Rotate the acceleration vectors about gravity opposite this direction
      Sensor1_Acc = qvq(formquat(-1*phi1,G_start1),Sensor1_Acc); // These equations are still in the local coordinates of the physical system
      Sensor3_Acc = qvq(formquat(-1*phi3,G_start3),Sensor3_Acc);
      // Finally save this under our running average
      Sensor1_GravityLEFT = vrunning_avg(Sensor1_GravityLEFT,Sensor1_Acc,LEFTcounter);
      Sensor2_GravityLEFT = vrunning_avg(Sensor2_GravityLEFT,Sensor2_Acc,LEFTcounter);
      Sensor3_GravityLEFT = vrunning_avg(Sensor3_GravityLEFT,Sensor3_Acc,LEFTcounter);
      // Advance counter
      LEFTcounter++;
      // Reset for next pass
      ready1 = false;
      ready3 = false;
    }
    if (LEFTcounter == LEFTcounterMax) {
      NEEDLEFT = false;
      SERIAL_PORT.println(F("Left-pointing gravity vectors saved! Vectors:"));
      SERIAL_PORT.print(Sensor1_GravityLEFT.x,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(Sensor1_GravityLEFT.y,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(Sensor1_GravityLEFT.z,3);
      SERIAL_PORT.print(Sensor2_GravityLEFT.x,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(Sensor2_GravityLEFT.y,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(Sensor2_GravityLEFT.z,3);
      SERIAL_PORT.print(Sensor3_GravityLEFT.x,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(Sensor3_GravityLEFT.y,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(Sensor3_GravityLEFT.z,3);
      SERIAL_PORT.println();
      s1adj = s1adj * (180.0/PI);
      s3adj = s3adj * (180.0/PI);
      SERIAL_PORT.println(F("Compensated angular displacement about Z/up (deg): "));
      SERIAL_PORT.print(F("Sensor 1: "));
      SERIAL_PORT.println(s1adj,1);
      SERIAL_PORT.print(F("Sensor 3: "));
      SERIAL_PORT.println(s3adj,1);
      SERIAL_PORT.println();
      amplitude_update(0); 
    }

}

void loop() {
  // put your main code here, to run repeatedly:

}

void integratesensordata(void) {
  // find the ROT vectors
  Sensor1_Rot = vintegrate_gyro(Sensor1_Gyro,Sensor1_GyroSaved);
  Sensor2_Rot = vintegrate_gyro(Sensor2_Gyro,Sensor2_GyroSaved); 
  Sensor3_Rot = vintegrate_gyro(Sensor3_Gyro,Sensor3_GyroSaved); 
  
  // subtract away the drift in the ROT vectors from gyro bias
  Sensor1_Rot = vsubtract(Sensor1_Rot,Sensor1_GyroBias); // recall that the GyroBias vectors have already had each component multipled by dt
  Sensor2_Rot = vsubtract(Sensor2_Rot,Sensor2_GyroBias);
  Sensor3_Rot = vsubtract(Sensor3_Rot,Sensor3_GyroBias);
}

Vectors vintegrate_gyro ( Vectors thisValue, Vectors &thisSavedValue ) {
  Vectors thisResult;
  thisResult = vscaler_mult(vdivide(vadd(thisValue,thisSavedValue),2.0),dt); // find average rate of rotation during dt and multiply that rate by dt and add to the old sum
  thisSavedValue = thisValue; // save the current reading for averaging the next time this function is called
  return(thisResult);
}

Vectors vdivide ( Vectors v, float r ) {
  Vectors w;
  w.x = v.x /r;
  w.y = v.y /r;
  w.z = v.z /r;
  return(w);
}

void compute_motionmodel_IP (float limb1length, float limb2length) {
  // Must find the coordinates of the sensor axes (a1x,a1y,a1z,a2x,a2y,a2z) in a coordinate system with z pointing up, y to the left, and x straight ahead.
  
  // Begin with each sensor's axes in the local coordinate frame (defined by those axes) of the sensor
  a1x.x = 1.0; a1x.y = 0; a1x.z = 0; // a1x, a1y, a1z are the x, y, and z axes of the sensor on the first limb segment
  a1y.x = 0; a1y.y = 1.0; a1y.z = 0;
  a1z.x = 0; a1z.y = 0; a1z.z = 1.0;
  a2x.x = 1.0; a2x.y = 0; a2x.z = 0; // a2x, a2y, a2z are the x, y, and z axes of the sensor on the second limb segment
  a2y.x = 0; a2y.y = 1.0; a2y.z = 0;
  a2z.x = 0; a2z.y = 0; a2z.z = 1.0;
  //Vectors yaxis = a1y; Vectors xaxis = a1x; Vectors zaxis = a1z; // just saving values for use later

  // For procedure below, we'll be using following idea: If quat q rotates v into w (v,w defined in frame fixed by v), then conj q takes vectors in the v coordinates and spits out their position in the coordinates of a frame fixed by w
  
  // For each sensor, generate a quaternion that rotates the sensor's z-axis into the IP gravity vector (which points up opposite the ground) and align's its y-axis (or negative y-axis, for left-handed people) with the LEFT vector's projection onto the plane perpendicular to the gravity vector.
  // By the above idea, the conjugate of the resulting quaternions will be a coordinate transform that puts the z-axis onto the gravity vector and x axis onto the mag vector's projection into the plane perpendicular to gravity -- call this the "geo-grav" frame
  // So, for example, assuming a2y started in its local coordinates (= 0,1,0), qvq(quat_conj(qct_s2),a2y) gives the physical y-axis in coordinates of the geo-grav frame
  Vectors qct_s1;
  Vectors qct_s2;
  if (!LEFT_HANDED) { // if right-handed
    qct_s1 = generate_coordinatetransform_wPlaneProj( a1z, LimbSeg1Sensor_GravityIP, a1y, LimbSeg1Sensor_LEFT );
    qct_s2 = generate_coordinatetransform_wPlaneProj( a2z, LimbSeg2Sensor_GravityIP, a2y, LimbSeg2Sensor_LEFT ); 
  } else { // else, if left-handed
    qct_s1 = generate_coordinatetransform_wPlaneProj( a1z, LimbSeg1Sensor_GravityIP, vscaler_mult(a1y,-1.0), LimbSeg1Sensor_LEFT ); // when left-handed, LimbSeg1Sensor_LEFT actually points right
    qct_s2 = generate_coordinatetransform_wPlaneProj( a2z, LimbSeg2Sensor_GravityIP, vscaler_mult(a2y,-1.0), LimbSeg2Sensor_LEFT ); // when left-handed, LimbSeg2Sensor_LEFT actually points right
  }
  
  // Next, find the quat that rotates the y-axis of the geo-grav coordinate frame around that frame's z-axis into alignment with the subject's left direction
  // We assume here that the limb segment 2 sensor's y-axis points opposite the limb segment 2's direction (forearm direction); we assume the forearm is pointing left, so the sensor axis is pointing right.
  // //Vectors qo = alignment_quat(yaxis, vscaler_mult( planeproj( qvq(quat_conj(qct_s2),a2y), xaxis, yaxis ), -1.0) ); // This equation is in the coordinates of the geo-grav frame; planeproj might be unnecessary?
  
  // Now want quaternions which will convert the local coordinates of each sensor into a coordinate frame w/ y-axis pointing left, x-axis pointing ahead, and z-axis pointing up
  // //Vectors q2 = quat_mult(quat_conj(qo),quat_conj(qct_s2)); // quat converting the coordinate system of the sensor on limb segment 2 into the desired external coordinate frame
  // //Vectors q1 = quat_mult(quat_conj(qo),quat_conj(qct_s1)); // quat converting the coordinate system of the sensor on limb segment 1 into the desired external coordinate frame

  // Do the actual conversion: 
  a1x = qvq(quat_conj(qct_s1),a1x); // these now are the initial-position coordinates of the sensor axes in an external reference frame w/ y-axis pointing left, x-axis pointing ahead, and z-axis pointing up
  a1y = qvq(quat_conj(qct_s1),a1y); // Notice that these are all "unshifted" (untranslated), meaning, for each sensor, coordinates assume that the "external" frame is centered on the sensor
  a1z = qvq(quat_conj(qct_s1),a1z); // So, we will "merge" the two systems by first setting the coordinates of the limbs and sensors, then shifting the axes with those 
  a2x = qvq(quat_conj(qct_s2),a2x); 
  a2y = qvq(quat_conj(qct_s2),a2y);
  a2z = qvq(quat_conj(qct_s2),a2z);

  // Find coordinates in our external system of limb segment 2 (term and origin, p2) and its sensor (s1). place origin of this external sensor at p2. 
  p2 = zerov; // assume the elbow rests at the origin of our external coordinate system
  p2.z = elbowthickness * 0.375; // midpoint of the elbow joint
  t = zerov;
  if (!LEFT_HANDED) {
    t.y = limb2length; // assume the forearm points along the y-axis (left direction) of our external coordinate system
  } else { // else, if left-handed
    t.y = limb2length * -1.0; // assume the forearm points along the negative y-axis (right direction) of our external coordinate system
  }
  if (t_as_wrist) {
    t.z = wristthickness * 0.5; // the midpoint of the wrist 
  } else if (t_as_marker) {
    t.z = wristthickness + wrist_marker_height; // the optical marker
  } else {
    t.z = wristthickness * 0.5;
  }
  s2 = vscaler_mult(t,s2ratio); // assume the limb2 sensor is 80% of the way down the limb (near the wrist)
  s2.z = wristthickness + sensorheighabovelimbthickness;
  // Assume limb segment 1 points in the opposite direction of its sensor's y-axis
  p1 = vscaler_mult(a1y,-1.0 * limb1length); 
  s1 = vadd(vscaler_mult(p1,s2ratio), vscaler_mult(a1z,(elbowthickness * 0.5) + sensorheighabovelimbthickness)); // assume the limb1 sensor is 50% of the way up the limb, and out in the direction of sensor 1's z-axis by half the elbow thickness plus 10mm
  
  // Finally, use the sensor coordinates to shift the axes out: 
  a1x = vadd(a1x,s1); 
  a1y = vadd(a1y,s1); 
  a1z = vadd(a1z,s1); 
  a2x = vadd(a2x,s2); 
  a2y = vadd(a2y,s2); 
  a2z = vadd(a2z,s2); 
  
  // Save with fast reset later: 
  p1_IP = p1; p2_IP = p2;
  t_IP = t;
  a1x_IP = a1x; a1y_IP = a1y; a1z_IP = a1z;
  s1_IP = s1;
  a2x_IP = a2x; a2y_IP = a2y; a2z_IP = a2z;
  s2_IP = s2;
}

Vectors generate_coordinatetransform_wPlaneProj( Vectors v1, Vectors v1rotated, Vectors v2, Vectors v2rotated ) {
  // when called to compute axes IP, v1rotated and v2rotated will be the IP gravity and mag vectors, which should be normalized
  Vectors v1rotated_norm = normalize(v1rotated);
  Vectors v2rotated_norm = normalize(v2rotated);
  Vectors q_ct1 = alignment_quat(v1,v1rotated_norm); 
  Vectors v2_new = qvq(q_ct1,v2);
  Vectors v1_new = qvq(q_ct1,v1);
  Vectors v2rotated_proj = planeproj ( v2rotated_norm , v2_new , crossp(v1_new,v2_new) ); // in the case for which function is written (compute axes IP), v1 = Zaxis, v2 = Xaxis, and so cross(v1,v2) = Yaxis, meaning we're projecting v2rotated (mag vector) onto x-y plane
  Vectors q_ct2 = alignment_quat(v2_new,v2rotated_proj);
  Vectors q_ct = quat_mult(q_ct2,q_ct1);
  return(q_ct); // this quat rotates v1 to v1rotated and rotates v2 to v2rotated
}

void compute_motionmodel (Vectors rotd1, Vectors rotd2) {

  /* This is a motion model for a two-pivot system. 
    Assume pivot point p1 is fixed. 
    Attached to p1 is a rigid rod with termination at point p2. 
    p2 is a second pivot point with another rigid rod attached, terminating at point t. 
    On the first rod between p1 and p2 is a 3-axis gyro, located at position s1, with gyro axes (unit) pointing to a1x, a2y, a2z.
    Similarly, between p2 and t is another gyro, located at s2, with axes pointing to a2x, a2y, and a2z. 
    If rotd1 gives the gyro readings (in radians) of the sensor at s1 and rotd2 gives the readings of the sensor at s2, 
    then the below code computes the new positions of all points for a given sensor reading. 
    I ran this code with readings at 1000Hz. */

  // Rotate points on limb segment 1 as if pivoting about s1
  Vectors q1 = rot_quat(rotd1,a1x,a1y,a1z,s1);
  Vectors p1_new = quatrot(q1,p1,s1); // p1 (shoulder joint) not actually rotating, but need to compute this to find translation for system
  Vectors p2_new = quatrot(q1,p2,s1);
  Vectors s1_new = s1; // quatrot(q1,s1,s1); // unecessary, as s1 is translated into origin
  Vectors a1x_new = quatrot(q1,a1x,s1);
  Vectors a1y_new = quatrot(q1,a1y,s1);
  Vectors a1z_new = quatrot(q1,a1z,s1);

  // This would have moved p1, which is actually stationary (shoulder joint in reach), so, translate the L1 system back to p1
  Vectors trans1 = vsubtract(p1,p1_new);
  p2_new = vadd(p2_new, trans1); // because p1 is stationary, we want to add the translation, i.e. p1_new + trans1 = p1
  s1_new = vadd(s1_new, trans1);
  a1x_new = vadd(a1x_new, trans1);
  a1y_new = vadd(a1y_new, trans1);
  a1z_new = vadd(a1z_new, trans1);

  // Rotate points on limb segment 2 as if pivoting about s2
  Vectors q2 = rot_quat(rotd2,a2x,a2y,a2z,s2);
  Vectors p2o_new = quatrot(q2,p2,s2); // The origin of limb segment 2; the first time (when computing p2_new) we treated p2 as the termination of LS1, now treating it as origin of LS2
  Vectors t_new = quatrot(q2,t,s2);
  Vectors s2_new = s2; // quatrot(q2,s2,s2); // unecessary, as s2 is translated into origin
  Vectors a2x_new = quatrot(q2,a2x,s2);
  Vectors a2y_new = quatrot(q2,a2y,s2);
  Vectors a2z_new = quatrot(q2,a2z,s2);

  // Must reattach the origin of limb segment 2 to the termination of limb segment 1
  Vectors trans2 = vsubtract(p2_new, p2o_new);
  t_new = vadd(t_new, trans2);
  s2_new = vadd(s2_new, trans2);
  a2x_new = vadd(a2x_new, trans2);
  a2y_new = vadd(a2y_new, trans2);
  a2z_new = vadd(a2z_new, trans2);
  
  // Finally, update the positions
  p2 = p2_new; // Remember, p1 should *nod* be updated to p1 new!
  t = t_new;
  a1x = a1x_new; a1y = a1y_new; a1z = a1z_new;
  s1 = s1_new;
  a2x = a2x_new; a2y = a2y_new; a2z = a2z_new;
  s2 = s2_new;
  
}

Vectors rot_quat ( Vectors r, Vectors a1, Vectors a2, Vectors a3, Vectors u ) {
  Vectors a1shift = vsubtract(a1,u); // It might be prodent to normalize these three shifted vectors, but given their construction when building the IP, they *should* be normalized already
  Vectors a2shift = vsubtract(a2,u); 
  Vectors a3shift = vsubtract(a3,u);
  Vectors qx = formquat(r.x,a1shift);
  Vectors qy = formquat(r.y,a2shift);
  Vectors qz = formquat(r.z,a3shift);
  Vectors q = quat_mult( qx, quat_mult( qy, qz) );
  q = normalize4(q); // we must ensure this is normalized, although it should be as well (if increased speed needed, drop this step).
  return(q);
}

Vectors quatrot ( Vectors q, Vectors v, Vectors p) {
  Vectors vshift = vsubtract(v,p);
  Vectors rotatedposition = qvq(q,vshift);
  rotatedposition = vadd(rotatedposition,p);
  return(rotatedposition);
}

Vectors vsubtract ( Vectors v1, Vectors v2 ) {
  Vectors output;
  output.x = v1.x - v2.x;
  output.y = v1.y - v2.y;
  output.z = v1.z - v2.z;
  return output;
}

Vectors vadd ( Vectors v1, Vectors v2 ) {
  Vectors output;
  output.x = v1.x + v2.x;
  output.y = v1.y + v2.y;
  output.z = v1.z + v2.z;
  return output;
}

Vectors normalize4 ( Vectors v ) {
  Vectors w;
  w = vdivide4(v,vmag4(v));
  return(w); 
}

float vmag4 ( Vectors thisVector ) {
  float MagV;
  MagV = sqrtf ( thisVector.r * thisVector.r + thisVector.x * thisVector.x + thisVector.y * thisVector.y + thisVector.z * thisVector.z );
  return MagV;
}

Vectors vdivide4 ( Vectors v, float r ) {
  Vectors w;
  w.r = v.r /r;
  w.x = v.x /r;
  w.y = v.y /r;
  w.z = v.z /r;
  return(w);
}

Vectors qvq ( Vectors thisQuat, Vectors thisPosition ) {
  // Will rotate the vector thisPosition by the quaternion thisQuat
  // Fewer computations than the other definition
  // Also, note that we're assuming our quaternions are unit quaternions (they should be, or should be very close). 

  thisPosition.r = 0;
  float qv_r = thisQuat.r * thisPosition.r - thisQuat.x * thisPosition.x - thisQuat.y * thisPosition.y - thisQuat.z * thisPosition.z ;
  float qv_x = thisQuat.r * thisPosition.x + thisQuat.x * thisPosition.r + thisQuat.y * thisPosition.z - thisQuat.z * thisPosition.y ; 
  float qv_y = thisQuat.r * thisPosition.y - thisQuat.x * thisPosition.z + thisQuat.y * thisPosition.r + thisQuat.z * thisPosition.x ;
  float qv_z = thisQuat.r * thisPosition.z + thisQuat.x * thisPosition.y - thisQuat.y * thisPosition.x + thisQuat.z * thisPosition.r ; 

  // This is the same operation as above, expect the sign is flipped everywhere we have a non-real quat component (since the second q in qvq is the inverse of q). 
  //float qvq_r = qv_r * thisQuat.r + qv_x * thisQuat.x + qv_y * thisQuat.y + qv_z * thisQuat.z ; // no point in computing r value of the output
  float qvq_x = qv_r * thisQuat.x * -1.0 + qv_x * thisQuat.r - qv_y * thisQuat.z + qv_z * thisQuat.y ;
  float qvq_y = qv_r * thisQuat.y * -1.0 + qv_x * thisQuat.z + qv_y * thisQuat.r - qv_z * thisQuat.x ;
  float qvq_z = qv_r * thisQuat.z * -1.0 - qv_x * thisQuat.y + qv_y * thisQuat.x + qv_z * thisQuat.r ;

  Vectors thisFinalPosition;
  thisFinalPosition.r = 0; // qvq_r ; // physically meaningful vectors have r = 0
  thisFinalPosition.x = qvq_x ;
  thisFinalPosition.y = qvq_y ;
  thisFinalPosition.z = qvq_z ;

  return(thisFinalPosition);
}

Vectors formquat ( float theta, Vectors v ) {
  // forms a quaternion that rotates space by angle theta (radians) about axis v
  Vectors q;
  float cos2theta = cosf(theta/2.0);
  float sin2theta = sinf(theta/2.0);
  q.r = cos2theta;
  q.x = sin2theta * v.x;
  q.y = sin2theta * v.y;
  q.z = sin2theta * v.z;
  return (q); 
}

Vectors alignment_quat ( Vectors v, Vectors w ) {
  // Want to form a quaternion that will rotate space so that one vector v falls onto another w? this does that
  float phi = vangle_rad(v,w);
  float sphi = sinf(phi/2.0);
  Vectors crossvw = crossp(v,w);
  float crossvw_m = vmag(crossvw);
  if ( crossvw_m != 0 ) crossvw = vdivide(crossvw,crossvw_m);
  Vectors q;
  q.r = cosf(phi/2.0);
  q.x = sphi * crossvw.x;
  q.y = sphi * crossvw.y;
  q.z = sphi * crossvw.z;
  q = normalize4(q);
  return(q); // this quat will rotate v into w
}

Vectors quat_conj ( Vectors q ) {
  // forms the conjugate of a quaternion q
  Vectors qc;
  qc.r = q.r;
  qc.x = -1 * q.x;
  qc.y = -1 * q.y;
  qc.z = -1 * q.z;
  return(qc); 
}

Vectors quat_mult ( Vectors q1, Vectors q2 ) {
  Vectors q = vadd( vscaler_mult(q2,q1.r) , vadd( vscaler_mult(q1,q2.r) , crossp(q1,q2) ) );
  q.r = q1.r * q2.r - dotp(q1,q2); 
  return(q);
}

Vectors vscaler_mult ( Vectors thisVector1, float thisfloat ) {
  Vectors output;
  output.x = thisVector1.x * thisfloat;
  output.y = thisVector1.y * thisfloat;
  output.z = thisVector1.z * thisfloat;
  return(output);
}

float dotp ( Vectors thisVector1, Vectors thisVector2 ) {
  float dotproduct;
  dotproduct = thisVector1.x * thisVector2.x + thisVector1.y * thisVector2.y + thisVector1.z * thisVector2.z ;
  return dotproduct;
}

Vectors crossp ( Vectors thisVector1, Vectors thisVector2 ) {
  Vectors crossproduct;
  crossproduct.x = thisVector1.y * thisVector2.z - thisVector1.z * thisVector2.y ;
  crossproduct.y = thisVector1.z * thisVector2.x - thisVector1.x * thisVector2.z ;
  crossproduct.z = thisVector1.x * thisVector2.y - thisVector1.y * thisVector2.x ;
  return(crossproduct);
}

Vectors planeproj ( Vectors u, Vectors n1, Vectors n2 ) { 
  // Assumes n1 and n2 are orthogonal ???
  Vectors projection;
  if ( vsame(n1,n2) ) { // if n1 and n2 are the same (and so don't define a plane), then take vector projection
    float phi = vangle_rad(u,n1);
    float scalarproj = vmag(u) * cosf(phi); 
    projection = vscaler_mult( normalize(n1) , scalarproj );
  } else { // find the projection of vector u onto the plane formed by othogonal vectors n1 and n2
    Vectors n = crossp(n1,n2);
    Vectors normproj = vscaler_mult( n, dotp(u,n) / ( vmag(n) * vmag(n) ) ); 
    projection = vsubtract(u,normproj); 
  }
  return(projection); // projection of a vector u onto the plane formed by n1 and n2
}

float vangle_rad ( Vectors v, Vectors w ) {
  // gives angle phi between vectors v and w
  float phi;
  if ( vsame(v,w) ) {
    phi = 0;
  } else if ( vsame(v,vscaler_mult(w,-1.0)) ) {
    phi = PI;
  } else {
    Vectors vn = normalize(v);
    Vectors wn = normalize(w);
    float x = dotp(vn,wn); // point of normalizing is so that these magnitudes are 1;  / ( vmag(vn) * vmag(wn) );
    if ( x > 1 ) x = 1;
    if ( x < -1 ) x = -1;
    phi = acosf(x);
  }
  return(phi); // returns radians
}

bool vsame ( Vectors v, Vectors w ) {
  if ( v.x == w.x && ( v.y == w.y && v.z == w.z ) ) {
    return(true);
  } else {
    return(false);
  }
}
