// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoSequences;
import frc.robot.CameraConf;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;
import frc.robot.utilityObjects.Vector2D;

import java.util.ArrayList;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PositionEstimator extends SubsystemBase {
  public static ArrayList<Vector2D> realCoralList = new ArrayList<>();
  public static double robotYawDriverRelative = 0;
  private static double gyroYawOld = 0;
  public static double robotYawRate = 0;
  public static Pose2d robotPosition = new Pose2d();
  public static Pose2d previousPosition = new Pose2d();
  public static Vector2D velocity = new Vector2D(0, 0);
  

  public static ArrayList<CameraConf> cameras = new ArrayList<CameraConf>();
  public static CameraConf cam1 = new CameraConf("camLeft", 0.1899, 0.1937, 0.6728, 0, 26, -20);
  public static CameraConf cam2 = new CameraConf("camRight",  0.1877, -0.1961, 0.7045, 0, -49 ,15);
 // public static CameraConf cam3 = new CameraConf("camBLeft", -0.55448, 0.27523, 0.1857, 0, -20, 33);
  //public static CameraConf cam4 = new CameraConf("camBRight", -0.5549, -0.2751, 0.1857, 0, -20, -33);
  public static Vector2D[] deltaBuffer = new Vector2D[50];
  public static double sumX = 0;
  public static double sumY = 0;

  private static int currentAutoCoral = 0;

  public static double lastTimestamp1 = 0;
  public static double lastTimestamp2 = 0;
  public static double trueLatency = 0;

  public static Pose2d[][] reefPositionPose2ds;
  public static Pose2d[] coralStationPose2ds;


  public static void ResetAngle() {
    Constants.gyro.setYaw(180);
  }


  public static double distToSpeaker() {
    double supposed = Functions.Pythagorean((Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x)-
    robotPosition.getX(), (Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y)-robotPosition.getY());
    //return supposed;
    return 0.967145602537*supposed+0.367526;
  }
  public static double angleToSpeaker()
  {
    return 90-Math.toDegrees(Math.atan2((Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y) -
    robotPosition.getY(),(Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x) - robotPosition.getX()));// + Constants.shootYawOffset;
  }
  public static int nearestAutoCoral() {
    try {
    int id = 0;
      double minDist = 100000;
      for (int i = 0; i < realCoralList.size(); i++) {
        double d = Functions.Pythagorean(realCoralList.get(i).x - robotPosition.getX(), realCoralList.get(i).y - robotPosition.getY());
        if (d < minDist) {
          id = AutoSequences.coralList[i];
          minDist = d;
        }
      }
      return id;
    } catch (Exception e) {
      System.out.println("The duckt tape helped again + " + e);
      return 0;
    }
  }
  public static double distToClosestCoral() {
    int n = nearestAutoCoral();
    try
    {
      return Functions.Pythagorean(realCoralList.get(n).x - robotPosition.getX(), realCoralList.get(n).y - robotPosition.getY());
    }
    catch(Exception e)
    {
      return 10000000;
    }
  }
  public static double angleToClosestCoral() {
    int n = nearestAutoCoral();
    return Math.toDegrees(Math.atan2(realCoralList.get(n).x - robotPosition.getX(), realCoralList.get(n).y - robotPosition.getY()));
  }
  public static int currentAutoCoral() {
    return currentAutoCoral;
  }
  public static void nextAutoCoral() {
    currentAutoCoral ++;
  }
  public static double distToCurrentCoral() {
    int n = currentAutoCoral;
    try
    {
      return Functions.Pythagorean(realCoralList.get(n).x - robotPosition.getX(), realCoralList.get(n).y - robotPosition.getY());
    }
    catch(Exception e)
    {
      return 10000000;
    }
  }
  public static double angleToCurrentCoral() {
    int n = currentAutoCoral;


    return Math.toDegrees(Math.atan2(realCoralList.get(n).x - robotPosition.getX(), realCoralList.get(n).y - robotPosition.getY()));
  }
  /*public static boolean atSpeakerAngle() {
    return Math.abs(Functions.DeltaAngleDeg(angleToSpeaker(), robotPosition.getRotation().getDegrees()))
    <Constants.speakerAngleVariation;
  }*/

  public static void removeClosestCoral() {
    realCoralList.remove(nearestAutoCoral());
  }

  public static Pose2d[][] getReefPositions() {
    Pose2d[] apriltags;
    if (Robot.isRedAlliance) apriltags = Constants.redReefApriltags;
    else apriltags = Constants.blueReefApriltags;
    double[] dists = Constants.reefDist;
    Pose2d[][] positions = new Pose2d[apriltags.length*2][dists.length];
    for (int i = 0; i < apriltags.length; i++) {
      Pose2d pos = apriltags[i];
      Rotation2d angle = pos.getRotation();
      for (int j = 0; j < dists.length; j++) {
        Pose2d rotatedPosL = Functions.RotatePose(new Pose2d(Constants.reefDist[j], -Constants.reefSideOffset+Constants.leftReefSideOffset, new Rotation2d(0)), pos.getRotation().getRadians());
        Pose2d rotatedPosR = Functions.RotatePose(new Pose2d(Constants.reefDist[j], Constants.reefSideOffset+Constants.rightReefSideOffset, new Rotation2d(0)), pos.getRotation().getRadians());
        positions[i*2][j] = new Pose2d(pos.getX()+rotatedPosL.getX(), pos.getY()+rotatedPosL.getY(), new Rotation2d(Math.toRadians(-angle.getDegrees()-90)));
        positions[i*2+1][j] = new Pose2d(pos.getX()+rotatedPosR.getX(), pos.getY()+rotatedPosR.getY(), new Rotation2d(Math.toRadians(-angle.getDegrees()-90)));
      }
    }
    return positions;
  }

  public static Pose2d[] getCoralStationPositions() {
    Pose2d[] apriltags;
    if (Robot.isRedAlliance) apriltags = Constants.redCoralStationsApriltags;
    else apriltags = Constants.blueCoralStationsApriltags;
    Pose2d[] positions = new Pose2d[apriltags.length*2];
    for (int i = 0; i < apriltags.length; i++) {
      Pose2d pos = apriltags[i];
      Rotation2d angle = pos.getRotation();
      Pose2d rotatedPosL = Functions.RotatePose(new Pose2d(Constants.coralStationDist, -Constants.coralStationSideOffset,
      new Rotation2d(0)), pos.getRotation().getRadians());
      Pose2d rotatedPosR = Functions.RotatePose(new Pose2d(Constants.coralStationDist, Constants.coralStationSideOffset,
      new Rotation2d(0)), pos.getRotation().getRadians());
      positions[i*2] = new Pose2d(pos.getX()+rotatedPosL.getX(), pos.getY()+rotatedPosL.getY(), new Rotation2d(Math.toRadians(-angle.getDegrees()-90)));
      positions[i*2+1] = new Pose2d(pos.getX()+rotatedPosR.getX(), pos.getY()+rotatedPosR.getY(), new Rotation2d(Math.toRadians(-angle.getDegrees()-90)));
    }
    return positions;
  }
  public static void calculateObjectiveLocations()
  {
    reefPositionPose2ds = getReefPositions();
    coralStationPose2ds = getCoralStationPositions();
  }

  public static Pose2d getNearest(Pose2d[] positions) {
    double dist = 10000;
    Pose2d closest = new Pose2d();
    for (int i = 0; i < positions.length; i++) {
      double d = Functions.Pythagorean(positions[i].getX()-robotPosition.getX(), positions[i].getY()-robotPosition.getY());
      if (d < dist) {
        dist = d;
        closest = positions[i];
      }
    }
    if (dist == 1000) closest = robotPosition;
    return closest;
  }

  public static Pose2d getNearestReefAtLevel() {
    int level = (ArmSubsystem.elevatorLevel-1<0)?0:(ArmSubsystem.elevatorLevel-1);
    Pose2d[] reefPose2dsAtLevel = new Pose2d[reefPositionPose2ds.length];
    for (int i=0; i<reefPositionPose2ds.length; i++) {
      reefPose2dsAtLevel[i] = reefPositionPose2ds[i][level];
    }
    return getNearest(reefPose2dsAtLevel);
  }

  public static Pose2d getNearestAtLevel() {
    int level = (ArmSubsystem.elevatorLevel-1<0)?0:(ArmSubsystem.elevatorLevel-1);
    Pose2d[] reefPose2dsAtLevel = new Pose2d[reefPositionPose2ds.length];
    for (int i=0; i<reefPositionPose2ds.length; i++) {
      reefPose2dsAtLevel[i] = reefPositionPose2ds[i][level];
    }
    return getNearest(Functions.CombinePose2dArrays(coralStationPose2ds, reefPose2dsAtLevel));
  }



public PositionEstimator(){
  cameras.add(0, cam1);
  cameras.add(1, cam2);
  //cameras.add(2, cam3);
 // cameras.add(3, cam4);
    }

  @Override
  public void periodic() {


    SmartDashboard.putNumber("corals remaining", realCoralList.size());
    SmartDashboard.putNumber("nearestAutoCoral", nearestAutoCoral());
    SmartDashboard.putNumber("distClosestCoral", distToClosestCoral());
    // This method will be called once per scheduler run
    robotYawDriverRelative = Functions.DeltaAngleDeg(0, -Constants.gyro.getYaw().getValueAsDouble());
    //robotYawRate = Constants.gyro.getRate();
    robotYawRate = -(Constants.gyro.getYaw().getValueAsDouble() - gyroYawOld)/0.02;
    gyroYawOld = Constants.gyro.getYaw().getValueAsDouble();

    if(Robot.isRedAlliance) {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(),
      new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, (robotYawDriverRelative + 90)))));
    } else if (Robot.isBlueAlliance) {
            robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(),
            new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, (robotYawDriverRelative - 90)))));
    } else {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(),
      new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative))));
    }
     velocity.x = ((
       - (Math.sin(Math.toRadians(SwerveSubsystem.frModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.GetVelocity())
       - (Math.sin(Math.toRadians(SwerveSubsystem.flModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.GetVelocity())
       - (Math.sin(Math.toRadians(SwerveSubsystem.brModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.GetVelocity())
       - (Math.sin(Math.toRadians(SwerveSubsystem.blModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.GetVelocity()))
        / 4.0);
    velocity.y = ((
       - (Math.cos(Math.toRadians(SwerveSubsystem.frModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.GetVelocity())
       - (Math.cos(Math.toRadians(SwerveSubsystem.flModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.GetVelocity())
       - (Math.cos(Math.toRadians(SwerveSubsystem.brModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.GetVelocity())
       - (Math.cos(Math.toRadians(SwerveSubsystem.blModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.GetVelocity()))
        / 4.0);

    /*for (int i = deltaBuffer.length-1; i > 0; i--) {
      deltaBuffer[i] = deltaBuffer[i - 1];
    }*/
    deltaBuffer[0] = velocity;

    previousPosition = robotPosition;


    double camsactive = 0;
    double combx = 0;
    double comby = 0;

    for(int i = 0; i < cameras.size(); i++){
              CameraConf currentcam = cameras.get(i);
              currentcam.refresh();
            if(currentcam.camCheck()){
                Pose2d estimpose = currentcam.getEstimatedGlobalPose(previousPosition);
                camsactive++;

                combx += estimpose.getX();
                comby += estimpose.getY();
                //System.out.println(combx);
                //System.out.println(comby);
            }

        }
if(camsactive != 0){
   robotPosition = new Pose2d(combx/camsactive, comby/camsactive, robotPosition.getRotation());
}else{

     robotPosition  = new Pose2d(robotPosition.getX() + velocity.x*0.02,
     robotPosition.getY() + velocity.y*0.02, robotPosition.getRotation());
}



}


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
