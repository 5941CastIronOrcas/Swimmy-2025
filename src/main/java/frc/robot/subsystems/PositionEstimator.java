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
import frc.robot.subsystems.CameraConf;
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

  public static ArrayList<CameraConf> cams = new ArrayList<CameraConf>();
    public CameraConf cam1 = new CameraConf();
    public CameraConf cam2 = new CameraConf();
  public static Vector2D[] deltaBuffer = new Vector2D[50];
  public static double sumX = 0;
  public static double sumY = 0;

  private static int currentAutoCoral = 0;

  public static double lastTimestamp1 = 0;
  public static double lastTimestamp2 = 0;
  public static double trueLatency = 0;


  public static void ResetAngle() {
    Constants.gyro.setYaw(180);
  }


  public static double distToSpeaker() {
    double supposed = Functions.Pythagorean((Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x)-robotPosition.getX(), (Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y)-robotPosition.getY());
    //return supposed;
    return 0.967145602537*supposed+0.367526;
  }
  public static double angleToSpeaker()
  {
    return 90-Math.toDegrees(Math.atan2((Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y) - robotPosition.getY(),(Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x) - robotPosition.getX()));// + Constants.shootYawOffset;
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
  public static boolean atSpeakerAngle() {
    return Math.abs(Functions.DeltaAngleDeg(angleToSpeaker(), robotPosition.getRotation().getDegrees()))<Constants.speakerAngleVariation;
  }

  public static void removeClosestCoral() {
    realCoralList.remove(nearestAutoCoral());
  }


ArrayList<CameraConf> cameras = new ArrayList<CameraConf>();

  @Override
  public void periodic() {
    cam1.Conf(Constants.apriltagCamera1Name, 0.18318, -0.20378, 0.6653, 0, -41, 0);
    cam2.Conf(Constants.apriltagCamera2Name,  0.18227, 0.2038, 0.7103, 0, 51 ,0);
    cameras.add(0, cam1);
    cameras.add(1, cam2);


    SmartDashboard.putNumber("corals remaining", realCoralList.size());
    SmartDashboard.putNumber("nearestAutoCoral", nearestAutoCoral());
    SmartDashboard.putNumber("distClosestCoral", distToClosestCoral());
    // This method will be called once per scheduler run
    robotYawDriverRelative = Functions.DeltaAngleDeg(0, -Constants.gyro.getYaw().getValueAsDouble());
    //robotYawRate = Constants.gyro.getRate();
    robotYawRate = -(Constants.gyro.getYaw().getValueAsDouble() - gyroYawOld)/0.02;
    gyroYawOld = Constants.gyro.getYaw().getValueAsDouble();

    if(Robot.isRedAlliance) {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative - 90))));
    } else if (Robot.isBlueAlliance) {
            robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative + 90))));
    } else {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative))));
    }
     velocity.x = ((
         (Math.sin(Math.toRadians(SwerveSubsystem.frModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.GetVelocity())
       + (Math.sin(Math.toRadians(SwerveSubsystem.flModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.GetVelocity())
       + (Math.sin(Math.toRadians(SwerveSubsystem.brModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.GetVelocity())
       + (Math.sin(Math.toRadians(SwerveSubsystem.blModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.GetVelocity()))
        / 4.0);
    velocity.y = ((
         (Math.cos(Math.toRadians(SwerveSubsystem.frModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.GetVelocity())
       + (Math.cos(Math.toRadians(SwerveSubsystem.flModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.GetVelocity())
       + (Math.cos(Math.toRadians(SwerveSubsystem.brModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.GetVelocity())
       + (Math.cos(Math.toRadians(SwerveSubsystem.blModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.GetVelocity()))
        / 4.0);

    for (int i = 49; i > 0; i--) {
      deltaBuffer[i] = deltaBuffer[i - 1];
    }
    deltaBuffer[0] = velocity;

    previousPosition = robotPosition;
    double camsactive = 0;
    double combx = 0;
    double comby = 0;

    for(int i = 0; i <= cameras.size(); i++){
            if(cameras.get(i).camCheck()){
                camsactive++;

                combx += cameras.get(i).getEstimatedGlobalPose(previousPosition).getX();
                comby += cameras.get(i).getEstimatedGlobalPose(previousPosition).getY();
            }

        }
if(camsactive != 0){
   robotPosition = new Pose2d(combx/camsactive, comby/camsactive, robotPosition.getRotation());
}else{

     robotPosition  = new Pose2d(robotPosition.getX() + velocity.x*0.02, robotPosition.getY() + velocity.y*0.02, robotPosition.getRotation());
}



}


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
