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

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public static Transform3d robotToCam1 = new Transform3d(new Translation3d(0.18318, -0.20378,0.6653), new Rotation3d(0, Units.degreesToRadians(-41),0));
  public static Transform3d robotToCam2 = new Transform3d(new Translation3d(0.18227, 0.2038, 0.7103), new Rotation3d(0,Units.degreesToRadians(51),0));
  public static PhotonCamera camera1 = new PhotonCamera(Constants.apriltagCamera1Name);
  public static PhotonCamera camera2 = new PhotonCamera(Constants.apriltagCamera2Name);
  public static PhotonPipelineResult result1 = new PhotonPipelineResult();
  public static PhotonPipelineResult result2 = new PhotonPipelineResult();

  public static PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam1);//new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam1);
  public static PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam2);
  public static Vector2D[] deltaBuffer = new Vector2D[50];
  public static double oldTimestamp1 = 0;
  public static double newTimestamp1 = 0;
  public static double latency1 = 0;
  public static double oldTimestamp2 = 0;
  public static double newTimestamp2 = 0;
  public static double latency2 = 0;
  public static double sumX = 0;
  public static double sumY = 0;
  public static double ambiguity1;
  public static double ambiguity2;

  private static int currentAutoCoral = 0;

  public static double lastTimestamp1 = 0;
  public static double lastTimestamp2 = 0;
  public static double trueLatency = 0;

  public static Boolean camCheck1() {
    //var result = camera1.getLatestResult();
    boolean targetCheck = result1.hasTargets();
    double[] distances = new double[result1.getTargets().size()];
    double distance = 100;
    try {
      if (targetCheck) {
        for(int i = 0; i < result1.getTargets().size(); i++)
        {
          distances[i] = PhotonUtils.calculateDistanceToTargetMeters(robotToCam1.getZ(), aprilTagFieldLayout.getTagPose(result1.getTargets().get(i).getFiducialId()).get().getZ(), robotToCam1.getRotation().getY(), Units.degreesToRadians(result1.getTargets().get(i).getPitch()));
        }
        double minDist = 1000;
        for(int i = 0; i < distances.length; i++)
        {
          if(distances[i] < minDist)
          {
            minDist = distances[i];
            distance = distances[i];
          }
        }
        //distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCam1.getZ(), aprilTagFieldLayout.getTagPose(result1.getBestTarget().getFiducialId()).get().getZ(), robotToCam1.getRotation().getY(), Units.degreesToRadians(result1.getBestTarget().getPitch()));
      }
      SmartDashboard.putNumber("distance1", distance);
    }
    catch (Exception e) {
      //System.out.println("Caught Error: " + e);
    }

    return targetCheck && ((result1.getTargets().size() >= 1 && distance < 5) || (ambiguity1 < 0.05 && ambiguity1 > 0 && distance < 3));
  }

  public static void ResetAngle() {
    Constants.gyro.setYaw(180);
  }

  public static Boolean camCheck2() {
    //var result2 = camera2.getLatestResult();
    double distance = 100;
    boolean targetCheck = result2.hasTargets();
    double[] distances = new double[result2.getTargets().size()];
    try {
      if (targetCheck) {
        for(int i = 0; i < result2.getTargets().size(); i++)
        {
          distances[i] = PhotonUtils.calculateDistanceToTargetMeters(robotToCam2.getZ(), aprilTagFieldLayout.getTagPose(result2.getTargets().get(i).getFiducialId()).get().getZ(), robotToCam2.getRotation().getY(), Units.degreesToRadians(result2.getTargets().get(i).getPitch()));
        }
        double minDist = 1000;
        for(int i = 0; i < distances.length; i++)
        {
          if(distances[i] < minDist)
          {
            minDist = distances[i];
            distance = distances[i];
          }
        }
        //distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCam2.getZ(), aprilTagFieldLayout.getTagPose(result2.getBestTarget().getFiducialId()).get().getZ(), robotToCam2.getRotation().getY(), Units.degreesToRadians(result2.getBestTarget().getPitch()));
      }
      SmartDashboard.putNumber("distance2", distance);
    }
    catch (Exception e) {
      //System.out.println("Caught Error: " + e);
    }
    return targetCheck && ((result2.getTargets().size() >= 1 && distance < 5) || (ambiguity2 < 0.05 && ambiguity2 > 0 && distance < 3));
  }

  public static PhotonTrackedTarget obtainTarget1() {
    //Sends back the most clear target and its data
    if (result1.hasTargets()) {
      return result1.getBestTarget();
    }
    else {
      return new PhotonTrackedTarget(); //(0, 0, 0, 0, 0, null, null, -1, null, null);
    }

    //In Java/C++, You must always check if the result has a target via hasTargets()/HasTargets()
    //before getting targets or else you may get a null pointer exception. Further, you must use
    //the same result in every subsequent call in that loop.

  }
  public static PhotonTrackedTarget obtainTarget2() {
    //Sends back the most clear target and its data
    if (result2.hasTargets()) {
      return result2.getBestTarget();
    }
    else {
      return new PhotonTrackedTarget(); //(0, 0, 0, 0, 0, null, null, -1, null, null);
    }

    //In Java/C++, You must always check if the result has a target via hasTargets()/HasTargets()
    //before getting targets or else you may get a null pointer exception. Further, you must use
    //the same result in every subsequent call in that loop.

  }

  public static Pose2d getEstimatedGlobalPose1() {
    //var emptyTarget = new PhotonTrackedTarget();
    if (!camCheck1()) {
      return previousPosition;
    }

    /*if (photonPoseEstimator.update().isPresent()) {
      return photonPoseEstimator.update().orElse(null).estimatedPose.toPose2d();
    } */


    try {
      EstimatedRobotPose poseResult = photonPoseEstimator1.update(result1).get();
      lastTimestamp1 = poseResult.timestampSeconds;
      return poseResult.estimatedPose.toPose2d();
    }
    catch(Exception e) {
      //System.out.println("Caught Error: " + e);
    }

    return previousPosition;
  }
  public static Pose2d getEstimatedGlobalPose2() {
    //var emptyTarget = new PhotonTrackedTarget();
    if (!camCheck2()) {
      return previousPosition;
    }

    /*if (photonPoseEstimator.update().isPresent()) {
      return photonPoseEstimator.update().orElse(null).estimatedPose.toPose2d();
    } */


    try {
      EstimatedRobotPose poseResult = photonPoseEstimator2.update(result2).get();
      lastTimestamp2 = poseResult.timestampSeconds;
      return poseResult.estimatedPose.toPose2d();
    }
    catch(Exception e) {
      //System.out.println("Caught Error: " + e);
    }

    return previousPosition;
  }

  /*public boolean isValid(Pose2d oldPose, Pose2d newPose) {
    double maxMovement = Constants.swerveMaxSpeed * camera1.getLatestResult().getLatencyMillis() * 0.001;
    double currentMovement = Functions.Pythagorean(oldPose.getX() - newPose.getX(), oldPose.getY() - newPose.getY());

    if (currentMovement > maxMovement) {
      return false;
    }
    else {
      return true;
    }
  }*/

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
    result1 = camera1.getLatestResult();
    result2 = camera2.getLatestResult();
    oldTimestamp1 = newTimestamp1;
    newTimestamp1 = result1.getTimestampSeconds();
    oldTimestamp2 = newTimestamp2;
    newTimestamp2 = result2.getTimestampSeconds();
    latency1 = (newTimestamp1-oldTimestamp1)*1000.;
    latency2 = (newTimestamp2-oldTimestamp2)*1000.;
    try
    {
      ambiguity1 = obtainTarget1().getPoseAmbiguity();
    }
    catch(Exception e)
    {
      ambiguity1 = -1;
    }
    try
    {
      ambiguity2 = obtainTarget2().getPoseAmbiguity();
    }
    catch(Exception e)
    {
      ambiguity2 = -1;
    }

    for (int i = 49; i > 0; i--) {
      deltaBuffer[i] = deltaBuffer[i - 1];
    }
    deltaBuffer[0] = velocity;

    previousPosition = robotPosition;
    Pose2d globalPose1 = robotPosition;
    Pose2d globalPose2 = robotPosition;
    if (camCheck1()) {
      //if((result1.getTargets().size() >= 2 || result1.getBestTarget().getPoseAmbiguity() < 0.2))
      //{
        globalPose1 = getEstimatedGlobalPose1();
      //}
      //else
      //{
        // globalPose1 = PhotonUtils.estimateFieldToRobot(robotToCam1.getZ(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(), target.getPitch(), Rotation2d.fromDegrees(-target.getYaw()), robotPosition.getRotation(), robotToCam1);
        //   PhotonTrackedTarget target = result1.getBestTarget();
        //   target.getPoseAmbiguity()
        //   double why;
        //   Translation2d camToTargetTranslate = PhotonUtils.estimateCameraToTargetTranslation(0, null)
        //   Transform2d camToTargetTransform = PhotonUtils.estimateCameraToTarget(null, globalPose2, null);
        //   Transform2d cameraToRobot =new Transform2d(-robotToCam1.getX(), -robotToCam1.getY(), new Rotation2d(robotToCam1.getRotation().getY()));
        //   globalPose1 = PhotonUtils.estimateFieldToRobot(camToTargetTransform, aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d(), cameraToRobot);
      //}
    }
    if(camCheck2())
    {
      //if(result2.getTargets().size() >= 2 || result2.getBestTarget().getPoseAmbiguity() < 0.2)
      //{
        globalPose2 = getEstimatedGlobalPose2();
      //}
    }
    if(camCheck1() && camCheck2())
    {
      Pose2d combinedPoses = new Pose2d((globalPose1.getX() + globalPose2.getX())/2.0, (globalPose1.getY() + globalPose2.getY())/2.0, robotPosition.getRotation());
      robotPosition = combinedPoses;
      //trueLatency = Timer.getFPGATimestamp() - lastTimestamp;
        /*for (int i = 0; i < (latency / 20); i++) {
          if (deltaBuffer[i] != null)
          {
            sumX += deltaBuffer[i].x;
            sumY += deltaBuffer[i].y;
          }
        }*/

        //robotPosition = new Pose2d(globalPose.getX() + sumX, globalPose.getY() + sumY, globalPose.getRotation());
    }
    else if(camCheck1())
    {
      robotPosition = new Pose2d(globalPose1.getX(), globalPose1.getY(), robotPosition.getRotation());
    }
    else if(camCheck2())
    {
      robotPosition = new Pose2d(globalPose2.getX(), globalPose2.getY(), robotPosition.getRotation());
    }
    else {
      // no apriltags detected
      robotPosition  = new Pose2d(robotPosition.getX() + velocity.x*0.02, robotPosition.getY() + velocity.y*0.02, robotPosition.getRotation());
    }
    // if (camCheck1() && getEstimatedGlobalPose() != null) {
    //   robotPosition = getEstimatedGlobalPose();
    // }
//     double RawVisionX = -1;
//     double RawVisionY = -1;
//     try {
//       System.out.println("I'm trying I promise");
//       Optional<EstimatedRobotPose> erpo = photonPoseEstimator.update();
//       System.out.println("Found this shit");
//       if (!erpo.isEmpty()) {
//         EstimatedRobotPose erp = erpo.get();
//         System.out.println("Found erp");
//         Pose3d er = erp.estimatedPose;
//         System.out.println("Found er");
//         RawVisionX = er.getX();
//         System.out.println("got x");
//         RawVisionY = er.getY();
//         System.out.println("got y");
//       }

// //      RawVisionX = photonPoseEstimator.update().get().estimatedPose.getX();
// //      RawVisionY = photonPoseEstimator.update().get().estimatedPose.getY();
//     } catch (Exception e) {
//       System.out.println("exception found @ 197");
//       System.out.println(e);
//     }
//     SmartDashboard.putNumber("RawVisionX", RawVisionX);
//     SmartDashboard.putNumber("RawVisionY", RawVisionY);



}

  // truespeed = deltaBuffer[camera1.getLatestResult().getLatencyMillis() / 20.0];

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
