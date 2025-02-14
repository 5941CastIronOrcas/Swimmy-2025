// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GamePieceDetector extends SubsystemBase {

  //public static PhotonCamera camera = new PhotonCamera(Constants.coralDetectionCameraName);
  public static boolean coralVisible = false;
  public static double coralPitch = 0;
  public static double coralYaw = 0;
  public static double coralDist = 0;
 // public static PhotonPipelineResult result = camera.getLatestResult();
 // public static PhotonTrackedTarget target = result.getBestTarget();
  

  public GamePieceDetector() {}

 /*  public static Boolean camCheck() {
    return result.hasTargets();
  }
  
 
  public static PhotonTrackedTarget obtainTarget() {
      return result.getBestTarget();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    target = obtainTarget();
    coralVisible = camCheck() && ArmSubsystem.armAngle < 40;
  
    if(coralVisible)
    {
      try
      {
        coralPitch = target.getPitch() + ArmSubsystem.armAngle;
        coralYaw = target.getYaw();
      }
      catch (Exception e)
      {

      }
    }
    else
    {
      coralPitch = 0;
      coralYaw = 0;
    }

    coralDist = Constants.coralCameraForwardOffset-Constants.coralCameraHeight / Math.tan(Math.toRadians(coralPitch + Constants.coralCameraAngle));
  }

  
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }*/
}
