// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class ClimberSubsystem extends SubsystemBase {
  public static DutyCycleEncoder climberEncoder = Constants.climberEncoder;
  public static double climberAngle = 0;
  public static double oldClimberAngle = 0;
  public static double winchAngle = 0;
  public static double climberVelocity = 0;
  public static double clawAngle = 0;
  public static boolean pivotToggle = false;

  public ClimberSubsystem() { //initializes the climbers
    Constants.climber.setPosition(0.);
  }

  @Override
  public void periodic() {
    oldClimberAngle = climberAngle;
    climberAngle = Functions.DeltaAngleDeg(360.*climberEncoder.get()-90.,0);
    winchAngle = Math.toDegrees(Constants.climber.getPosition().getValueAsDouble());
    climberVelocity = (climberAngle-oldClimberAngle)/Robot.DeltaTime();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public static void pullInClimber(double speed) {
    speed = Functions.Clamp(speed, climberAngle>=Constants.maxClimberAngle?0:-1, climberAngle<=Constants.minClimberAngle?0:1);
    Constants.climber.set((Constants.climberInvert)?-speed:speed);
  }

  public static void rotateClimberTo(double angle) {
    angle = Functions.Clamp(angle, Constants.minClimberAngle, Constants.maxClimberAngle);
    pullInClimber(Functions.Clamp(Constants.climberPivotPMult*(Functions.DeltaAngleDeg(angle, climberAngle)), 
                                  -Constants.maxClimberPivotSpeed, Constants.maxClimberPivotSpeed));
    DriverDisplay.climberTarget.setDouble(angle);
  }

}