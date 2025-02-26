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
  public static double climberPivotAngle = 0;
  public static double oldClimberAngle = 0;
  public static double climberAngle = 0;
  public static double climberVelocity = 0;
  public static double clawAngle = 0;
  //public static double rClimberAngle = 0;

  public ClimberSubsystem() { //initializes the climbers
    //Constants.climber2.getEncoder().setPosition(0);
    Constants.climberClaw.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    climberPivotAngle = (360.*climberEncoder.get()-245.);
    oldClimberAngle = climberAngle;
    climberAngle = Math.toDegrees(Constants.climber.getPosition().getValueAsDouble());
    climberVelocity = (climberAngle-oldClimberAngle)/Robot.DeltaTime();
    clawAngle = Constants.climberClaw.getEncoder().getPosition();
    //rClimberAngle = Constants.climber2.getEncoder().getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /*public static void moveClimbers(double in) { //moves the climbers, adding or subtracting half of difference to make the difference between the two climber powers equal to that number.
    Constants.climber1.set(Functions.Clamp(in*(Constants.climber1Invert?-1:1), 
    !Constants.lClimberSwitch.get()?0: Functions.Clamp(Constants.climberReductionMult*
    (lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    lClimberAngle>=Constants.climberMaxHeight?0:1));

    Constants.climber2.set(Functions.Clamp(in*(Constants.climber2Invert?-1:1), 
    !Constants.rClimberSwitch.get()?0:Functions.Clamp(Constants.climberReductionMult*
    (lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    rClimberAngle>=Constants.climberMaxHeight?0:1));
  }*/

  public static void moveClaw(double speed) {
    speed = Functions.Clamp(speed, -Functions.Clamp(0.2*(clawAngle-Constants.minClawAngle), 0, 1), Functions.Clamp(-(0.2*(clawAngle-Constants.maxClawAngle)), 0, 1));
    Constants.climberClaw.set(Constants.climberClawInvert?-speed:speed);
  }

  public static void pullInClimber(double speed) {
    speed = Functions.Clamp(speed, -Functions.Clamp(0.2*(climberPivotAngle-Constants.minClimberAngle), 0, 1), Functions.Clamp(-(0.2*(climberPivotAngle-Constants.maxClimberAngle)), 0, 1));
    Constants.climber.set((Constants.climberInvert)?-speed:speed);
    //Constants.climber2.set((Constants.climber2Invert)?-speed:speed);
  }

  public static void rotateClimberPivot(double speed) {
    speed = Functions.Clamp(speed, -Functions.Clamp(0.2*(climberPivotAngle-Constants.minClimberAngle), 0, 1), Functions.Clamp(-(0.2*(climberPivotAngle-Constants.maxClimberAngle)), 0, 1)) - (Constants.climberPivotGravMult*Math.cos(Math.toRadians(climberAngle)));
    Constants.climberPivot.set(Constants.climberPivotInvert?-speed:speed);
  }

  public static void rotateClimber(double speed) {
    pullInClimber(speed);
    rotateClimberPivot(speed/8.);
  }

  public static void climberToAngle(double angle) {
    angle = Functions.Clamp(angle, Constants.minClimberAngle, Constants.maxClimberAngle);
   rotateClimberPivot(Functions.Clamp((Constants.climberPivotPMult*(angle - Constants.climberPivot.get()))
   - (Constants.climberPivotDMult*climberVelocity), - 
   Constants.maxClimberPivotSpeed, Constants.maxClimberPivotSpeed));
   DriverDisplay.climberTarget.setDouble(angle);
  }
}