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

public class ClimberSubsystem extends SubsystemBase {
  public static RelativeEncoder climberEncoder = Constants.climberPivot.getAlternateEncoder();
  public static double climberAngle = 0;
  public static double lClimberAngle = 0;
  public static double rClimberAngle = 0;

  public ClimberSubsystem() { //initializes the climbers
    Constants.climber1.getEncoder().setPosition(0);
    Constants.climber2.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    if (!Constants.lClimberSwitch.get()) Constants.climber1.getEncoder().setPosition(0.0); //sets climber position to 0 if it's at the bottom
    if (!Constants.rClimberSwitch.get()) Constants.climber2.getEncoder().setPosition(0.0);
    climberAngle = climberEncoder.getPosition();
    lClimberAngle = Constants.climber1.getEncoder().getPosition();
    rClimberAngle = Constants.climber2.getEncoder().getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void moveClimbers(double in) { //moves the climbers, adding or subtracting half of difference to make the difference between the two climber powers equal to that number.
    Constants.climber1.set(Functions.Clamp(in*(Constants.climber1Invert?-1:1), 
    !Constants.lClimberSwitch.get()?0: Functions.Clamp(Constants.climberReductionMult*
    (lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    lClimberAngle>=Constants.climberMaxHeight?0:1));

    Constants.climber2.set(Functions.Clamp(in*(Constants.climber2Invert?-1:1), 
    !Constants.rClimberSwitch.get()?0:Functions.Clamp(Constants.climberReductionMult*
    (lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    rClimberAngle>=Constants.climberMaxHeight?0:1));
  }

  public static void moveClaw(double speed) {
    Constants.climberclaw.set(speed);
  }

  public static void moveClimber(double speed) {
    Constants.climber1.set(speed);
    Constants.climber2.set(speed);
  }

  public static void rotateClimberPivot(double speed) {
    Constants.climberPivot.set(Functions.Clamp(speed, 
                              -Functions.Clamp(0.2*(climberAngle-Constants.minClimberAngle), 0, 1), 
                              Functions.Clamp(-(0.2*(climberAngle-Constants.maxClimberAngle)), 0, 1)) 
                              + (Constants.climberPivotGravMult*Math.cos(Math.toRadians(climberAngle))));
  }

  public static void climberPivot(double angle) {
    angle = Functions.Clamp(angle, Constants.minClimberAngle, Constants.maxClimberAngle);
   rotateClimberPivot(Functions.Clamp((Constants.climberPivotPMult*(angle - Constants.climberPivot.get()))
   - (Constants.climberPivotDMult*climberEncoder.getVelocity()), - 
   Constants.maxClimberPivotSpeed, Constants.maxClimberPivotSpeed));
  }
}