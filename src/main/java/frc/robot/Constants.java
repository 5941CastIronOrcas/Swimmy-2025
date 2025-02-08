// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utilityObjects.Vector2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //CONTROLLER STUFF:
  public static final XboxController controller1 = new XboxController(0); //drivetrain controller
  public static final XboxController controller2 = new XboxController(1); //arm controller
  public static final double controllerDeadZone = 0.1; //how far the stick can go without triggering input

  //Misc
  //public static final DigitalInput limpRobotButton = new DigitalInput(0);
  public static final int defaultAutoSequence = 0;

  public static double timeSinceStartAtAutoStart = 0;
  //SWERVE STUFF:
  //Gyro
  public static final Pigeon2 gyro = new Pigeon2(54); 
  //Swerve Motor Declarations
  public static final SparkMax flaMotor = new SparkMax(27, MotorType.kBrushless); //front left angle motor
  public static final SparkMax fltMotor = new SparkMax(26, MotorType.kBrushless); //front left throttle motor
  public static final SparkMax fraMotor = new SparkMax(25, MotorType.kBrushless); //front right angle motor
  public static final SparkMax frtMotor = new SparkMax(24, MotorType.kBrushless); //front right throttle motor
  public static final SparkMax blaMotor = new SparkMax(21, MotorType.kBrushless); //back left angle motor
  public static final SparkMax bltMotor = new SparkMax(20, MotorType.kBrushless); //back left throttle motor
  public static final SparkMax braMotor = new SparkMax(23, MotorType.kBrushless); //back right angle motor
  public static final SparkMax brtMotor = new SparkMax(22, MotorType.kBrushless); //back right throttle motor
  //Swerve Motor Inversions
  public static final boolean fltInvert = true;
  public static final boolean frtInvert = false;
  public static final boolean bltInvert = false;
  public static final boolean brtInvert = false;
  //Swerve Encoder Declarations
  public static final CANcoder flEncoder = new CANcoder(50);
  public static final CANcoder frEncoder = new CANcoder(51);
  public static final CANcoder blEncoder = new CANcoder(52);
  public static final CANcoder brEncoder = new CANcoder(53);
  //Swerve Module Constants
  public static final double swerveDriveRatio = 1.00 / 6.75; //L2=1/6.75  L3=1/6.12
  public static final double swerveWheelCircumference = 0.096774 * Math.PI; // in m
  public static final double modulePMult = 0.01;
  public static final double maxThrottleChange = 2.0; //the maximum amount the wheel throttle of each module is allowed to change per frame (max 2.0)
  public static final double swerveMaxAccel = 2.0; //the max amount swerve is allowed to accelerate, measured in percent per frame (max 2.0)
  public static final double swerveMaxAccelExtended = 0.6; //the max amount swerve is allowed to accelerate when the arm is fully extended
  //Swerve Drive Turning Constants
  public static final double turnMult = 1.0; //the max speed Swerve is EVER allowed to turn at
  public static final double swerveAutoTurnPMult = 0.007;
  public static final double swerveAutoTurnDMult = 0.0005;
  public static final double swerveAutoTurnMaxSpeed = 1.0; //the max speed Swerve is allowed to turn at when turning itself
  public static final double swerveAutoTurnDeadZone = 2.0; //if swerve is pointing within this many degrees of where it wants to point, it stops rotating.
  public static final double swerveAngleVariation = 1.0; //swerve has to be within this many degrees of the right direction for the shooter to shoot
  //Swerve Drive To/Auto Constants
  public static final double pathPlannerPMult = 1.;
  public static final double pathPlannerAngularPMult = 1.;
  public static final double swerveDriveToPMult = 1.0;
  public static final double robotMaxVelocity = 7.0; //measured max velocity of robot when going straight with max input
  public static final double robotMaxRotationalVelocity = 8.0; //max rotational velocity of robot in radians
  public static final double swerveDriveToDMult = 0.14;
  public static final double swerveDriveToDeadZone = 0.03; //if the robot is within this many meters of the target position, it stops moving.
  public static final double swerveSquareDriveToDeadZone = 0.03;
  //Swerve Collect Ring Constants
  public static final double swerveCollectNotePMult = 0.5;
  
  
  //GAME PIECE MANIPULATION STUFF:

  //Elevator Motor Declarations
  public static final SparkMax elevator1 = new SparkMax(28, MotorType.kBrushless); //left arm motor viewing from the front of the robot
  public static final SparkMax elevator2 = new SparkMax( 29, MotorType.kBrushless); //right arm motor
  public static final Boolean elevator1Invert = true;
  public static final Boolean elevator2Invert = false;
  //Coral Intake Motor Declarations
  public static final SparkMax coralIntakePivot = new SparkMax(30, MotorType.kBrushless);
  public static final SparkMax coralIntake = new SparkMax(31, MotorType.kBrushless);
  public static final SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();
  //Algae Intake Motor Declarations
  public static final SparkMax algaeIntakePivot = new SparkMax(32, MotorType.kBrushless);
  public static final SparkMax algaeIntake = new SparkMax(33, MotorType.kBrushless);
  public static final SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
  
  //Arm Sensor Declarations
  public static DutyCycleEncoder coralEncoder = new DutyCycleEncoder(1);

  //ALL PIDS STILL NEED TO BE CALIBRATED
  //public static final DigitalInput[] noteDetectionSwitches = new DigitalInput[]{new DigitalInput(0)};
  //Elevator Control Constants
  public static final double elevatorPMult = 0.1;
  public static final double elevatorDMult = 0.0;
  public static final double elevatorGravMult = 0.02; //how much the elevator PID compensates for gravity
  public static final double maxElevatorSpeed = 1.0; //Max speed the elevator PID is allowed to output to the elevator motor
  public static final double elevatorVariation = 0.2; //how close the elevator has to be to the target height in inches to allow intake/deposit
  public static final double intakeHeight = 20.;
  public static final double reef1Height = 18.;
  public static final double reef2Height = 31.875;
  public static final double reef3Height = 47.625;
  public static final double reef4Height = 72.;
  //Coral Intake Control Constants
  public static final double coralPMult = 1.0;
  public static final double coralDMult = 1.0; 
  public static final double coralGravMult = 1.0;
  public static final double maxCoralPivotSpeed = 1.0;
  public static final double coralAngleVariation = 1.0;
  public static final double coralIntakeAngle = 0.0; //the angle in degrees the coral intake should be at to intake coral
  public static final double reef1Angle = 10.0;
  public static final double reef2Angle = 75.8;
  public static final double reef4Angle = 90.;
  public static final double coralIntakeSpeed = 1.;
  public static final double reef1Speed = 0.2;
  //Algae Intake Control Constants
  public static final double algaePMult = 1.;
  public static final double algaeDMult = 1.;
  public static final double algaeGravMult = 1.;
  public static final double maxAlgaePivotSpeed = 1.;
  public static final double algaeAngleVariation = 1.;
  public static final double algaeIntakeAngle = 0.;
  public static final double processorAngle = 0.;
  public static final double algaeIntakeSpeed = 1.;
  
  
  //CLIMBER STUFF
  //Climber Motor Declarations
  public static final SparkMax climberPivot = new SparkMax(34, MotorType.kBrushless);
  public static final SparkMax climber1 = new SparkMax(35, MotorType.kBrushless);
  public static final SparkMax climber2 = new SparkMax(36, MotorType.kBrushless);
  public static final SparkMax climberclaw = new SparkMax(37, MotorType.kBrushless);
  public static DigitalInput lClimberSwitch = new DigitalInput(2);
  public static DigitalInput rClimberSwitch = new DigitalInput(3);
  public static DigitalInput tesDigitalInputHUH = new DigitalInput(7);
  public static final Boolean climber1Invert = false;
  public static final Boolean climber2Invert = false;
  //Climber Sensor Declarations
  public static DutyCycleEncoder climberEncoder = new DutyCycleEncoder(2);
  //Climber Control Constants
  public static final double climberBalancePMult = 0.01;
  public static final double climberBalanceDMult = 0.01;
  public static final double climberMaxHitSpeed = 0.5;
  public static final double climberSmoothingStart = 20;
  public static final double climberSmoothingEnd = 10;
  public static final double climberReductionMult = (climberMaxHitSpeed-1) / (climberSmoothingStart-climberSmoothingEnd);
  //public static final double climberMaxSpeed = 1;
  public static final double climberMaxHeight = 95;
  public static final double climberGoToPMult = 0.2;

  //POSITION ESTIMATION AND FIELD CONSTANTS:  
  public static final Vector2D redSpeaker = new Vector2D(16.579342, 5.547868);
  public static final Vector2D blueSpeaker = new Vector2D(-0.0381, 5.547868 );
  public static final Vector2D redAmpDepositPosition = new Vector2D(14.700758, 8.24); //7.74
  public static final Vector2D blueAmpDepositPosition = new Vector2D(1.8415, 8.24);
  public static final double speakerHeight = 2.05; //height of speaker opening in meters
  public static final double speakerAngleVariation = 5.0; //how many degrees the arm angle can be from the target and still shoot
  public static final double noteCameraHeight = 0.22; //in meters for note detector
  public static final double noteCameraForwardOffset = 0.54; // forward distance from robot center to note detector camera in meters
  public static final double noteCameraAngle = -20; //for note detector
  public static final double swerveMaxSpeed = 4.4; //the max speed we're capable of moving at in m/s (used for discarding impossible data)
  public static final String apriltagCamera1Name = "camLeft"; //LEFT (shooter forward)
  public static final String apriltagCamera2Name = "camRight"; //RIGHT (shooter forward)
 // public static final String noteDetectionCameraName = "Arducam_OV9782_USB_Camera";
  public static final double FieldDisplayOffsetX = 1.1225;
  public static final double FieldDisplayOffsetY = 0.326;



  public static final Vector2D[] blueNotesPos = new Vector2D[] {new Vector2D(2.9464, 4.1057), new Vector2D(2.9464, 5.5535), new Vector2D(2.9464, 7.0013)}; 
  public static final Vector2D[] redNotesPos = new Vector2D[] {new Vector2D(13.6449, 4.1057), new Vector2D(13.6449, 5.5535), new Vector2D(13.6449, 7.0013)};
  public static final Vector2D[] centerNotesPos = new Vector2D[] {new Vector2D(8.2956, 0.7529), new Vector2D(8.2956, 2.4293), new Vector2D(8.2956, 4.1057), new Vector2D(8.2956, 5.7821), new Vector2D(8.2956, 7.4585)};
  public static Vector2D[] allNotesPos = new Vector2D[centerNotesPos.length + redNotesPos.length];
  
  //OTHER CONSTANTS:
  public static final double gravity = 9.81; //gravitational acceleration in m/s^2
  //ANSI Color Codes:
  public static final String ansiRESET = "\u001B[0m";
  public static final String ansiBLK = "\u001B[30m";
  public static final String ansiRED = "\u001B[31m";
  public static final String ansiGRN = "\u001B[32m";
  public static final String ansiYLW = "\u001B[33m";
  public static final String ansiBLU = "\u001B[34m";
  public static final String ansiPRP = "\u001B[35m";
  public static final String ansiCYN = "\u001B[36m";
  public static final String ansiWHT = "\u001B[37m";
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  
  }
}
