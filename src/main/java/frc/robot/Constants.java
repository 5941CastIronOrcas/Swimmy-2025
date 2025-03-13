// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.ArmSubsystem;
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
  public static final double warningTemp = 70.;

  public static double timeSinceStartAtAutoStart = 0;
  public static final double gravity = 9.81; //gravitational acceleration in m/s^2
  //SWERVE STUFF:
  //Gyro
  public static final Pigeon2 gyro = new Pigeon2(44);
  //Swerve Motor Declarations
  public static final SparkMax flaMotor = new SparkMax(20, MotorType.kBrushless); //front left angle motor
  public static final SparkMax fltMotor = new SparkMax(21, MotorType.kBrushless); //front left throttle motor
  public static final SparkMax fraMotor = new SparkMax(22, MotorType.kBrushless); //front right angle motor
  public static final SparkMax frtMotor = new SparkMax(23, MotorType.kBrushless); //front right throttle motor
  public static final SparkMax blaMotor = new SparkMax(26, MotorType.kBrushless); //back left angle motor
  public static final SparkMax bltMotor = new SparkMax(27, MotorType.kBrushless); //back left throttle motor
  public static final SparkMax braMotor = new SparkMax(24, MotorType.kBrushless); //back right angle motor
  public static final SparkMax brtMotor = new SparkMax(25, MotorType.kBrushless); //back right throttle motor
  //Swerve Motor Inversions
  public static final boolean fltInvert = true;
  public static final boolean frtInvert = false;
  public static final boolean bltInvert = true;
  public static final boolean brtInvert = false;
  //Swerve Encoder Declarations
  public static final CANcoder flEncoder = new CANcoder(40);
  public static final CANcoder frEncoder = new CANcoder(41);
  public static final CANcoder blEncoder = new CANcoder(42);
  public static final CANcoder brEncoder = new CANcoder(43);
  //Swerve Module Constants
  public static final double swerveDriveRatio = 1.00 / 8.14; //L1=1/8.14  L2=1/6.75  L3=1/6.12
  public static final double swerveWheelCircumference = 0.096774 * Math.PI; // in m
  public static final double modulePMult = 0.01;
  public static final double maxThrottleChange = 2.0; //the maximum amount the wheel throttle of each module is allowed to change per frame (max 2.0)
  public static final double swerveMaxAccel = 2.0; //the max amount swerve is allowed to accelerate, measured in percent per frame (max 2.0)
  public static final double swerveMaxAccelExtendedX = 0.001; //the max amount swerve is allowed to accelerate when the arm is fully extended
  public static final double swerveMaxAccelExtendedY = 0.01;
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
  public static final double swerveAngledDriveToDeadZone = 0.3;
  public static final double swerveAngledDriveToRadius = 2.;
  public static final double[] snapAngles = new double[]{0.,60.,120.,180.,240.,300.,360.,234.,126.};
  //Swerve Collect Ring Constants
  public static final double swerveCollectCoralPMult = 0.5;


  //GAME PIECE MANIPULATION STUFF:

  //Elevator Motor Declarations
  public static final SparkMax elevator1 = new SparkMax(28, MotorType.kBrushless); //left arm motor viewing from the front of the robot
  public static final SparkMax elevator2 = new SparkMax( 29, MotorType.kBrushless); //right arm motor
  public static final Boolean elevator1Invert = false;
  public static final Boolean elevator2Invert = true;
  //Coral Intake Motor Declarations
  public static final SparkMax coralIntakePivot = new SparkMax(30, MotorType.kBrushless);
  public static final SparkMax coralIntake = new SparkMax(31, MotorType.kBrushless);
  public static final SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();
  public static final Boolean coralIntakePivotInvert = false;
  public static final Boolean coralIntakeInvert = false;
  //Algae Intake Motor Declarations
  public static final SparkMax algaeIntake = new SparkMax(32, MotorType.kBrushless);
  public static final SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();

  //Arm Sensor Declarations
  public static DigitalInput elevatorBottom = new DigitalInput(3);
  public static DigitalInput elevatorTop = new DigitalInput(4);
  public static DigitalInput linebreakSensor = new DigitalInput(9);


  //public static final DigitalInput[] coralDetectionSwitches = new DigitalInput[]{new DigitalInput(0)};
  //Elevator Control Constants
  public static final double elevatorAngleOffsetThreshold = 330;
  public static final double elevatorPMult = 0.075;
  public static final double elevatorDMult = 0.0;
  public static final double elevatorGravMult = 0.02; //how much the elevator PID compensates for gravity
  public static final double maxElevatorSpeed = 1.; //Max speed the elevator PID is allowed to output to the elevator motor
  public static final double elevatorVariation = 0.2; //how close the elevator has to be to the target height in inches to allow intake/deposit
  public static final double angleToHeightRatio = 42.75/5085.;
  //public static final double intakeHeightFromGround = 17;
  public static final double intakeHeight = 10.;//36.5-intakeHeightFromGround;
  public static final double reef1Height = 0.;
  public static final double reef2Height = 16;
  public static final double reef3Height = 27.5;
  public static final double reef4Height = 47; //72.
  public static final double maxElevatorHeight = 47;
  public static final double maxElevatorAngle = ArmSubsystem.heightToAngle(maxElevatorHeight);
  public static final double elevatorAccelLimit = 0.05;
  //Coral Intake Control Constants
  public static final double intakeAccelLimit = 0.05;
  public static final double coralPMult = 0.015;
  public static final double coralDMult = 0.;
  public static final double coralIMult = 0.;
  public static final double coralIClamp = 0.;
  public static final double coralGravMult = 0.015;
  public static final double coralGravOffset = 0;
  public static final double withCoralPMult = 0.015;
  public static final double withCoralDMult = 0.;
  public static final double withCoralGravMult = 0.03;
  public static final double withCoralGravOffset = 0;
  public static final double maxCoralPivotSpeed = 1.0;
  //public static final double coralAngleVariation = 1.0;
  public static final double coralIntakeAngle = 0.0; //the angle in degrees the coral intake should be at to intake coral
  public static final double reef1Angle = 0.0;
  public static final double reef2Angle = 65;
  public static final double reef4Angle = 70.;
  public static final double coralIntakeSpeed = 1.;
  public static final double reef1Speed = 0.2;
  public static final double minCoralAngle = -5;
  public static final double maxCoralAngle = 95;
  public static final double intakeMass = 1.81;
  public static final double intakeCenterRadius = 0.157445929589;
  public static final double intakeCenterAngle = 20.3440038775;
  public static final double springAngle = 0.384442501908;
  public static final double springRadius = Math.toDegrees(0.113305058401);
  public static final Vector2D springEndPos = new Vector2D(0.0594868,0.1476248);
  public static final double springEndAngle = 0.383054972827;
  public static final double springEndDist = 0.15915954558;
  public static final double gForceTimesRadius = intakeMass*gravity*intakeCenterRadius;
  public static final double sForceTimesRadius = 14.6346 * 2. * springRadius;
  public static final double compensationMinDeltaAngle = 0.1;
  public static final double coralMaxAdaptiveAngle = 45;
  public static final double coralMinAdaptiveAngle = 25;
  //Algae Intake Control Constants
  public static final double algaePMult = 0.005;
  public static final double algaeDMult = 0.;
  public static final double algaeGravMult = 0.;
  public static final double maxAlgaePivotSpeed = 1.;
  public static final double algaeAngleVariation = 1.;
  public static final double algaeIntakeAngle = 0.;
  public static final double processorAngle = 0.;
  public static final double algaeIntakeSpeed = 1.;
  public static final double minAlgaeAngle = 0.;
  public static final double maxAlgaeAngle = 30.;


  //CLIMBER STUFF
  //Climber Motor Declarations
  //public static final SparkMax climberPivot = new SparkMax(34, MotorType.kBrushless);
  public static final TalonFX climber = new TalonFX(45);
  //public static final SparkMax climber2 = new SparkMax(36, MotorType.kBrushless);
  //public static final SparkMax climberClaw = new SparkMax(36, MotorType.kBrushless);
  public static final Boolean climberInvert = true;
  //public static final Boolean climber2Invert = false;
  public static final Boolean climberPivotInvert = false;
  public static final Boolean climberClawInvert = false;
  public static DutyCycleEncoder climberEncoder = new DutyCycleEncoder(5);
  //public static DigitalInput tesDigitalInputHUH = new DigitalInput(7);
  //Climber Control Constants
  public static final double climberPivotPMult = 0.01;
  public static final double climberPivotDMult = 0.01;
  public static final double climberPivotGravMult = 0.1;
  public static final double maxClimberPivotSpeed = 0.;
  public static final double climberMaxHitSpeed = 0.5;
  public static final double climberSmoothingStart = 20;
  public static final double climberSmoothingEnd = 10;
  public static final double climberReductionMult = (climberMaxHitSpeed-1) / (climberSmoothingStart-climberSmoothingEnd);
  //public static final double climberMaxSpeed = 1;
  public static final double climberMaxHeight = 95;
  public static final double climberGoToPMult = 0.2;
  public static final double minClimberAngle = 0;
  public static final double maxClimberAngle = 215;
  public static final double minClawAngle = -3600.;
  public static final double maxClawAngle = 3600.;


  //POSITION ESTIMATION AND FIELD CONSTANTS:
  public static final Vector2D redSpeaker = new Vector2D(16.579342, 5.547868);
  public static final Vector2D blueSpeaker = new Vector2D(-0.0381, 5.547868 );
  public static final Vector2D redAmpDepositPosition = new Vector2D(14.700758, 8.24); //7.74
  public static final Vector2D blueAmpDepositPosition = new Vector2D(1.8415, 8.24);
  public static final double speakerHeight = 2.05; //height of speaker opening in meters
  public static final double speakerAngleVariation = 5.0; //how many degrees the arm angle can be from the target and still shoot
  public static final double coralCameraHeight = 0.22; //in meters for coral detector
  public static final double coralCameraForwardOffset = 0.54; // forward distance from robot center to coral detector camera in meters
  public static final double coralCameraAngle = -20; //for coral detector
  public static final double swerveMaxSpeed = 4.4; //the max speed we're capable of moving at in m/s (used for discarding impossible data)
  public static final AprilTagFieldLayout aprilTagFieldLayout =  AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
 // public static final String coralDetectionCameraName = "Arducam_OV9782_USB_Camera";
  public static final double FieldDisplayOffsetX = 0; //1.1225;
  public static final double FieldDisplayOffsetY = 0; //0.326;



  public static final Pose2d[] blueReefApriltags = new Pose2d[] {aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(22).get().toPose2d()};

  public static final Pose2d[] redReefApriltags = new Pose2d[] {aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(11).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
                                                                aprilTagFieldLayout.getTagPose(9).get().toPose2d()};
  public static final Pose2d[] blueCoralStationsApriltags = new Pose2d[] {aprilTagFieldLayout.getTagPose(12).get().toPose2d(),
                                                                          aprilTagFieldLayout.getTagPose(13).get().toPose2d()};
  public static final Pose2d[] redCoralStationsApriltags = new Pose2d[] {aprilTagFieldLayout.getTagPose(1).get().toPose2d(),
                                                                          aprilTagFieldLayout.getTagPose(2).get().toPose2d()};
  public static final double reefSideOffset = 0.1651;
  public static final double leftReefSideOffset = 0.005;
  public static final double rightReefSideOffset = 0.005;
  public static final double[] reefDist = new double[]{0.4,0.4,0.4,0.4};
  public static final double coralStationSideOffset = 0.54;
  public static final double coralStationDist = 0.38;


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
