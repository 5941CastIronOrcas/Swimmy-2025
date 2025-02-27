// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoSequences;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;



public class DriverDisplay extends SubsystemBase {
  
  //Auto Selectors and Whatnot
  public static double angleToAssign = 0;
  public static ShuffleboardTab AutoStuff = Shuffleboard.getTab("Autonomous");
  public static GenericEntry AutoSequence = AutoStuff.add("Select Your Auto Here", 0).getEntry();
  public static GenericEntry AutoSequenceDisplay = AutoStuff.add("You Have Selected:", "N/A").getEntry();
  public static GenericEntry rng = AutoStuff.add("rng", 0).getEntry();

  public static GenericEntry AutoSuccesfullShots = AutoStuff.add("Succesfull Shots: ", "N/A").getEntry();

  public static GenericEntry gyroOrientation = AutoStuff.add("Gyro Start Angle", 0).getEntry(); 
  public static GenericEntry gyroOrientationDisplay = AutoStuff.add("Gyro Angle Selected", "N/A").getEntry();

  // Auto Sequences selector
  public static GenericEntry coralIgnorancePreset = AutoStuff.add("coralPatternPreset", "N/A").getEntry();
  //public static GenericEntry coralIgnoranceHEX = AutoStuff.add("coralPatternHEX", "N/A").getEntry();
  public static GenericEntry coralIgnoranceCheck = AutoStuff.add("Selected Coral Ignorance [0 - ignore, 1 - include]", "N/A").getEntry();

  public static GenericEntry coralsIgnorance = AutoStuff.add("coralsToggleInput", "12345678").getEntry();

  //CoralDetector
  public static ShuffleboardTab coralDetector = Shuffleboard.getTab("CoralDetector");
  public static GenericEntry showCoral = coralDetector.add("Coral Visible",false).getEntry();
  public static GenericEntry showCoralPitch = coralDetector.add("Coral Pitch", 0).getEntry();
  public static GenericEntry showCoralYaw = coralDetector.add("Coral Yaw", 0).getEntry();
  public static GenericEntry coralDistance = coralDetector.add("Coral Distance", 0).getEntry();
 
  //Arm
  public static ShuffleboardTab arm = Shuffleboard.getTab("Arm");
  public static GenericEntry elevatorMotorAngle = arm.add("Elevator Height Motor Angle", 0).getEntry();
  public static GenericEntry elevatorHeight = arm.add("Elevator Height", 0).getEntry();
  public static GenericEntry correctElevatorHeight = arm.add("Elevator at Correct Height", false).getEntry();
  public static GenericEntry elevatorBottomSwitch = arm.add("Elevator Bottom Switch", false).getEntry();
  public static GenericEntry elevatorTopSwitch = arm.add("Elevator Top Switch", false).getEntry();
  public static GenericEntry intakeAngle = arm.add("Intake Angle", 0).getEntry();
  public static GenericEntry correctIntakeAngle = arm.add("Intake at Correct Angle", false).getEntry();
  public static GenericEntry arduinoRecall = arm.add("Arduino Recall", 0).getEntry();
  public static GenericEntry hasCoral = arm.add("Has Coral", false).getEntry();
  public static GenericEntry elevatorTarget = arm.add("Elevator Target", 0).getEntry();
  public static GenericEntry intakeTarget = arm.add("Intake Target", 0).getEntry();
  public static GenericEntry compensation = arm.add("Compensation", 0).getEntry();
  public static GenericEntry elevatorThrottle = arm.add("Elevator Throttle", 0).getEntry();
  public static GenericEntry motorPower1 = arm.add("Elevator1 Amps", 0).getEntry();
  public static GenericEntry motorPower2 = arm.add("Elevator2 Amps", 0).getEntry();
  public static GenericEntry intakePivotPower = arm.add("Intake Pivot Amps", 0).getEntry();
  public static GenericEntry intakePower = arm.add("Intake Amps", 0).getEntry();
  public static GenericEntry inRange = arm.add("In Range", false).getEntry();
  public static GenericEntry pivotVelocity = arm.add("Intake Pivot Velocity", 0.).getEntry();
  public static GenericEntry pivotThrottle = arm.add("Intake Pivot Throttle", 0.).getEntry();

  //Climber
  public static ShuffleboardTab climber = Shuffleboard.getTab("Climber");
  //public static GenericEntry climberR = climber.add("ClimberR Input", 0).getEntry();
  public static GenericEntry climberInput = climber.add("Climber Input", 0).getEntry();
  public static GenericEntry climberAngle = climber.add("Winch Motor Angle", 0).getEntry();
  //public static GenericEntry rClimberAngle = climber.add("RPosition", 0).getEntry(); 
  public static GenericEntry climberPivotAngle = climber.add("Climber Pivot Angle", 0).getEntry();
  public static GenericEntry climberTarget = climber.add("Climber Target Angle", 0).getEntry();
  public static GenericEntry robotRoll = climber.add("RobotRoll", 0).getEntry();
  public static GenericEntry robotPitch = climber.add("RobotPitch", 0).getEntry();


  //Swerve
  public static ShuffleboardTab swerve = Shuffleboard.getTab("Swerve");
  public static GenericEntry frAngle = swerve.add("Fr Angle", 0).getEntry();
  public static GenericEntry flAngle = swerve.add("Fl Angle", 0).getEntry();
  public static GenericEntry brAngle = swerve.add("Br Angle", 0).getEntry();
  public static GenericEntry blAngle = swerve.add("Bl Angle", 0).getEntry();
  public static GenericEntry frVelocity = swerve.add("Fr Velocity", 0).getEntry();
  public static GenericEntry flVelocity = swerve.add("Fl Velocity", 0).getEntry();
  public static GenericEntry brVelocity = swerve.add("Br Velocity", 0).getEntry();
  public static GenericEntry blVelocity = swerve.add("Bl Velocity", 0).getEntry();
  public static GenericEntry driveX = swerve.add("Drive X", 0).getEntry();
  public static GenericEntry driveY = swerve.add("Drive Y", 0).getEntry();
  public static GenericEntry driveRotate = swerve.add("Drive Spin", 0).getEntry();
  public static GenericEntry totalDriveAmps = swerve.add("Total Drive Amps", 0).getEntry();
  public static GenericEntry xVelocity = swerve.add("Robot X Velocity", 0).getEntry();
  public static GenericEntry yVelocity = swerve.add("Robot Y Velocity", 0).getEntry();
  public static GenericEntry rotVelocity = swerve.add("Robot Rotational Velocity", 0).getEntry();
  public static GenericEntry atTargetPosition = swerve.add("At Target Position", false).getEntry();
  public static GenericEntry atTargetAngle = swerve.add("At Target Angle", false).getEntry();
  public static GenericEntry accelX = swerve.add("Acceleration X", 0).getEntry();
  public static GenericEntry accelY = swerve.add("Acceleration Y", 0).getEntry();
  





  //GOA
  public static ShuffleboardTab goa = Shuffleboard.getTab("GOA");
  public static GenericEntry avoidanceX = goa.add("AvoidanceX", 0).getEntry();
  public static GenericEntry avoidanceY = goa.add("AvoidanceY", 0).getEntry();
  

  //position estimator
  public static ShuffleboardTab position = Shuffleboard.getTab("Position Estimator");
    public static GenericEntry isPresent1 = position.add("Is Present 1", false).getEntry();
    public static GenericEntry isPresent2 = position.add("Is Present 2", false).getEntry();
    public static GenericEntry ambiguity1 = position.add("Ambiguity 1", 0).getEntry();
    public static GenericEntry ambiguity2 = position.add("Ambiguity 2", 0).getEntry();
    public static GenericEntry speed = position.add("speed", 0).getEntry();
    public static GenericEntry latency = position.add("Latency", 0).getEntry();
    public static GenericEntry robotX = position.add("Robot X", 0).getEntry();
    public static GenericEntry robotY = position.add("Robot Y", 0).getEntry();
    public static GenericEntry driverYaw = position.add("DriverYaw", 0).getEntry();
    public static GenericEntry fieldYaw = position.add("FieldYaw", 0).getEntry();
    private final Field2d m_field = new Field2d();
    


  public DriverDisplay() {
  SmartDashboard.putData("Field", m_field); 

  }

  @Override
  public void periodic() {
    //if(ArmSubsystem.hasCoral) ControllerRumble.RumbleBothControllersBothSides(0.5);
    //else ControllerRumble.RumbleBothControllersBothSides(0);
    
    AutoSuccesfullShots.setInteger((int)AutoSequences.succesfulShots);
    switch ((int)DriverDisplay.gyroOrientation.getInteger(0)) {
      case 0:
        angleToAssign = 180;
        DriverDisplay.gyroOrientationDisplay.setString("0 Angle Selected (Intake Forward)");
        break;

      case 1:
        angleToAssign = 120;
        DriverDisplay.gyroOrientationDisplay.setString("Right Subwoofer Side Selected");
        break;

      case 2:
        angleToAssign = -120;
        DriverDisplay.gyroOrientationDisplay.setString("Left Subwoofer Side Selected");
        break;

      case 3:
        angleToAssign = -90;
        DriverDisplay.gyroOrientationDisplay.setString("Intake Face Left 90 Degree");
        break;

      case 4:
        angleToAssign = 90;
        DriverDisplay.gyroOrientationDisplay.setString("Intake Face Right 90 Degree");
        break;


      default:
      angleToAssign = 180;
      DriverDisplay.AutoSequenceDisplay.setString("INVALID VALUE");
        break;
    }

    //Auto
    String selectedAutoName = "ERROR IN DISPLAYING";
    switch (Robot.selectedAutoSequence) {
      case 0:
          selectedAutoName = "NO AUTO";
          break;

      case 1:
        selectedAutoName = "Retract Climbers";
        break;

      case 2:
        selectedAutoName = "Retract Climbers, Drive Out";      
        break;

      case 3:
        selectedAutoName = "Retract Climbers, Shoot Basic";      
        break;

      case 4:
        selectedAutoName = "Retract Climbers, Shoot Aimbot";      
        break;

      case 5:
        selectedAutoName = "Retract Climbers, Shoot Basic, Drive Out";
        break;

      case 6:
        selectedAutoName = "Drive Out, Retract Climbers, Shoot Aimbot";
        break;
      
      case 7:
        selectedAutoName = "Retract Climbers, Shoot Basic, Collect Nearest, Shoot";
        break;
      
      case 8:
        selectedAutoName = "Retract Climbers, Shoot Aimbot, Collect Nearest, Shoot";
        break;
      
      case 9:
        selectedAutoName = "Retract Climbers, Shoot Basic, Collect and Shoot as Many as Possible";
        break;
      
      case 10:
        selectedAutoName = "Retract Climbers, Shoot Aimbot, Collect and Shoot as Many as Possible";
        break;

      default:
        selectedAutoName = "THE VALUE IS INVALID";
        break;
    }



        
    // coral choosing
    String coralIgnoranceInpt = DriverDisplay.coralsIgnorance.getString("12345678");

    if (coralIgnoranceInpt.length() >= 8) coralIgnoranceInpt = coralIgnoranceInpt.substring(0, 8);
    if (coralIgnoranceInpt.length() < 1) coralIgnoranceInpt = "12345678";
    coralIgnoranceInpt = Functions.RemoveMultiples(coralIgnoranceInpt);
    //for (int i = 0; i < 8; i++) coralIgnoranceInpt = coralIgnoranceInpt.substring(0, i) + (coralIgnoranceInpt.charAt(i) == '0' ? '0' : '1') + coralIgnoranceInpt.substring(i, coralIgnoranceInpt.length() - 1);
      //for (int i = 0; i < coralIgnoranceInpt.length(); i++) AutoSequences.coralsIncluded[i] = coralIgnoranceInpt.charAt(i) == '1' ? true : false;
    AutoSequences.coralList = new int[coralIgnoranceInpt.length()];
    
    for (int i = 0; i < coralIgnoranceInpt.length(); i++) {
      AutoSequences.coralList[i] = Character.getNumericValue(coralIgnoranceInpt.charAt(i));
    }
    String outputString = "";
    for (int i = 0; i < AutoSequences.coralList.length; i++) outputString += AutoSequences.coralList[i] + ", "; 
    DriverDisplay.coralIgnoranceCheck.setString("Enabled Corals" + outputString);

    DriverDisplay.AutoSequenceDisplay.setString(selectedAutoName);
    DriverDisplay.rng.setDouble(Math.random());

    //Arm
    DriverDisplay.elevatorMotorAngle.setDouble(ArmSubsystem.newElevatorAngle);
    DriverDisplay.elevatorHeight.setDouble(ArmSubsystem.elevatorHeight);
    DriverDisplay.correctElevatorHeight.setBoolean(ArmSubsystem.correctHeight);
    DriverDisplay.elevatorBottomSwitch.setBoolean(ArmSubsystem.elevatorBottom);
    DriverDisplay.elevatorTopSwitch.setBoolean(ArmSubsystem.elevatorTop);
    DriverDisplay.intakeAngle.setDouble(ArmSubsystem.coralAngle);
    DriverDisplay.correctIntakeAngle.setBoolean(ArmSubsystem.correctAngle);
    DriverDisplay.compensation.setDouble(ArmSubsystem.coralCompensation);
    DriverDisplay.arduinoRecall.setDouble(1.234);
    DriverDisplay.hasCoral.setBoolean(ArmSubsystem.hasCoral);
    DriverDisplay.motorPower1.setDouble(Constants.elevator1.getOutputCurrent());
    DriverDisplay.motorPower2.setDouble(Constants.elevator2.getOutputCurrent());
    DriverDisplay.intakePivotPower.setDouble(Constants.coralIntakePivot.getOutputCurrent());
    DriverDisplay.intakePower.setDouble(Constants.coralIntake.getOutputCurrent());
    DriverDisplay.inRange.setBoolean(ArmSubsystem.inRange);
    DriverDisplay.pivotVelocity.setDouble(ArmSubsystem.coralEncoder.getVelocity()*100.);
    DriverDisplay.pivotThrottle.setDouble(Constants.coralIntakePivot.get());


    //Climber
    DriverDisplay.climberInput.setDouble(Constants.climber.get());
    //DriverDisplay.climberR.setDouble(Constants.climber2.get());
    DriverDisplay.climberAngle.setDouble(ClimberSubsystem.winchAngle);
    //DriverDisplay.rClimberAngle.setDouble(ClimberSubsystem.rClimberAngle);
    DriverDisplay.climberPivotAngle.setDouble(ClimberSubsystem.climberPivotAngle);
    DriverDisplay.robotRoll.setDouble(Constants.gyro.getRoll().getValueAsDouble());
    DriverDisplay.robotPitch.setDouble(Constants.gyro.getPitch().getValueAsDouble());
    


    //swerve
    DriverDisplay.frAngle.setDouble(SwerveSubsystem.frModule.GetAngle());
    DriverDisplay.flAngle.setDouble(SwerveSubsystem.flModule.GetAngle());
    DriverDisplay.brAngle.setDouble(SwerveSubsystem.brModule.GetAngle());
    DriverDisplay.blAngle.setDouble(SwerveSubsystem.blModule.GetAngle());
    DriverDisplay.frVelocity.setDouble(SwerveSubsystem.frModule.GetVelocity());
    DriverDisplay.flVelocity.setDouble(SwerveSubsystem.flModule.GetVelocity());
    DriverDisplay.brVelocity.setDouble(SwerveSubsystem.brModule.GetVelocity());
    DriverDisplay.blVelocity.setDouble(SwerveSubsystem.blModule.GetVelocity());
    DriverDisplay.totalDriveAmps.setDouble(Constants.fraMotor.getOutputCurrent() + Constants.flaMotor.getOutputCurrent() + Constants.braMotor.getOutputCurrent() + Constants.blaMotor.getOutputCurrent());
    DriverDisplay.xVelocity.setDouble(SwerveSubsystem.currentSpeed.vxMetersPerSecond);
    DriverDisplay.yVelocity.setDouble(SwerveSubsystem.currentSpeed.vyMetersPerSecond);;
    DriverDisplay.rotVelocity.setDouble(SwerveSubsystem.currentSpeed.omegaRadiansPerSecond);;
    DriverDisplay.atTargetPosition.setBoolean(SwerveSubsystem.atTargetPosition);
    DriverDisplay.atTargetAngle.setBoolean(SwerveSubsystem.atTargetAngle);
    DriverDisplay.accelX.setDouble(SwerveSubsystem.accelX);
    DriverDisplay.accelY.setDouble(SwerveSubsystem.accelY);
    

   
    

    //Coral Detector
    DriverDisplay.showCoralYaw.setDouble(GamePieceDetector.coralYaw);
    DriverDisplay.showCoralPitch.setDouble(GamePieceDetector.coralPitch);
    DriverDisplay.showCoral.setBoolean(GamePieceDetector.coralVisible);
    DriverDisplay.coralDistance.setDouble(GamePieceDetector.coralDist);


    
    //position estimator
    DriverDisplay.isPresent1.setBoolean(PositionEstimator.camCheck1());
    DriverDisplay.isPresent2.setBoolean(PositionEstimator.camCheck2());
    DriverDisplay.ambiguity1.setDouble(PositionEstimator.ambiguity1);
    DriverDisplay.ambiguity2.setDouble(PositionEstimator.ambiguity2);
    DriverDisplay.latency.setDouble(PositionEstimator.latency1);
    DriverDisplay.speed.setDouble(Functions.Pythagorean(PositionEstimator.velocity.x, PositionEstimator.velocity.y));
    DriverDisplay.robotX.setDouble(PositionEstimator.robotPosition.getX());
    DriverDisplay.robotY.setDouble(PositionEstimator.robotPosition.getY());
    DriverDisplay.driverYaw.setDouble(PositionEstimator.robotYawDriverRelative);
    DriverDisplay.fieldYaw.setDouble( Functions.DeltaAngleDeg(0, PositionEstimator.robotPosition.getRotation().getDegrees()));
    Pose2d positionPose2d = new Pose2d(PositionEstimator.robotPosition.getX()+Constants.FieldDisplayOffsetX, PositionEstimator.robotPosition.getY()+Constants.FieldDisplayOffsetY, new Rotation2d(-PositionEstimator.robotPosition.getRotation().getRadians() - 0.5*Math.PI));
    m_field.setRobotPose(positionPose2d);
    

   
    
  

  }
}