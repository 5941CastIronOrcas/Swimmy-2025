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
  public static GenericEntry armAngle = arm.add("Arm Angle", 0).getEntry();
  public static GenericEntry correctArmAngle = arm.add("Arm at correct angle", false).getEntry();
  public static GenericEntry elevatorBottomSwitch = arm.add("Elevator Bottom Switch", false).getEntry();
  public static GenericEntry elevatorTopSwitch = arm.add("Elevator Top Switch", false).getEntry();
  public static GenericEntry arduinoRecall = arm.add("Arduino Recall", 0).getEntry();
  public static GenericEntry armHasCoral = arm.add("Has Coral", false).getEntry();
  public static GenericEntry armTarget = arm.add("Arm Target", 0).getEntry();
  public static GenericEntry armThrottle = arm.add("Arm Throttle", 0).getEntry();
  public static GenericEntry shooterFast = arm.add("Shooter fast", false).getEntry();
  public static GenericEntry motorPower1 = arm.add("ArmMotor1 Amps", 0).getEntry();
  public static GenericEntry motorPower2 = arm.add("ArmMotor2 Amps", 0).getEntry();
  public static GenericEntry motor1Rpm = arm.add("LowShooterRpm", 0).getEntry();
  public static GenericEntry motor2Rpm = arm.add("UpShooterRpm", 0).getEntry();
  public static GenericEntry range = arm.add("Range", 0).getEntry();
  public static GenericEntry ShootTotalPower = arm.add("Shooter Amps", 0).getEntry();
  public static GenericEntry IntakeTotalPower = arm.add("Intake Amps", 0).getEntry();
  public static GenericEntry atSpeakerAngle = arm.add("Facing speaker", false).getEntry();
  public static GenericEntry inRange = arm.add("In Range", false).getEntry();


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
  public static GenericEntry driveX = swerve.add("driveX", 0).getEntry();
  public static GenericEntry driveY = swerve.add("driveY", 0).getEntry();
  public static GenericEntry driveRotate = swerve.add("driveSpin", 0).getEntry();
  public static GenericEntry totalDriveAmps = swerve.add("Total Amps", 0).getEntry();
  public static GenericEntry atTargetPosition = swerve.add("atTargetPos", false).getEntry();
  public static GenericEntry atTargetAngle = swerve.add("atTargetAngle", false).getEntry();
  





  //Climber
  public static ShuffleboardTab climber = Shuffleboard.getTab("Climber");
  public static GenericEntry lClimberAngle = climber.add("LPosition", 0).getEntry();
  public static GenericEntry rClimberAngle = climber.add("RPosition", 0).getEntry(); 
  public static GenericEntry robotRoll = climber.add("RobotRoll", 0).getEntry();
  public static GenericEntry climberR = climber.add("ClimberR Input", 0).getEntry();
  public static GenericEntry climberL = climber.add("ClimberL Input", 0).getEntry();




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
    DriverDisplay.armAngle.setDouble(0.);
    DriverDisplay.arduinoRecall.setDouble(1.0);
    DriverDisplay.elevatorBottomSwitch.setBoolean(Constants.elevatorBottom.get());
    DriverDisplay.elevatorTopSwitch.setBoolean(Constants.elevatorTop.get());
    DriverDisplay.armHasCoral.setBoolean(ArmSubsystem.hasCoral);
    DriverDisplay.motorPower1.setDouble(Constants.elevator1.getOutputCurrent());
    DriverDisplay.motorPower2.setDouble(Constants.elevator2.getOutputCurrent());
    DriverDisplay.IntakeTotalPower.setDouble(Constants.coralIntake.getOutputCurrent());
    DriverDisplay.correctArmAngle.setBoolean(ArmSubsystem.correctArmAngle);
    DriverDisplay.shooterFast.setBoolean(ArmSubsystem.shooterFast);
    DriverDisplay.inRange.setBoolean(ArmSubsystem.inRange);



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
    DriverDisplay.atTargetPosition.setBoolean(SwerveSubsystem.atTargetPosition);
    DriverDisplay.atTargetAngle.setBoolean(SwerveSubsystem.atTargetAngle);
    

   
    

    //Coral Detector
    DriverDisplay.showCoralYaw.setDouble(GamePieceDetector.coralYaw);
    DriverDisplay.showCoralPitch.setDouble(GamePieceDetector.coralPitch);
    DriverDisplay.showCoral.setBoolean(GamePieceDetector.coralVisible);
    DriverDisplay.coralDistance.setDouble(GamePieceDetector.coralDist);


    //climber
    DriverDisplay.lClimberAngle.setDouble(ClimberSubsystem.lClimberAngle);
    DriverDisplay.rClimberAngle.setDouble(ClimberSubsystem.rClimberAngle);
    DriverDisplay.robotRoll.setDouble(Constants.gyro.getRoll().getValueAsDouble());
    DriverDisplay.climberR.setDouble(Constants.climber2.get());
    DriverDisplay.climberL.setDouble(Constants.climber1.get());
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