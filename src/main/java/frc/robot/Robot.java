// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.net.http.HttpClient;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arduino;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private static double oldTime = 0.;
  private static double newTime = 0.;
  private static double deltaTime = 0.;
  //public static PathPlannerAuto auto;
  public static Boolean isRedAlliance = true;
  public static Boolean isBlueAlliance = false;
  public static boolean robotLimp = true;
  public static short framesCoralNotPresent = 0;
  public static int selectedAutoSequence = Constants.defaultAutoSequence;
  public static double timeSinceRPSstart = 0;
  public static int RPS = -1; //variable that chooses which thing in RPS
  //public static boolean limpButtonOld = Constants.limpRobotButton.get();

  private RobotContainer m_robotContainer;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our

    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //auto = new PathPlannerAuto("Auto 1");
    Arduino.ArduinoConnect(); //sets all climber encoders and the robot's gyro to zero.
    //Constants.climber2.getEncoder().setPosition(0);
    Constants.gyro.setYaw(180);
    
    SwerveSubsystem.oldVelocityX=0;
    SwerveSubsystem.oldVelocityY=0;
    for (int i = 0; i < Constants.redCoralsPos.length; i++) Constants.allCoralsPos[i] = Robot.isRedAlliance ? Constants.redCoralsPos[i] : Constants.blueCoralsPos[i]; //gets the values in either redCoralsPos or blueCoralsPos depending on the current team, and adds them to the start of the allCoralsPos array.
    for (int i = 0; i < Constants.centerCoralsPos.length; i++) Constants.allCoralsPos[i + Constants.redCoralsPos.length] = Constants.centerCoralsPos[i]; //adds the center corals to the end of the allCoralsPos array.
     
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    isRedAlliance = DriverStation.getAlliance().toString().equals("Optional[Red]");
    isBlueAlliance = DriverStation.getAlliance().toString().equals("Optional[Blue]");
    selectedAutoSequence = (int)DriverDisplay.AutoSequence.getInteger(Constants.defaultAutoSequence);
    oldTime = newTime;
    newTime = Timer.getFPGATimestamp();
    deltaTime = newTime-oldTime;
  }

  public static double DeltaTime() {
    return deltaTime;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  //@Override
  /*public void disabledPeriodic() {
    if(!limpButtonOld && Constants.limpRobotButton.get())
    {
      robotLimp = !robotLimp;
      Functions.setRobotLimp(robotLimp);
    }
    limpButtonOld = Constants.limpRobotButton.get();
  }*/

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      robotLimp = false;
      Functions.setRobotLimp(robotLimp);
    }
    ArmSubsystem.coralEncoder.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    double speed = 1-(0.75*Constants.controller1.getLeftTriggerAxis());
    double LSX = Functions.Exponential(Functions.DeadZone(Constants.controller1.getLeftX(), Constants.controllerDeadZone)) * speed; //gets each controller's inputs
    double LSY = -Functions.Exponential(Functions.DeadZone(Constants.controller1.getLeftY(), Constants.controllerDeadZone)) * speed;
    double RSX = Functions.Exponential(Functions.DeadZone(Constants.controller1.getRightX(), Constants.controllerDeadZone)) * speed;
    double RSY = -Functions.Exponential(Functions.DeadZone(Constants.controller1.getRightY(), Constants.controllerDeadZone)) * speed;
    double LSY2 = -Functions.Exponential(Functions.DeadZone(Constants.controller2.getLeftY(), Constants.controllerDeadZone));
    double RSY2 = -Functions.Exponential(Functions.DeadZone(Constants.controller2.getRightY(), Constants.controllerDeadZone));
    double RSX2 = Functions.Exponential(Functions.DeadZone(Constants.controller2.getRightX(), Constants.controllerDeadZone));
    double RSAngle = 90-Math.toDegrees(Math.atan2(RSY, RSX)); //gets the angle the right stick on controller 1 is pointing to.
    /* 
    Constants.flaMotor.set(1);
    Constants.fltMotor.set(1);
    Constants.frtMotor.set(1);
    Constants.fraMotor.set(1);
    Constants.blaMotor.set(1);
    Constants.bltMotor.set(1);
    Constants.brtMotor.set(1);
    Constants.braMotor.set(1);
     */
    if(Constants.controller1.getLeftBumper()) //snaps to specific directions to climb and score amp
    {
      RSAngle = Math.abs(Functions.DeltaAngleDeg(0, RSAngle)) < 105.0 ? 90 * Math.round(RSAngle / 90.0) : 120.0 * Math.round(RSAngle / 120.0);
    }

    if (Constants.controller1.getRightBumperPressed()) { //resets the robot's yaw
      Constants.gyro.setYaw(180);
    }
    //SwerveSubsystem.Drive(LSX, LSY, RSX);
    //SwerveSubsystem.DriveDriverOriented(LSX, LSY, RSX);
    //SwerveSubsystem.DriveFieldOriented(LSX, LSY, RSX);

    //Swerve
    if(Constants.controller1.getBButton())
    {
      SwerveSubsystem.CollectCoral(LSX, LSY, speed);
    }
    else if(Constants.controller1.getXButton())
    {
      SwerveSubsystem.FaceSpeaker(LSX, LSY, speed);
    }
    else if(Constants.controller1.getYButton())
    {
      SwerveSubsystem.GoToAmp(speed, speed, LSX, LSY);
    }
    else
    {
      SwerveSubsystem.DriveDriverOrientedAtAngle(LSX,LSY,RSAngle+180,Functions.Pythagorean(RSX, RSY));
    }

    //Arm
    ArmSubsystem.moveElevator(LSY2*0.35);
    ArmSubsystem.rotateCoralIntake(RSY2*0.1);
    //ClimberSubsystem.rotateClimber(RSY2*0.5);
    /*if(Constants.controller2.getBackButton()) {
      ArmSubsystem.moveElevatorTo(0);
     // ClimberSubsystem.moveClimbers(-1,0);
    }
    else if (Constants.controller2.getStartButtonPressed()) {
      RPSStart();
    }
    else if (Constants.controller2.getStartButton()) {
      //RPS();
    }
    else {
      timeSinceRPSstart = 0;
      if(Constants.controller1.getBButton())
      {
        ArmSubsystem.IntakeRing();
      }
      else if(Constants.controller1.getXButton())
      {
        //score coral function goes here
      }
      else if(Constants.controller1.getRightTriggerAxis() > 0.5)
      {
        ArmSubsystem.IntakeRing();
      }
      else
      {
        if(Constants.controller2.getYButton())
        {
          
        }
        else
        {
          if(Constants.controller2.getRightBumper())
          {
            //ArmSubsystem.manualMoveArmTo();
          }
          else
          {
            ArmSubsystem.moveElevator(LSY2);
          }
          
        }
        if(Constants.controller2.getXButton())
          {
            ArmSubsystem.spinIntake(0.75);
          }
          else if(Constants.controller2.getBButton())
          {
            ArmSubsystem.spinIntake(-0.25);
          } else if (Constants.controller2.getAButton()) {
            ArmSubsystem.intake(0.75);
          }
          else
          {
            ArmSubsystem.spinIntake(-Constants.controller2.getLeftTriggerAxis());
          }
        
      }

     // ClimberSubsystem.moveClimbers(RSY2, RSX2);
    }*/



    //if(ArmSubsystem.hasCoral) ControllerRumble.RumbleBothControllersBothSides(0.5);
    //else ControllerRumble.RumbleBothControllersBothSides(0);
    
  }

  public void RPSStart() { //Called the frame the start button is pressed, initiating the rock paper scissors stuff
    timeSinceRPSstart = 0.0;
    RPS = (int)(Math.random()*3);
  }

 /*  public void RPS() { //This function is called any time the start button is currently pressed down. It handles the climber and arm motion for rock paper scissors.
    timeSinceRPSstart += 0.025;
    if (timeSinceRPSstart <= 4.0) {
      //The period of time when it's moving the climbers up and down and preparing to "shoot"
      double pos = (Functions.TriangleWave(timeSinceRPSstart-0.25)+1)*Constants.climberMaxHeight*0.25;
      ClimberSubsystem.moveClimbersTo(pos, pos, 1);
      ArmSubsystem.rotateArm(0);
    }
    else {
      switch (RPS) {
        case 0:
          //Rock
          ArmSubsystem.moveArmTo(45);
          ClimberSubsystem.moveClimbers(0, 0);
          break;
        case 1:
          //Paper
          ClimberSubsystem.moveClimbersTo(0, Constants.climberMaxHeight, 1);
          ArmSubsystem.rotateArm(0);
          break;
        case 2:
          //Scissors
          ClimberSubsystem.moveClimbersTo(Constants.climberMaxHeight, 0, 1);
          ArmSubsystem.rotateArm(0);
          break;
      } 
    }
  } */

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //for (int i = 0; i < Constants.allCoralsPos.length; i++) PositionEstimator.realCoralList.add(Constants.allCoralsPos[i]);
    Constants.timeSinceStartAtAutoStart = Timer.getFPGATimestamp();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //AutoSequences.AutoStart();

  }


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    /*if(selectedAutoSequence >= 7 && AutoSequences.isAutoTimeBetween(0.1, 16))
    {
      if (Math.abs(Functions.DeltaAngleDeg(PositionEstimator.angleToClosestCoral()+180, PositionEstimator.robotPosition.getRotation().getDegrees())) < 15 && PositionEstimator.distToClosestCoral() > 0.2+Constants.coralCameraForwardOffset && PositionEstimator.distToClosestCoral() < 3+Constants.coralCameraForwardOffset && !CoralDetector.coralVisible) {
      framesCoralNotPresent++;
      if (framesCoralNotPresent >= 40) {
        PositionEstimator.removeClosestCoral();
        framesCoralNotPresent = 0;
      }
    }
    else
      {
        framesCoralNotPresent = 0;
      }
    }*/
  }  
  public static boolean coralIgnoranceGetInpt(String inptStr) {
    return inptStr.toLowerCase().equals("n") || inptStr.toLowerCase().equals("no") || inptStr.toLowerCase().equals("none") || inptStr.toLowerCase().equals("0");
  }
  /*
  public static String BoolToHexString(boolean[] inptData) {
    if (inptData[0]) {
      if (inptData[1]) {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      } else {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      }
    } else {
      if (inptData[1]) {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      } else {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      }
    }
    return "0x" + ;
  } */
}
