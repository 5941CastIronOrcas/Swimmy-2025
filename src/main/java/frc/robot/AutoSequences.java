package frc.robot;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.GamePieceDetector;
import frc.robot.subsystems.PositionEstimator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilityObjects.Vector2D;

public class AutoSequences {
  public static int[] noteList;

  public static boolean wasInDriveToNote = false;
  public static boolean hadNote = false;
  public static int succesfulShots = 0;

  public static void AutoStart() {
    wasInDriveToNote = false;
    hadNote = false;
    succesfulShots = 0;
    Constants.gyro.setYaw(DriverDisplay.angleToAssign);
    PositionEstimator.realNoteList = new ArrayList<>();
    for (int i = 0; i < noteList.length; i++) 
    {
        PositionEstimator.realNoteList.add(Constants.allNotesPos[noteList[i]-1]);
      
    }
  }

  /* kill all the motors
    public static void autoSequence0() {
        Functions.killAllMotors();
    }

  // retract climbers
    public static void autoSequence1() {
        if (isAutoTimeBetween(0, 3)) ClimberSubsystem.moveClimbers(-1, 0);
        else Functions.killAllMotors();
    }

  // drive out
    public static void autoSequence2() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if(isAutoTimeBetween(0.0, 1.5)) SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
        else Functions.killAllMotors();
    }

  // shoot basic
    public static void autoSequence3() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 1)) {
            ArmSubsystem.manualMoveArmTo();
            ArmSubsystem.SpinShooter(1);
        } else if (isAutoTimeBetween(1, 2)) {
            ArmSubsystem.SpinIntake(0.75);
            ArmSubsystem.manualMoveArmTo();
            ArmSubsystem.SpinShooter(1);
        }
        else Functions.killAllMotors();
    }

  //shoot aimbot
    public static void autoSequence4() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 2)) {
            ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
            ArmSubsystem.ShootSpeaker();
            SwerveSubsystem.FaceSpeaker(0, 0, 0.25);
        } else Functions.killAllMotors();
    }

  // shoot basic, drive out
    public static void autoSequence5() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 2)) 
        {
          ArmSubsystem.manualMoveArmTo();
          ArmSubsystem.SpinShooter(1); 
        }
        else if (isAutoTimeBetween(2, 3)) {
            SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            ArmSubsystem.manualMoveArmTo();
            ArmSubsystem.SpinShooter(1);
            ArmSubsystem.SpinIntake(0.75);
        } else if (isAutoTimeBetween(3, 4.85)) {
          ArmSubsystem.SpinIntake(0);
          ArmSubsystem.SpinShooter(0);
          ArmSubsystem.rotateArm(0);
            SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
        } else Functions.killAllMotors(); 
    }

  // drive out, shoot aimbot
    public static void autoSequence6() {
        ClimberSubsystem.moveClimbers(-1, 0);
    if (isAutoTimeBetween(0, 1.5)) {
        SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
    } else if (isAutoTimeBetween(1.5, 4.5)) {
        SwerveSubsystem.FaceSpeaker(0, 0, 0.25);
        ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
        ArmSubsystem.ShootSpeaker();
    } else Functions.killAllMotors();
    
    }

  // Retract Climbers, Shoot Basic, Collect Nearest, Shoot
    public static void autoSequence7() {
      if (isAutoTimeBetween(0, 15) && succesfulShots <= 2 && PositionEstimator.realNoteList.size() > 0) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          hadNote = true;
          ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
          if (ArmSubsystem.dist < Constants.maxShootingRange) {
            ArmSubsystem.ShootSpeaker();
          } else  {
            if (succesfulShots == 0) {
              SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);
              ArmSubsystem.SpinIntake(0);
            } else if (succesfulShots == 1) {
              SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            }
          }

        } else {
          if (hadNote) {
            succesfulShots++;
            if (wasInDriveToNote) PositionEstimator.removeClosestNote();
            wasInDriveToNote = false;
          }
          hadNote = false;
          if (GamePieceDetector.noteVisible && GamePieceDetector.noteDist < 3) {
            ArmSubsystem.IntakeRing();
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = Constants.allNotesPos[PositionEstimator.nearestAutoNote()];
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote()+180, 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote()+180, 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
    }

  // Retract Climbers, Shoot Aimbot, Collect Nearest, Shoot
  public static void autoSequence8() {
      if (isAutoTimeBetween(0, 15) && succesfulShots <= 2 && PositionEstimator.realNoteList.size() > 0) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          if (ArmSubsystem.dist < Constants.maxShootingRange) {
            SwerveSubsystem.FaceSpeaker(0, 0, 1);
            ArmSubsystem.PrepShooter(1);
            ArmSubsystem.ShootSpeaker2();
          }
          else  {
            SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);
            ArmSubsystem.SpinIntake(0);
          }
          
        } else {
          if (hadNote) {
            succesfulShots++;
            if (wasInDriveToNote) PositionEstimator.removeClosestNote();
            wasInDriveToNote = false;
          }
          hadNote = false;
          ArmSubsystem.IntakeRing();
          if (GamePieceDetector.noteVisible && GamePieceDetector.noteDist < 3) {
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = PositionEstimator.realNoteList.get(PositionEstimator.nearestAutoNote());
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote()+180, 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote()+180, 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
    
  }

  // Retract Climbers, Shoot Basic, Collect and Shoot as Many as Possible
  public static void autoSequence9() {
      if (isAutoTimeBetween(0, 15) && PositionEstimator.realNoteList.size() > 0) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          hadNote = true;
          ArmSubsystem.PrepShooter(1);
          if (ArmSubsystem.dist < Constants.maxShootingRange) {
            ArmSubsystem.ShootSpeaker();
          }
          else  {
            if (succesfulShots == 0) {
              SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            } else if (succesfulShots == 1) {
              SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);
              ArmSubsystem.SpinIntake(0);
            }         
          }

        } else {
          if (hadNote) {
            succesfulShots++;
            if (wasInDriveToNote) PositionEstimator.removeClosestNote();
            wasInDriveToNote = false;
          }
          hadNote = false;
          if (GamePieceDetector.noteVisible && GamePieceDetector.noteDist < 3) {
            ArmSubsystem.IntakeRing();
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = Constants.allNotesPos[PositionEstimator.nearestAutoNote()];
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote()+180, 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote()+180, 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
  }

  // Retract Climbers, Shoot Aimbot, Collect and Shoot as Many as Possible
  public static void autoSequence10() {
    SmartDashboard.putNumber("Fuckin auto", succesfulShots);
      if (isAutoTimeBetween(0, 15) && !(!(PositionEstimator.realNoteList.size() > 0) && !ArmSubsystem.hasNote)) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          hadNote = true;
          if (ArmSubsystem.dist < Constants.maxShootingRange) {
            SwerveSubsystem.FaceSpeaker(0, 0, 1);
            ArmSubsystem.PrepShooter(1);
            ArmSubsystem.ShootSpeaker2();
          }
          else  {
            SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.8, 0.5, 0, 0);
            ArmSubsystem.SpinIntake(0);
          }
          
        } else {
          if (hadNote) {
            succesfulShots++;
            if (wasInDriveToNote && succesfulShots > 1) {
              PositionEstimator.removeClosestNote();
              System.out.println("Trying to remove the note, I promise!!!");
            }
            wasInDriveToNote = false;
          }
          hadNote = false;
          ArmSubsystem.IntakeRing();
          if (GamePieceDetector.noteVisible && GamePieceDetector.noteDist < 2) {
            wasInDriveToNote = true;
            SwerveSubsystem.CollectNote(0, 0, 0.8);        
          } else {
            Vector2D closestNote = PositionEstimator.realNoteList.get(PositionEstimator.nearestAutoNote());
            if (Math.abs(Functions.DeltaAngleDeg(PositionEstimator.robotPosition.getRotation().getDegrees(), PositionEstimator.angleToClosestNote()+180)) < 10) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote()+180, 0.8, 0.5, 0, 0);
            else SwerveSubsystem.DriveFieldOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote()+180, 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
    
  }*/



  public static boolean isAutoTimeBetween(double timeMin, double timeMax) {
    return timeMin < Timer.getFPGATimestamp() - Constants.timeSinceStartAtAutoStart && timeMax > Timer.getFPGATimestamp() - Constants.timeSinceStartAtAutoStart ;
  }
  
}
 