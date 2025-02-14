package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;


public class ArmSubsystem extends SubsystemBase {
  public static RelativeEncoder coralEncoder = Constants.coralIntakePivot.getAlternateEncoder();//getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); //the encoder that reads the arm's position
  public static RelativeEncoder elevator1Encoder = Constants.elevator1.getEncoder();
  public static RelativeEncoder elevator2Encoder = Constants.elevator2.getEncoder();
  public static double elevatorHeight = 0;
  public static double coralAngle = 0; //the current angle of the coral intake
  public static double oldSpeakerAngle = 0; //the angle at which the robot should put it's arm in order to shoot into the speaker one frame ago
  public static double newSpeakerAngle = 0; //the angle at which the robot should put it's arm in order to shoot into the speaker
  public static double dist = 0; //the distance between the center of the robot and the speaker
  public static boolean inRange = false; //if the robot is within the minimum shooting range
  public static double g = Constants.gravity; //gravitational acceleration m/s/s
  public static boolean hasCoral = false; //whether or not the robot is currently holding a coral
  public static boolean shooterFast = false; //whether or not the shooter is spinning at or above the minimum shoot speed
  public static boolean correctArmAngle = false; //whether or not the arm is close enough to the correct angle to shoot to get the coral in the speaker
  public static boolean lineBreak; //the distance measured by the ultrasonic hooked into the arduino
  public static boolean elevatorBottom = Constants.elevatorBottom.get();
  public static boolean elevatorTop = Constants.elevatorTop.get();
  int noCoralFrames = 0; //the number of frames that have passed since the last time the ultrasonic sensor saw a Coral
  
  public ArmSubsystem() {
    
  }

  @Override
  public void periodic() {
    if (elevatorBottom) {
      elevator1Encoder.setPosition(0);
      elevator2Encoder.setPosition(0);
    }
    if (elevatorTop) {
      elevator1Encoder.setPosition(Constants.maxElevatorHeight);
      elevator2Encoder.setPosition(Constants.maxElevatorHeight);
    }
    coralAngle = -(coralEncoder.getPosition() * 1.)+0.; //sets the ArmAngle appropriately
    elevatorHeight = angleToHeight(elevator1Encoder.getPosition());
    dist = PositionEstimator.distToSpeaker();
   // inRange = dist < Constants.maxShootingRange; //checks if robot is in range of the speaker

    //recalledValue = Arduino.getCallArduino((byte) 0x2e); //????????
      if(!lineBreak) //if linebreak can see a coral
      {
        hasCoral = true; //immediately assume the coral is real
        noCoralFrames = 0; //set the number of frames since a coral was seen to 0
      }
      else noCoralFrames++; //if linebreak can't see a coral, increase noCoralFrames
      if(noCoralFrames>5) hasCoral = false; //if it has been 40 frames since linebreak saw a coral, then assume the robot is not holding a coral

  }

  @Override
  public void simulationPeriodic() {

  }

  public static void moveElevatorTo(double h) { //uses a pd controller to go to a given angle.
   h = Functions.Clamp(h, 0, Constants.maxElevatorHeight);
   moveElevator(Functions.Clamp((Constants.elevatorPMult*(h - elevatorHeight)) 
    -(Constants.elevatorDMult*elevator1Encoder.getVelocity()), 
    -Constants.maxElevatorSpeed, Constants.maxElevatorSpeed));
     DriverDisplay.armTarget.setDouble(h);
  }
  
  public static void moveElevator(double t) { //moves the arm with a certain amount of power, ranging from 1 to -1. the funky stuff in the first line just limits the arm angle.
    t = Functions.Clamp(t, -Functions.Clamp(0.2*(elevatorHeight), 0, 1), Functions.Clamp(-(0.2*(elevatorHeight-Constants.maxElevatorHeight)), 0, 1)) + (elevatorBottom?0:Constants.elevatorGravMult);
    Constants.elevator1.set((Constants.elevator1Invert)?-t:t);
    Constants.elevator2.set((Constants.elevator2Invert)?-t:t);
    DriverDisplay.armThrottle.setDouble(t);
  }
  public static void SpinIntake(double input) //spins the intake at the inputted speed (-1 to 1), applying safety limits as needed.
  {
    Constants.coralIntake.set(Functions.Clamp(-input, -1, coralAngle < 15 ? 0 : 1));
  }
  public static void Intake(double input) //spins the intake motor in an attempt to pick up a Coral, stops once a Coral has been collected.  
  {
    Constants.coralIntakeConfig.idleMode(IdleMode.kBrake);
    SpinIntake((hasCoral)?0:input);
  }
  public static void IntakeRing() { //moves the arm to the intake position, and tries to pick up a Coral
    moveElevatorTo(Constants.intakeHeight);
    Intake(0.75);
  }


  public static double angleToHeight (double angle) {
    return angle*Constants.angleToHeightRatio;
  }

  public static double heightToAngle (double height) {
    return height/Constants.angleToHeightRatio;
  }

}