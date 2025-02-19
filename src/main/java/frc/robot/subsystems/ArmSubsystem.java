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
  public static boolean correctHeight = false; //whether or not the arm is close enough to the correct angle to shoot to get the coral in the speaker
  public static boolean correctAngle = false;
  public static boolean lineBreak = false; //the distance measured by the ultrasonic hooked into the arduino
  public static boolean elevatorBottom = false;
  public static boolean elevatorTop = false;
  int noCoralFrames = 0; //the number of frames that have passed since the last time the ultrasonic sensor saw a Coral
  
  public ArmSubsystem() {
    
  }

  @Override
  public void periodic() {
    elevatorBottom = Constants.elevatorBottom.get();
    elevatorTop = Constants.elevatorTop.get();
    if (elevatorBottom) {
      elevator1Encoder.setPosition(0);
      elevator2Encoder.setPosition(0);
    }
    if (elevatorTop) {
      //elevator1Encoder.setPosition(Constants.maxElevatorHeight);
      //elevator2Encoder.setPosition(Constants.maxElevatorHeight);
    }
    coralAngle = -(coralEncoder.getPosition() * 1.)+0.; //sets the coralAngle appropriately
    elevatorHeight = angleToHeight(elevator1Encoder.getPosition());
    dist = PositionEstimator.distToSpeaker();
   // inRange = dist < Constants.maxShootingRange; //checks if robot is in range of the speaker
    lineBreak = Constants.linebreakSensor.get();

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
    DriverDisplay.elevatorTarget.setDouble(h);
  }
  public static void moveElevator(double t) { //moves the arm with a certain amount of power, ranging from 1 to -1. the funky stuff in the first line just limits the arm angle.
    t = Functions.Clamp(t, -Functions.Clamp(0.2*(elevatorHeight), 0, 1), Functions.Clamp(-(0.2*(elevatorHeight-Constants.maxElevatorHeight)), 0, 1)) + (elevatorBottom?0:Constants.elevatorGravMult);
    Constants.elevator1.set((Constants.elevator1Invert)?-t:t);
    Constants.elevator2.set((Constants.elevator2Invert)?-t:t);
    DriverDisplay.elevatorThrottle.setDouble(t);
  }

  public static void rotateCoralIntakeTo(double a) {
    a = Functions.Clamp(a, Constants.minCoralAngle, Constants.maxCoralAngle);
    rotateCoralIntake(Functions.Clamp((Constants.coralPMult*(a - coralAngle)) 
    -(Constants.coralDMult*coralEncoder.getVelocity()), 
    -Constants.maxCoralPivotSpeed, Constants.maxCoralPivotSpeed));
    DriverDisplay.intakeTarget.setDouble(a);
  }
  public static void rotateCoralIntake(double t) { //moves the arm with a certain amount of power, ranging from 1 to -1. the funky stuff in the first line just limits the arm angle.
    t = Functions.Clamp(t, -Functions.Clamp(0.2*(coralAngle), 0, 1), Functions.Clamp(-(0.2*(coralAngle-Constants.maxCoralAngle)), 0, 1)) + (Constants.coralGravMult*Math.cos(Math.toRadians(coralAngle)));
    Constants.coralIntakePivot.set((Constants.coralIntakePivotInvert)?-t:t);
  }

  public static void spinIntake(double input) //spins the intake at the inputted speed (-1 to 1), applying safety limits as needed.
  {
    Constants.coralIntake.set(Functions.Clamp(-input, -1, coralAngle < 15 ? 0 : 1));
  }
  public static void intake(double input) //spins the intake motor in an attempt to pick up a Coral, stops once a Coral has been collected.  
  {
    Constants.coralIntakeConfig.idleMode(IdleMode.kBrake);
    spinIntake((hasCoral)?0:input);
  }
  public static void IntakeRing() { //moves the arm to the intake position, and tries to pick up a Coral
    moveElevatorTo(Constants.intakeHeight);
    intake(0.75);
  }


  public static double angleToHeight (double angle) {
    return angle*Constants.angleToHeightRatio;
  }

  public static double heightToAngle (double height) {
    return height/Constants.angleToHeightRatio;
  }

}