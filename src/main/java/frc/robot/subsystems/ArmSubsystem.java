package frc.robot.subsystems;

//import javax.smartcardio.CommandAPDU;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.utilityObjects.Vector2D;


public class ArmSubsystem extends SubsystemBase {
  public static SparkAbsoluteEncoder coralEncoder = Constants.algaeIntake.getAbsoluteEncoder();//Constants.coralEncoderSpark.getAlternateEncoder();//getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); //the encoder that reads the arm's position
  public static RelativeEncoder elevator1Encoder = Constants.elevator1.getEncoder();
  public static RelativeEncoder elevator2Encoder = Constants.elevator2.getEncoder();
  public static double oldElevatorAngle = 0;
  public static double newElevatorAngle = 0;
  public static double elevatorAngleOffset = 0;
  public static double elevatorHeight = 0;
  public static double coralAngle = 0; //the current angle of the coral intake
  public static double oldCoralAngle = 0; //the angle at which the robot should put it's arm in order to shoot into the speaker one frame ago
  public static double coralAngleTarget = 0;
  public static double dist = 0; //the distance between the center of the robot and the speaker
  public static double g = Constants.gravity; //gravitational acceleration m/s/s
  public static boolean hasCoral = false; //whether or not the robot is currently holding a coral
  public static boolean hasAlgae = false;
  public static boolean correctHeight = false; // whether or not the arm is close enough to the correct angle to shoot to get the coral in the speaker
  public static boolean correctAngle = false;
  public static boolean lineBreak = false; //the distance measured by the ultrasonic hooked into the arduino
  public static boolean elevatorBottom = false;
  public static boolean elevatorTop = false;
  public static double coralCompensation = 0;
  public static double intakeIntegral = 0.;
  public static double elevatorThrottle = 0.;
  int noCoralFrames = 0; //the number of frames that have passed since the last time the ultrasonic sensor saw a Coral

  public ArmSubsystem() {
    //coralEncoder.setPosition(0);
    //Constants.coralIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    //Constants.coralIntakeConfig.closedLoop.maxMotion.;
  }

  @Override
  public void periodic() {
    elevatorBottom = !Constants.elevatorBottom.get();
    elevatorTop = !Constants.elevatorTop.get();
    if (elevatorBottom) {
      elevator1Encoder.setPosition(0);
      elevator2Encoder.setPosition(0);
    }
    if (elevatorTop) {
      elevator1Encoder.setPosition(Constants.maxElevatorAngle);
      elevator2Encoder.setPosition(Constants.maxElevatorAngle);
    }
    oldCoralAngle = coralAngle;
    coralAngle = coralEncoder.getPosition()*360;//-(Math.toDegrees(coralEncoder.getPosition()))*2.09; //sets the coralAngle appropriately
    oldElevatorAngle = newElevatorAngle;
    if (oldElevatorAngle-newElevatorAngle>Constants.elevatorAngleOffsetThreshold) elevatorAngleOffset += 360;
    if (oldElevatorAngle-newElevatorAngle<-Constants.elevatorAngleOffsetThreshold) elevatorAngleOffset -= 360;
    newElevatorAngle = Math.toDegrees(elevator1Encoder.getPosition()) + elevatorAngleOffset;
    elevatorHeight = angleToHeight(newElevatorAngle);
    dist = PositionEstimator.distToSpeaker();
    //coralCompensation = getCompensation();
   // inRange = dist < Constants.maxShootingRange; //checks if robot is in range of the speaker
    lineBreak = Constants.linebreakSensor.get();

    //recalledValue = Arduino.getCallArduino((byte) 0x2e); //????????
    if(!lineBreak) //if linebreak can see a coral
    {
      hasCoral = true; //immediately assume the coral is real
      //noCoralFrames = 0; //set the number of frames since a coral was seen to 0
    }
    else hasCoral=false;
    //else noCoralFrames++; //if linebreak can't see a coral, increase noCoralFrames
    //if(noCoralFrames>5) hasCoral = false; //if it has been 40 frames since linebreak saw a coral, then assume the robot is not holding a coral
    intakeIntegral += Functions.DeltaAngleDeg(coralAngle, coralAngleTarget)*Constants.coralIMult;
    intakeIntegral = Functions.Clamp(intakeIntegral, -Constants.coralIClamp, Constants.coralIClamp);
  }

  @Override
  public void simulationPeriodic() {

  }
//commands
public Command comMoveArm(int level){
        double h = 0.0;
        double a = 0.0;

        switch (level) {
            case 0:
                h = Constants.intakeHeight;
                a = Constants.coralIntakeAngle;
                break;
            case 1:
                h = Constants.reef1Height;
                a = Constants.reef1Angle;
                break;

            case 2:
                h = Constants.reef2Height;
                a = Constants.reef2Angle;
                break;
            case 3:
                h = Constants.reef3Height;
                a = Constants.reef2Angle;
                break;
            case 4:
                h = Constants.reef4Height;
                a = Constants.reef4Angle;
                break;
            default:
                DriverStation.reportError("height not set", false);
                break;
        }
        final double h2, a2;
        h2 = h;
        a2 = a;

        return this.runOnce(() -> moveArmTo(h2, a2, 0));
  }
    public Command comSpinIntake(int i){
        double speed;
        switch (i) {
            case 0:
             speed = 0.0;
                break;
            case 1:
             speed = 0.75;
                break;
            case 2:
             speed = -0.75;
                break;

            default:
                speed = 0.0;
                break;
        }
        return this.runOnce(() -> intakeCoral(speed));
    }



  public static void moveElevatorTo(double h) { //uses a pd controller to go to a given angle.
   h = Functions.Clamp(h, 0, Constants.maxElevatorHeight);
   moveElevator(Functions.Clamp((Constants.elevatorPMult*(h - elevatorHeight))
    -(Constants.elevatorDMult*elevator1Encoder.getVelocity()),
    -Constants.maxElevatorSpeed, Constants.maxElevatorSpeed));
    DriverDisplay.elevatorTarget.setDouble(h);
  }
  public static void moveElevator(double t) { //moves the arm with a certain amount of power, ranging from 1 to -1. the funky stuff in the first line just limits the arm angle.
    elevatorThrottle += Functions.Clamp(t-elevatorThrottle, -Constants.elevatorAccelLimit, Constants.elevatorAccelLimit);
    t = Functions.Clamp(elevatorThrottle, -Functions.Clamp(0.2*(elevatorHeight), 0, 1), Functions.Clamp(-(0.2*(elevatorHeight-Constants.maxElevatorHeight)), 0, 1)) + (elevatorBottom?0:Constants.elevatorGravMult);
    Constants.elevator1.set((Constants.elevator1Invert)?-t:t);
    Constants.elevator2.set((Constants.elevator2Invert)?-t:t);
  }

  public static void rotateCoralIntakeTo(double a, double o) {
    if (hasCoral) {
      a = Functions.Clamp(a, Constants.minCoralAngle, Constants.maxCoralAngle);
    coralAngleTarget = a;
    rotateCoralIntake((Constants.withCoralPMult*Functions.DeltaAngleDeg(coralAngle, a))
    -(Constants.withCoralDMult*coralEncoder.getVelocity())
    +intakeIntegral-o);
    }
    else {
    a = Functions.Clamp(a, Constants.minCoralAngle, Constants.maxCoralAngle);
    coralAngleTarget = a;
    rotateCoralIntake((Constants.coralPMult*Functions.DeltaAngleDeg(coralAngle, a))
    -(Constants.coralDMult*coralEncoder.getVelocity())
    +intakeIntegral-o);
    }
    DriverDisplay.intakeTarget.setDouble(a);
  }
  public static void rotateCoralIntake(double t) { //moves the arm with a certain amount of power, ranging from 1 to -1. the funky stuff in the first line just limits the arm angle.
    //t = Functions.Clamp(t, -Functions.Clamp(0.2*(coralAngle-(Constants.minCoralAngle)), 0, 1), Functions.Clamp(-(0.2*(coralAngle-Constants.maxCoralAngle)), 0, 1));//+getCompensation();
    //t = Functions.Clamp(t, (coralAngle < (elevatorHeight/Constants.maxElevatorHeight)
   // *Constants.coralMinAdaptiveAngle)?0.2:-1, (coralAngle > ((Constants.maxElevatorHeight
   // -elevatorHeight)/Constants.maxElevatorHeight)*Constants.coralMaxAdaptiveAngle)?-0.2:1);
    t = Functions.Clamp(t - (hasCoral?Constants.withCoralGravMult:Constants.coralGravMult)*Math.sin(Math.toRadians(coralAngle-(hasCoral?Constants.coralGravOffset:Constants.withCoralGravOffset))), -Constants.maxCoralPivotSpeed, Constants.maxCoralPivotSpeed);
    Constants.coralIntakePivot.set((Constants.coralIntakePivotInvert)?-t:t);
  }

  public static void moveArmTo(double h, double a, double o) {
    moveElevatorTo(h);
    if (Math.abs(h-elevatorHeight)<3.) rotateCoralIntakeTo(a,o);
    else rotateCoralIntakeTo(Functions.Clamp(a, Constants.coralMinAdaptiveAngle, Constants.coralMaxAdaptiveAngle),o); //rotateCoralIntakeTo(45.,o);
    
    //rotateCoralIntakeTo(Functions.Clamp(coralAngle, Constants.coralMinAdaptiveAngle, elevatorHeight<10?Constants.coralMaxAdaptiveAngle:80), o);

  }

  public static void spinCoralIntake(double input) //spins the intake at the inputted speed (-1 to 1), applying safety limits as needed.
  {
    Constants.coralIntake.set(Functions.Clamp(-input, -0.5,  1));
  }
  public static void intakeCoral(double input) //spins the intake motor in an attempt to pick up a Coral, stops once a Coral has been collected.
  {
    Constants.coralIntakeConfig.idleMode(IdleMode.kBrake);
    spinCoralIntake(input<0.?((hasCoral)?0.05:input):input);
  }
  public static void moveToIntakeCoral() { //moves the arm to the intake position, and tries to pick up a Coral
    moveElevatorTo(Constants.intakeHeight);
    intakeCoral(0.75);
  }

  public static void spinAlgaeIntake(double input) //spins the intake at the inputted speed (-1 to 1), applying safety limits as needed.
  {
    Constants.algaeIntake.set(Functions.Clamp(-input, -0.5,  1));
  }
  public static void intakeAlgae(double input) //spins the intake motor in an attempt to pick up a Coral, stops once a Coral has been collected.
  {
    Constants.coralIntakeConfig.idleMode(IdleMode.kBrake);
    spinAlgaeIntake(input<0.?((hasAlgae)?0.05:input):input);
  }
  public static void moveToIntakeAlgae() { //moves the arm to the intake position, and tries to pick up a Coral
    moveElevatorTo(Constants.reef1Height);
    intakeAlgae(0.75);
  }


  public static double angleToHeight (double angle) {
    return angle*Constants.angleToHeightRatio;
  }

  public static double heightToAngle (double height) {
    return height/Constants.angleToHeightRatio;
  }

  /*public static double getCompensation() {
    if (Math.abs(coralAngle-oldCoralAngle)>Constants.compensationMinDeltaAngle) return coralCompensation;
    double centerRadius = Constants.intakeCenterRadius;
    double centerAngle = Math.toRadians(coralAngle+Constants.intakeCenterAngle);
    double springRadius = Constants.springRadius;
    double springAngle = Math.toRadians(coralAngle-Constants.springAngle);
    Vector2D springPos = new Vector2D(springRadius*Math.cos(springAngle), springRadius*Math.sin(springAngle));
    double totalSpringAngle = springAngle + Constants.springEndAngle;
    double springDist = Functions.Pythagorean(springPos.x+Constants.springEndPos.x, springPos.y-Constants.springEndPos.y);
    double angleToSpring = Math.asin((springDist*Math.sin(totalSpringAngle))/springRadius);
    double netTorque = (Constants.gForceTimesRadius*Math.sin(centerAngle))-(Constants.sForceTimesRadius*Math.sin(angleToSpring));
    coralCompensation = netTorque/2.6;
    return coralCompensation;
  }*/

}
