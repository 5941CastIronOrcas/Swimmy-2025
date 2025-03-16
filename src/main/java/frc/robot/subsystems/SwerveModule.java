package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.Functions;

import com.ctre.phoenix6.hardware.CANcoder;

/** The SwerveModule class takes the input angle and throttle from SwerveSubsystem, and rotates the angle and throttle motors.
 **/

public class SwerveModule {
    private SparkMax angleMotor;
    private SparkMax throttleMotor;
    private CANcoder encoder;
    private double driveRatio = 1./8.14;
    private float aMult;
    private float tMult;
    public double anglePos;
    //public double velocity;
    public double defaultAngle;
    public SwerveModule (SparkMax angleMotorIN, SparkMax throttleMotorIN, CANcoder encoderIN, boolean aMotorInvert, boolean tMotorInvert, double dAngle, double driveRatioIN) { //constructor used for each swerve module
        
        angleMotor = angleMotorIN;
        throttleMotor = throttleMotorIN;
        encoder = encoderIN;
        aMult = ((aMotorInvert)?-1:1);
        tMult = ((tMotorInvert)?-1:1);
        defaultAngle = dAngle;
        driveRatio=driveRatioIN;
        
    }
    public double GetVelocity() //gets the velocity that the wheel is moving at in m/s
    {
        return tMult * Functions.DeadZone(((throttleMotor.getEncoder().getVelocity() + 
        (encoder.getVelocity().getValueAsDouble()*(27.0 / 17.0)*(1.0/3.0)*60))/60.0) * driveRatio * 
        Constants.swerveWheelCircumference, 0.00001);
    }
    public double GetAngle() //gets the angle of the wheel
    {
        return Functions.DeltaAngleDeg(0, encoder.getPosition().getValueAsDouble() * 360);
    }
    public void Drive(double angle, double speed) { //turns the wheel to the correct angle, and spins the throttle motor.
        double throttle = speed;
        //anglePos = encoder.getAbsolutePosition().getValueAsDouble() * 360;
        
        anglePos = GetAngle();
        //velocity = tMult * Functions.DeadZone(((throttleMotor.getEncoder().getVelocity() + (encoder.getVelocity().getValueAsDouble()*(27.0 / 17.0)*(1.0/3.0)*60))/60.0) * Constants.swerveDriveRatio * Constants.swerveWheelCircumference, 0.00001);

        if(Math.abs(speed) < 0.001) { //if the target speed is slow enough, it rotates the wheels to make a diamond shape, improving traction when the robot isn't moving.
            //angle = anglePos;
            angle = defaultAngle;
        }

        if (Math.abs(Functions.DeltaAngleDeg(anglePos, angle)) > Math.abs(Functions.DeltaAngleDeg(anglePos, angle + 180))) { //if the angle on the other side is closer, it just goes to that angle and rotates the wheel backwards.
            angle += 180;
            throttle *= -1;
        }
        //double compensationSpeed = -encoder.getVelocity().getValueAsDouble()*(27.0 / 17.0)*(1.0/3.0)*Constants.swerveDriveRatio*60;
        //double currentSpeed = throttleMotor.getEncoder().getVelocity();
        angleMotor.set(Functions.Clamp(aMult*Functions.DeltaAngleDeg(angle, anglePos) * (Constants.modulePMult), -1,1)); //uses a P controller to rotate the wheel to the desired position.
       // throttleMotor.set(throttleMotor.get()+((compensationSpeed-currentSpeed)*0.00001));
        throttleMotor.set(throttle * tMult);
        
    }
}