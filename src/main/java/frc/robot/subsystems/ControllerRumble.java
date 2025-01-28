// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControllerRumble extends SubsystemBase {
  public static double cycleHighIntensity = 0;
  public static double cycleLowIntensity = 0;
  public static double cycleCycle = 0;

  public static double controller1RumbleL;
  public static double controller1RumbleR;
  public static double controller2RumbleL;
  public static double controller2RumbleR;

  public static double controller1RumbleLtarget;
  public static double controller1RumbleRtarget;
  public static double controller2RumbleLtarget;
  public static double controller2RumbleRtarget;
    /** Creates a new ExampleSubsystem. */
  public ControllerRumble() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Constants.controller1.setRumble(RumbleType.kLeftRumble, controller1RumbleL);
    Constants.controller1.setRumble(RumbleType.kRightRumble, controller1RumbleR);
    Constants.controller2.setRumble(RumbleType.kLeftRumble, controller2RumbleL);
    Constants.controller2.setRumble(RumbleType.kRightRumble, controller2RumbleR);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void RumbleCycle() {
  }

  public static void RumbleBothControllersBothSides(double intensity) {
    controller1RumbleL = intensity;
    controller1RumbleR = intensity;
    controller2RumbleL = intensity;
    controller2RumbleR = intensity;
  }

  public static void smoothRumbleTransitionBothControllersBothSides(double step, double target) {
    smoothTransition1L(step, target);
    smoothTransition1R(step, target);
    smoothTransition2L(step, target);
    smoothTransition2R(step, target);
  }

  public static void smoothTransition1L(double step, double target) {
    controller1RumbleLtarget = target;
    if (Math.abs(controller1RumbleL - controller1RumbleLtarget) <= target) controller1RumbleL = controller1RumbleLtarget;
    else if (controller1RumbleL - controller1RumbleLtarget > 0) controller1RumbleL -= step;
    else if (controller1RumbleL - controller1RumbleLtarget < 0) controller1RumbleL += step;
  }

  public static void smoothTransition1R(double step, double target) {
    controller1RumbleRtarget = target;
    if (Math.abs(controller1RumbleR - controller1RumbleRtarget) <= target) controller1RumbleR = controller1RumbleRtarget;
    else if (controller1RumbleR - controller1RumbleRtarget > 0) controller1RumbleR -= step;
    else if (controller1RumbleR - controller1RumbleRtarget < 0) controller1RumbleR += step;
  }
  
  public static void smoothTransition2L(double step, double target) {
    controller2RumbleLtarget = target;
    if (Math.abs(controller2RumbleL - controller2RumbleLtarget) <= target) controller2RumbleL = controller2RumbleLtarget;
    else if (controller2RumbleL - controller2RumbleLtarget > 0) controller2RumbleL -= step;
    else if (controller2RumbleL - controller2RumbleLtarget < 0) controller2RumbleL += step;
  }
  
  public static void smoothTransition2R(double step, double target) {
    controller2RumbleRtarget = target;
    if (Math.abs(controller2RumbleR - controller2RumbleRtarget) <= target) controller2RumbleR = controller2RumbleRtarget;
    else if (controller2RumbleR - controller2RumbleRtarget > 0) controller2RumbleR -= step;
    else if (controller2RumbleR - controller2RumbleRtarget < 0) controller2RumbleR += step;
  }
  
}
