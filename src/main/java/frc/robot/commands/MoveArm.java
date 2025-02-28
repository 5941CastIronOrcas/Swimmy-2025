// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.ArmSubsystem.*;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveArm extends Command {

  public MoveArm() {
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveElevatorTo(intakeHeight);
    rotateCoralIntakeTo(coralIntakeAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(elevatorHeight < intakeHeight+8 && coralAngle <coralIntakeAngle +8){
      end(true);
    }
    }

  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
