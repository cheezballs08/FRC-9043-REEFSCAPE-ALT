// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPositionCommand extends Command {

  ElevatorSubsystem subsystem;
  
  double desiredPosition;

  public SetElevatorPositionCommand(ElevatorSubsystem subsystem, double desiredPosition) {
    this.subsystem = subsystem;
    this.desiredPosition = desiredPosition;

    addRequirements(this.subsystem);
  }

  @Override
  public void initialize() {

    if (this.isFinished()) {
      CommandScheduler.getInstance().cancel(this);
    }

  }

  @Override
  public void execute() {
    subsystem.setElevatorPosition(desiredPosition);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    
    double error = desiredPosition - subsystem.getEncoderPosition();

    if (Math.abs(error) < MathConstants.epsilon && subsystem.getEncoderVelocity() < MathConstants.epsilon) {
      return true;
    }

    return false;
  }
}
