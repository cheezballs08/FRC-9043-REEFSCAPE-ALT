// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorMotorVoltage extends Command {

  ElevatorSubsystem elevatorSubsystem;

  double voltage;

  public SetElevatorMotorVoltage(ElevatorSubsystem elevatorSubsystem, double voltage) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.voltage = voltage;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setVoltages(voltage);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setVoltages(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setVoltages(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
