// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.commands.teleop.SetElevatorPostitionCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(ControllerConstants.controllerPort);

  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  SetElevatorPostitionCommand setPositionToL1 = new SetElevatorPostitionCommand(elevatorSubsystem, ElevatorConstants.level1Height);
  SetElevatorPostitionCommand setPositionToL2 = new SetElevatorPostitionCommand(elevatorSubsystem, ElevatorConstants.level2Height);
  SetElevatorPostitionCommand setPositionToL3 = new SetElevatorPostitionCommand(elevatorSubsystem, ElevatorConstants.level3Height);
  SetElevatorPostitionCommand setPositionToL4 = new SetElevatorPostitionCommand(elevatorSubsystem, ElevatorConstants.level4Height);

  Trigger xTrigger = controller.x();
  Trigger aTrigger = controller.a();
  Trigger bTrigger = controller.b();
  Trigger yTrigger = controller.y();

  Trigger leftBumper = controller.leftBumper();
  Trigger rightBumper = controller.rightBumper();

  public RobotContainer() {
    this.configureBindings();
  }

  private void configureBindings() {
    xTrigger.onTrue(setPositionToL1);
    aTrigger.onTrue(setPositionToL2);
    bTrigger.onTrue(setPositionToL3);
    yTrigger.onTrue(setPositionToL4);
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto(AutoConstants.autoName);
  }
}
