package frc.robot;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.base.DriveCommand;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.DefaultSwerve;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.utils.DriveType;

@SuppressWarnings("unused")
public class RobotContainer {

  /* <--------------------------------------------------------------------------------------------------------------------> */

  CommandXboxController controller = new CommandXboxController(ControllerConstants.controllerPort);

  Trigger x = controller.x();
  Trigger a = controller.a();
  Trigger b = controller.b();
  Trigger y = controller.y();
  Trigger rb = controller.rightBumper();
  Trigger rt = controller.rightTrigger(0.1);
  Trigger lb = controller.leftBumper();
  Trigger lt = controller.leftTrigger(0.1);

  /* <--------------------------------------------------------------------------------------------------------------------> */

  DrivetrainSubsystem drivetrainSubsystem = new DefaultSwerve();

  DriveCommand teleopDriveCommand = new DriveCommand(
    drivetrainSubsystem,
    DriveType.FieldRelative,
    () -> Math.abs(controller.getLeftY()) > ControllerConstants.deadband ? controller.getLeftY() * 3 : 0,
    () -> Math.abs(controller.getLeftX()) > ControllerConstants.deadband ? controller.getLeftX() * 3 : 0,
    () -> Math.abs(controller.getRightX()) > ControllerConstants.deadband ? controller.getRightX() * 3 : 0 
  );

  InstantCommand resetOdometry = new InstantCommand(() -> drivetrainSubsystem.resetOdometry(RobotConstants.initialPose));

  /* <--------------------------------------------------------------------------------------------------------------------> */

  public RobotContainer() {
    configureBindings();
    namedCommands();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
  
    x.and(a).and(b).and(y).onTrue(resetOdometry);
  
    rb.and(rt).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
  }

  public void periodic() {}

  public void namedCommands() {  }

  public Command getAutonomousCommand() {
    return Commands.print("Çalış amk pornooo");
  }
}
