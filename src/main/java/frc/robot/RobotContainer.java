package frc.robot;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.drivetrain.ChaseApriltag;
import frc.robot.commands.auto.drivetrain.ChaseBestApriltag;
import frc.robot.commands.base.DriveCommand;
import frc.robot.commands.teleop.algea.IntakeAlgea;
import frc.robot.commands.teleop.algea.OuttakeAlgea;
import frc.robot.commands.teleop.climb.ClimbBackward;
import frc.robot.commands.teleop.climb.ClimbForward;
import frc.robot.commands.teleop.coral.AngleCoralIntake;
import frc.robot.commands.teleop.coral.IntakeCoral;
import frc.robot.commands.teleop.coral.MaintainAngle;
import frc.robot.commands.teleop.coral.OuttakeCoral;
import frc.robot.commands.teleop.elevator.MaintainHeight;
import frc.robot.commands.teleop.elevator.MoveElevator;
import frc.robot.commands.teleop.elevator.SetElevatorMotorVoltage;
import frc.robot.constants.AlgeaIntakeConstants;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.algea.AlgaeIntakeSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSimSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.drivetrain.DefaultSwerve;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorRealSubsystem;
import frc.robot.subsystems.elevator.ElevatorSimSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.ArticulationHelper;
import frc.robot.utils.CameraPosition;
import frc.robot.utils.DriveType;
import frc.robot.utils.Logger;
import frc.robot.utils.MechansimSim;

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
    () -> Math.abs(controller.getLeftY()) > ControllerConstants.deadband ? controller.getLeftY() * -3 : 0,
    () -> Math.abs(controller.getLeftX()) > ControllerConstants.deadband ? controller.getLeftX() * -3 : 0,
    () -> Math.abs(controller.getRightX()) > ControllerConstants.deadband ? controller.getRightX() * -3 : 0 
  );

  InstantCommand resetOdometry = new InstantCommand(() -> drivetrainSubsystem.resetOdometry(RobotConstants.initialPose));

  InstantCommand cancelCommands = new InstantCommand(() -> CommandScheduler.getInstance().cancelAll());

  /* <--------------------------------------------------------------------------------------------------------------------> */
  
  ElevatorSubsystem elevatorSubsystem = new ElevatorSimSubsystem();

  public RobotContainer() {}

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
    elevatorSubsystem.setDefaultCommand(new SetElevatorMotorVoltage(elevatorSubsystem, 0));

    rb.and(rt).onTrue(cancelCommands);

    x.onTrue(new SetElevatorMotorVoltage(elevatorSubsystem, 6));
    b.onTrue(new SetElevatorMotorVoltage(elevatorSubsystem, -1));
  }

  public void initialize() {
    RobotConstants.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    
    RobotConstants.updateInitialPose();
    AutoConstants.updatePoses();
    AutoConstants.logPoses();
    
    this.configureBindings();
    this.namedCommands();

    drivetrainSubsystem.resetOdometry(RobotConstants.initialPose);
  }

  public void periodic() {
    RobotConstants.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Logger.log("Robot/Alliance", RobotConstants.alliance);
  }

  public void namedCommands() {}

  public Command getAutonomousCommand() {
    return Commands.print("SG AMK OTONOM FALAN YOK");
  }
}
