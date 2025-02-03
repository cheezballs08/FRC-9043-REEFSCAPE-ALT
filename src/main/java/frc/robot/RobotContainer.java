package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.teleop.algea.IntakeAlgea;
import frc.robot.commands.teleop.algea.OuttakeAlgea;
import frc.robot.commands.teleop.climb.ClimbBackward;
import frc.robot.commands.teleop.climb.ClimbForward;
import frc.robot.commands.teleop.coral.AngleCoralIntake;
import frc.robot.commands.teleop.coral.IntakeCoral;
import frc.robot.commands.teleop.coral.OuttakeCoral;
import frc.robot.commands.teleop.drivetrain.TeleopDriveCommand;
import frc.robot.commands.teleop.elevator.SetElevatorPositionCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swerve.DrivetrainSubsystem;
import frc.robot.utils.DriveType;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(ControllerConstants.controllerPort);
  
  DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(
    drivetrainSubsystem,
    DriveType.FieldRelative,
    controller::getLeftX,
    controller::getLeftY,
    controller::getRightX
  );

  
  AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  IntakeAlgea intakeAlgea = new IntakeAlgea(algaeIntakeSubsystem);
  OuttakeAlgea outtakeAlgea = new OuttakeAlgea(algaeIntakeSubsystem);

  
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  ClimbForward climbForward = new ClimbForward(climbSubsystem);
  ClimbBackward climbBackward = new ClimbBackward(climbSubsystem);


  CoralIntakeSubsystem coralIntakeSubsystem = new CoralIntakeSubsystem();

  IntakeCoral intakeCoral = new IntakeCoral(coralIntakeSubsystem);
  OuttakeCoral outtakeCoral = new OuttakeCoral(coralIntakeSubsystem);
  AngleCoralIntake angleToL1 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L1Angle);
  AngleCoralIntake angleToL2 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L2Angle);
  AngleCoralIntake angleToL3 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L3Angle);
  AngleCoralIntake angleToL4 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L4Angle);

  
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  SetElevatorPositionCommand heightToL1 = new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.L1Height);
  SetElevatorPositionCommand heightToL2 = new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.L2Height);
  SetElevatorPositionCommand heightToL3 = new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.L3Height);
  SetElevatorPositionCommand heightToL4 = new SetElevatorPositionCommand(elevatorSubsystem, ElevatorConstants.L4Height);

  // TODO: Named commandler i buraya ekle, şuan nedense olmuyor 

  public RobotContainer() {
    this.configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto(AutoConstants.autoName);
  }
}
