package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.drivetrain.ChaseApriltag;
import frc.robot.commands.base.DriveCommand;
import frc.robot.commands.teleop.algea.IntakeAlgea;
import frc.robot.commands.teleop.algea.OuttakeAlgea;
import frc.robot.commands.teleop.climb.ClimbBackward;
import frc.robot.commands.teleop.climb.ClimbForward;
import frc.robot.commands.teleop.coral.AngleCoralIntake;
import frc.robot.commands.teleop.coral.IntakeCoral;
import frc.robot.commands.teleop.coral.OuttakeCoral;
import frc.robot.commands.teleop.elevator.MoveElevator;
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
import frc.robot.subsystems.elevator.ElevatorSimSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.utils.DriveType;

@SuppressWarnings("unused")
public class RobotContainer {

  /* <--------------------------------------------------------------------------------------------------------------------> */

  CommandXboxController controller = new CommandXboxController(ControllerConstants.controllerPort);

  Trigger x = controller.x();
  Trigger a = controller.a();
  Trigger b = controller.b();
  Trigger y = controller.y();

  /* <--------------------------------------------------------------------------------------------------------------------> */

  DrivetrainSubsystem drivetrainSubsystem = new DefaultSwerve();

  DriveCommand teleopDriveCommand = new DriveCommand(
    drivetrainSubsystem,
    DriveType.FieldRelative,
    () -> -controller.getLeftX(),
    () -> -controller.getLeftY(),
    () -> -controller.getRightX() 
  );

  ChaseApriltag chaseApriltag18 = new ChaseApriltag(
    drivetrainSubsystem,
    18,
    0.5,
    0,
    0
  );

  InstantCommand resetOdometry = new InstantCommand(() -> drivetrainSubsystem.resetOdometry(RobotConstants.initialPose));

  /* <--------------------------------------------------------------------------------------------------------------------> */

  CoralIntakeSubsystem coralIntakeSubsystem = new CoralIntakeSimSubsystem();

  AngleCoralIntake toRestAngle = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.restAngle);
  AngleCoralIntake toFeedAngle = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.feedAngle);
  AngleCoralIntake toL1Angle = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L1Angle);
  AngleCoralIntake toL2Angle = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L2Angle);
  AngleCoralIntake toL3Angle = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L2Angle);
  AngleCoralIntake toL4Angle = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L4Angle);
  
  IntakeCoral intakeCoral = new IntakeCoral(coralIntakeSubsystem);
  OuttakeCoral outtakeCoral = new OuttakeCoral(coralIntakeSubsystem);

  /* <--------------------------------------------------------------------------------------------------------------------> */

  ElevatorSubsystem elevatorSubsystem = new ElevatorSimSubsystem();

  MoveElevator toRestHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.restHeight);
  MoveElevator toFeedHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.feedHeight);
  MoveElevator toL1Height = new MoveElevator(elevatorSubsystem, ElevatorConstants.L1Height);
  MoveElevator toL2Height = new MoveElevator(elevatorSubsystem, ElevatorConstants.L2Height);
  MoveElevator toL3Height = new MoveElevator(elevatorSubsystem, ElevatorConstants.L3Height);
  MoveElevator toL4Height = new MoveElevator(elevatorSubsystem, ElevatorConstants.L4Height);

  /* <--------------------------------------------------------------------------------------------------------------------> */

  ParallelCommandGroup restPosition = new ParallelCommandGroup(
    toRestAngle,
    toRestHeight
  );

  ParallelCommandGroup feedPosition = new ParallelCommandGroup(
    toFeedAngle,
    toFeedHeight
  );

  ParallelCommandGroup L1Position = new ParallelCommandGroup(
    toL1Angle,
    toL1Height
  );

  ParallelCommandGroup L2Position = new ParallelCommandGroup(
    toL2Angle,
    toL2Height
  );

  ParallelCommandGroup L3Position = new ParallelCommandGroup(
    toL3Angle,
    toL3Height
  );

  ParallelCommandGroup L4Position = new ParallelCommandGroup(
    toL4Angle,
    toL4Height
  );

  /* <--------------------------------------------------------------------------------------------------------------------> */

  SequentialCommandGroup takeCoral = new SequentialCommandGroup(
    feedPosition,
    intakeCoral
  );

  SequentialCommandGroup putCoralToL1 = new SequentialCommandGroup(
    L1Position,
    outtakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL2 = new SequentialCommandGroup(
    L2Position,
    outtakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL3 = new SequentialCommandGroup(
    L3Position,
    outtakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL4 = new SequentialCommandGroup(
    L4Position,
    outtakeCoral.asProxy()
  );

  /* <--------------------------------------------------------------------------------------------------------------------> */

  MechansimSim mechansimSim = new MechansimSim(coralIntakeSubsystem, elevatorSubsystem);

  MechansimSim algeaMechansimSim = new MechansimSim();

  /* <--------------------------------------------------------------------------------------------------------------------> */

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
    coralIntakeSubsystem.setDefaultCommand(restPosition);
    elevatorSubsystem.setDefaultCommand(restPosition);

    x.onTrue(restPosition);
    a.onTrue(L1Position);
    b.onTrue(L2Position);
    y.onTrue(feedPosition);

    x.and(b).toggleOnTrue(chaseApriltag18);


    x.and(a).and(b).and(y).onTrue(resetOdometry);

  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto(AutoConstants.autoName);
  }
}
