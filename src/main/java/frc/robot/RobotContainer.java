package frc.robot;

import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.subsystems.elevator.ElevatorSimSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.ArticulationHelper;
import frc.robot.utils.CameraPosition;
import frc.robot.utils.DriveType;
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

  VisionProcessingUnit frontUnit = VisionProcessingUnit.getUnit(CameraPosition.Front);
  VisionProcessingUnit leftUnit = VisionProcessingUnit.getUnit(CameraPosition.Left);
  VisionProcessingUnit rightUnit = VisionProcessingUnit.getUnit(CameraPosition.Right);

  VisionSystemSim visionSimulation = VisionProcessingUnit.getSimulation();

  /* <--------------------------------------------------------------------------------------------------------------------> */

  DrivetrainSubsystem drivetrainSubsystem = new DefaultSwerve();

  DriveCommand teleopDriveCommand = new DriveCommand(
    drivetrainSubsystem,
    DriveType.FieldRelative,
    () -> Math.abs(controller.getLeftY()) > ControllerConstants.deadband ? controller.getLeftY() * 3 : 0,
    () -> Math.abs(controller.getLeftX()) > ControllerConstants.deadband ? controller.getLeftX() * 3 : 0,
    () -> Math.abs(controller.getRightX()) > ControllerConstants.deadband ? controller.getRightX() * 3 : 0 
  );

  ChaseApriltag chaseApriltag18 = new ChaseApriltag(
    drivetrainSubsystem,
    18,
    0.68,
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

  MoveElevator toCoralFeedHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.coralFeedHeight);
  MoveElevator toL1CoralHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.coralL1Height);
  MoveElevator toL2CoralHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.coralL2Height);
  MoveElevator toL3CoralHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.coralL3Height);
  MoveElevator toL4CoralHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.coralL4Height);

  MoveElevator toAlgeaStartHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.algeaStartHeight);
  MoveElevator toAlgeaOutputHeight = new MoveElevator(elevatorSubsystem, ElevatorConstants.algeaOutputHeight);
  MoveElevator toAlgeaStage1Height = new MoveElevator(elevatorSubsystem, ElevatorConstants.algeaStage1Height);
  MoveElevator toAlgeaStage2Height = new MoveElevator(elevatorSubsystem, ElevatorConstants.algeaStage2Height);

  /* <--------------------------------------------------------------------------------------------------------------------> */

  /* AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  IntakeAlgea intakeAlgea = new IntakeAlgea(algaeIntakeSubsystem);
  OuttakeAlgea outtakeAlgea = new OuttakeAlgea(algaeIntakeSubsystem);

  /* <--------------------------------------------------------------------------------------------------------------------> */
   
  /* ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  ClimbForward climbForward = new ClimbForward(climbSubsystem);
  ClimbBackward climbBackward = new ClimbBackward(climbSubsystem);

  /* <--------------------------------------------------------------------------------------------------------------------> */
  
  // TODO: Proxy için bir çözüm bul.
  ParallelCommandGroup restPosition = new ParallelCommandGroup(
    toRestAngle.asProxy(),
    toRestHeight.asProxy()
  );

  ParallelCommandGroup feedPosition = new ParallelCommandGroup(
    toFeedAngle.asProxy(),
    toCoralFeedHeight.asProxy()
  );

  ParallelCommandGroup L1CoralPosition = new ParallelCommandGroup(
    toL1Angle.asProxy(),
    toL1CoralHeight.asProxy()
  );

  ParallelCommandGroup L2CoralPosition = new ParallelCommandGroup(
    toL2Angle.asProxy(),
    toL2CoralHeight.asProxy()
  );

  ParallelCommandGroup L3CoralPosition = new ParallelCommandGroup(
    toL3Angle.asProxy(),
    toL3CoralHeight.asProxy()
  );

  ParallelCommandGroup L4CoralPosition = new ParallelCommandGroup(
    toL4Angle.asProxy(),
    toL4CoralHeight.asProxy()
  );

  /* <--------------------------------------------------------------------------------------------------------------------> */

  SequentialCommandGroup takeCoral = new SequentialCommandGroup(
    feedPosition.asProxy(),
    intakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL1 = new SequentialCommandGroup(
    L1CoralPosition.asProxy(),
    outtakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL2 = new SequentialCommandGroup(
    L2CoralPosition.asProxy(),
    outtakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL3 = new SequentialCommandGroup(
    L3CoralPosition.asProxy(),
    outtakeCoral.asProxy()
  );

  SequentialCommandGroup putCoralToL4 = new SequentialCommandGroup(
    L4CoralPosition.asProxy(),
    outtakeCoral.asProxy()
  );

  /* <--------------------------------------------------------------------------------------------------------------------> */

  MechansimSim mechansimSim = new MechansimSim(coralIntakeSubsystem, elevatorSubsystem);

  ArticulationHelper articulationHelper = new ArticulationHelper(drivetrainSubsystem, coralIntakeSubsystem, elevatorSubsystem);

  /* <--------------------------------------------------------------------------------------------------------------------> */

  public RobotContainer() {
    configureBindings();
    namedCommands();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
    coralIntakeSubsystem.setDefaultCommand(toRestAngle);
    elevatorSubsystem.setDefaultCommand(toRestHeight);

    x.and(rb).onTrue(L1CoralPosition);
    a.and(rb).onTrue(L2CoralPosition);
    b.and(rb).onTrue(L3CoralPosition);
    y.and(rb).onTrue(L4CoralPosition);
    
    x.and(lb).toggleOnTrue(chaseApriltag18);

    x.and(a).and(b).and(y).onTrue(resetOdometry);
  
    rb.and(rt).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
  }

  public void periodic() {
    frontUnit.update();
    leftUnit.update();
    rightUnit.update();

    visionSimulation.update(drivetrainSubsystem.getSimPose());

    mechansimSim.update();
    articulationHelper.update();
  }

  public void namedCommands() {
    NamedCommands.registerCommand("chaseApriltag18", chaseApriltag18);
    NamedCommands.registerCommand("L2CoralPosition", L2CoralPosition);
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto(AutoConstants.autoName);
  }
}
