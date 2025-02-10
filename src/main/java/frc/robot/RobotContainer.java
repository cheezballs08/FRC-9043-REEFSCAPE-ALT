package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.drivetrain.SetPositionRelativeToApriltag;
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
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.utils.DriveType;

@SuppressWarnings("unused")
public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(ControllerConstants.controllerPort);

  Trigger x = controller.x();
  Trigger a = controller.a();
  Trigger b = controller.b();
  Trigger y = controller.y();

  DrivetrainSubsystem drivetrainSubsystem = new DefaultSwerve();

  // Bunların hepsi sabitlere atılacak, şuanlık test amaçlı böyle, aynı zamanda şuan her şey blue alliance a göre.
  DriveCommand teleopDriveCommand = new DriveCommand(
    drivetrainSubsystem,
    DriveType.FieldRelative,
    () -> -controller.getLeftX() * 3,
    () -> -controller.getLeftY() * 3,
    () -> -controller.getRightX() * 3
  );

  InstantCommand resetOdometry = new InstantCommand(() -> drivetrainSubsystem.resetOdometry(RobotConstants.initialPose));


  CoralIntakeSubsystem coralIntakeSubsystem = new CoralIntakeSimSubsystem();

  IntakeCoral intakeCoral = new IntakeCoral(coralIntakeSubsystem);
  OuttakeCoral outtakeCoral = new OuttakeCoral(coralIntakeSubsystem);

  AngleCoralIntake angleToL1 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L1Angle);
  AngleCoralIntake angleToL2 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L2Angle);
  AngleCoralIntake angleToL3 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L3Angle);
  AngleCoralIntake angleToL4 = new AngleCoralIntake(coralIntakeSubsystem, CoralIntakeConstants.L4Angle);


  MechansimSim mechansimSim = new MechansimSim(coralIntakeSubsystem);
  
  /*AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  IntakeAlgea intakeAlgea = new IntakeAlgea(algaeIntakeSubsystem);
  OuttakeAlgea outtakeAlgea = new OuttakeAlgea(algaeIntakeSubsystem);

  
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  ClimbForward climbForward = new ClimbForward(climbSubsystem);
  ClimbBackward climbBackward = new ClimbBackward(climbSubsystem);

  
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  MoveElevator heightToL1 = new MoveElevator(elevatorSubsystem, ElevatorConstants.L1Height);
  MoveElevator heightToL2 = new MoveElevator(elevatorSubsystem, ElevatorConstants.L2Height);
  MoveElevator heightToL3 = new MoveElevator(elevatorSubsystem, ElevatorConstants.L3Height);
  MoveElevator heightToL4 = new MoveElevator(elevatorSubsystem, ElevatorConstants.L4Height);*/

  public RobotContainer() {
    /*NamedCommands.registerCommand("outtakeAlgea", outtakeAlgea);
    NamedCommands.registerCommand("intakeAlgea", intakeAlgea);
    NamedCommands.registerCommand("climbForward", climbForward);
    NamedCommands.registerCommand("climbBackward", climbBackward);
    NamedCommands.registerCommand("intakeCoral", intakeCoral);
    NamedCommands.registerCommand("outtakeCoral", outtakeCoral);
    NamedCommands.registerCommand("angleToL1", angleToL1);
    NamedCommands.registerCommand("angleToL2", angleToL2);
    NamedCommands.registerCommand("angleToL3", angleToL3);
    NamedCommands.registerCommand("angleToL4", angleToL4);
    NamedCommands.registerCommand("heightToL1", heightToL1);
    NamedCommands.registerCommand("heightToL2", heightToL2);
    NamedCommands.registerCommand("heightToL3", heightToL3);
    NamedCommands.registerCommand("heightToL4", heightToL4);*/

    this.configureBindings();

  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
    
    x.onTrue(angleToL1);
    a.onTrue(angleToL2);
    b.onTrue(angleToL3);
    y.onTrue(angleToL4);

  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto(AutoConstants.autoName);
  }
}
