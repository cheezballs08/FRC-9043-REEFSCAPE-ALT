package frc.robot;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import frc.robot.constants.AlgeaIntakeConstants;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class MechansimSim {
  LoggedMechanism2d ui;

  LoggedMechanismRoot2d root;

  LoggedMechanismLigament2d coralLigament;

  LoggedMechanismLigament2d algeaLigament;

  LoggedMechanismLigament2d elevatorLigament;

  public MechansimSim(CoralIntakeSubsystem coralIntakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.ui = new LoggedMechanism2d(2, 2);
    
    this.root = ui.getRoot("MechansimRoot", 1.20, 0.05);

    this.coralLigament = coralIntakeSubsystem.getLigament();

    this.elevatorLigament = elevatorSubsystem.getLigament();

    root.append(coralLigament);
  }

  public MechansimSim() {
    this.ui = new LoggedMechanism2d(2, 2);

    this.root = ui.getRoot("MechansimRoot", 1.25, 0.60);

    this.algeaLigament = new LoggedMechanismLigament2d(
      "AlgeaLigament", 
      AlgeaIntakeConstants.lenght, 
      0
    );

    root.append(algeaLigament);
  }

  public void update() {
    Logger.recordOutput("Mechansim", ui);
  }

}
