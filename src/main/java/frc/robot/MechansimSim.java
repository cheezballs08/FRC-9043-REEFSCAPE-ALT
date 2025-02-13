package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.constants.AlgeaIntakeConstants;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.utils.Logger;

public class MechansimSim {
  Mechanism2d ui;

  MechanismRoot2d root;

  MechanismLigament2d coralLigament;

  MechanismLigament2d algeaLigament;

  MechanismLigament2d elevatorLigament;

  public MechansimSim(CoralIntakeSubsystem coralIntakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.ui = new Mechanism2d(2, 2);
    
    this.root = ui.getRoot("MechansimRoot", 1.20, 0.05);

    this.coralLigament = coralIntakeSubsystem.getLigament();

    this.elevatorLigament = elevatorSubsystem.getLigament();

    root.append(coralLigament);
  }

  public MechansimSim() {
    this.ui = new Mechanism2d(2, 2);

    this.root = ui.getRoot("MechansimRoot", 1.25, 0.60);

    this.algeaLigament = new MechanismLigament2d(
      "AlgeaLigament", 
      AlgeaIntakeConstants.lenght, 
      0
    );

    root.append(algeaLigament);
  }

  public void update() {
    Logger.log("Mechansim", ui);
  }

}
