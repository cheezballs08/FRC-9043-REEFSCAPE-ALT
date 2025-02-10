package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.utils.Logger;

public class MechansimSim {
  CoralIntakeSubsystem coralIntakeSubsystem;

  Mechanism2d ui;

  MechanismRoot2d root;

  MechanismLigament2d ligament;;

  public MechansimSim(CoralIntakeSubsystem coralIntakeSubsystem) {
    
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    
    this.ui = new Mechanism2d(100,100);
    
    this.root  = ui.getRoot("MechansimRoot", 0.20, 0.25);

    this.ligament = coralIntakeSubsystem.getLigament();
  
    root.append(ligament);
  }

  public void update() {
    Logger.log("Mechansim", ui);
  }

}
