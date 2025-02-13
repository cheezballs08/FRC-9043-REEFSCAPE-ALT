package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSimSubsystem extends SubsystemBase {

  ElevatorSim simulation = new ElevatorSim(
    null, 
    0, 
    0, 
    0, 
    0, 
    0, 
    false, 
    0, 
    null
    );
  
}
