package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSimSubsystem implements ElevatorSubsystem {

  ElevatorSim simulation;

  ProfiledPIDController controller;

  ElevatorFeedforward feedforward;

  public ElevatorSimSubsystem() {
    this.simulation  = new ElevatorSim(
      DCMotor.getNEO(2), 
      ElevatorConstants.gearing, 
      ElevatorConstants.elevatorMass, 
      ElevatorConstants.drumRadius, 
      0, 
      ElevatorConstants.elevatorHeight, 
      true, 
      ElevatorConstants.startingHeight, 
      0,
      0
    );
    
    this.controller = new ProfiledPIDController(
      ElevatorConstants.P, 
      ElevatorConstants.I, 
      ElevatorConstants.D, 
      ElevatorConstants.constraints
    );

    this.feedforward = new ElevatorFeedforward(
      ElevatorConstants.S, 
      ElevatorConstants.G, 
      ElevatorConstants.V, 
      ElevatorConstants.A
    );
  }

  @Override
  public void setElevatorPosition(double desiredPosition) {
    
  }

  @Override
  public void setSpeeds(double speed) {
    
  
  }

  @Override
  public void setVoltages(double voltage) {
  }
  
}
