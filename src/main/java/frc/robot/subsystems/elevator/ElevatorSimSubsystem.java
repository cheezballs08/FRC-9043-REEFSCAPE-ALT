package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSimSubsystem implements ElevatorSubsystem {

  ElevatorSim simulation;

  ProfiledPIDController controller;

  ElevatorFeedforward feedforward;

  LoggedMechanismLigament2d ligament;


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
    
    this.ligament = new LoggedMechanismLigament2d(
      "elevator", 
      ElevatorConstants.startingHeight, 
      90
    );  
  }

  @Override
  public void periodic() {}

  @Override
  public void setElevatorPosition(double desiredPosition) {
    double currentPosition = simulation.getPositionMeters();
    this.ligament.setLength(currentPosition);
    
    double currentVelocity = simulation.getVelocityMetersPerSecond();

    double output = controller.calculate(currentPosition, desiredPosition);

    output += feedforward.calculate(currentVelocity);

    this.setVoltages(output);

  }

  @Override
  public void setVoltages(double voltage) {
    simulation.setInputVoltage(voltage);
  }

  @Override
  public LoggedMechanismLigament2d getLigament() {
    return ligament;
  }
  
}
