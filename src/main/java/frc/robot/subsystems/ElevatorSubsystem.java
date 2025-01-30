package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.CANCoderWrapper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  SparkMax elevatorMotor1, elevatorMotor2;

  CANCoderWrapper encoder;

  PIDController controller;

  public ElevatorSubsystem() {
    this.elevatorMotor1 = new SparkMax(ElevatorConstants.elevatorMotor1ID, ElevatorConstants.elevatorMotor1type);
    this.elevatorMotor1.configure(ElevatorConstants.motor1Config, ModuleConstants.resetMode, ModuleConstants.persistMode);

    this.elevatorMotor2 = new SparkMax(ElevatorConstants.elevatorMotor2ID, ElevatorConstants.elevatorMotor2type);
    this.elevatorMotor1.configure(ElevatorConstants.motor2Config, ModuleConstants.resetMode, ModuleConstants.persistMode);
    
    this.encoder = new CANCoderWrapper(new CANcoder(ElevatorConstants.encoderID));

    this.encoder.setPositionConversionConstant(ElevatorConstants.encoderSpeedConversionFactor);
    this.encoder.setVelocityConversionConstant(ElevatorConstants.encoderAccelerationConversionFactor);

    this.controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    this.controller.setIZone(ElevatorConstants.kIZ);
  }

  @Override
  public void periodic() {

  }

  public void setSpeeds(double speed) {
    this.elevatorMotor1.set(speed);
    this.elevatorMotor2.set(speed);
  }

  public void setElevatorPosition(double desiredPosition) {

    if (desiredPosition > ElevatorConstants.elevatorHeight) {
      System.err.println("Desired position is too high");
      
      return;
      
    }

    double currentPosition = encoder.getPosition();

    double output = controller.calculate(currentPosition, desiredPosition);

    this.setSpeeds(output);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public double getEncoderVelocity() {
    return encoder.getVelocity();
  }



}
