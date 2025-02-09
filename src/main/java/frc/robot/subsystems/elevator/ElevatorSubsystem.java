package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.utils.CANCoderWrapper;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  SparkMax motor1, motor2;

  CANCoderWrapper encoder;

  ProfiledPIDController controller;

  public ElevatorSubsystem() {
    this.motor1 = new SparkMax(ElevatorConstants.motor1ID, ElevatorConstants.motor1type);
    this.motor1.configure(ElevatorConstants.motor1Config, MotorConstants.resetMode, MotorConstants.persistMode);

    this.motor2 = new SparkMax(ElevatorConstants.motor2ID, ElevatorConstants.motor2type);
    this.motor1.configure(ElevatorConstants.motor2Config, MotorConstants.resetMode, MotorConstants.persistMode);
    
    this.encoder = new CANCoderWrapper(new CANcoder(ElevatorConstants.encoderID));

    this.encoder.setPositionConversionFactor(ElevatorConstants.encoderPositionConversionFactor);
    this.encoder.setVelocityConversionFactor(ElevatorConstants.encoderVelocityConversionFactor);

    this.controller = new ProfiledPIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, ElevatorConstants.constraints);
    this.controller.setIZone(ElevatorConstants.IZ);
  }

  @Override
  public void periodic() {

  }

  public void setSpeeds(double speed) {
    this.motor1.set(speed);
    this.motor2.set(speed);
  }

  public void setElevatorPosition(double desiredPosition) {

    if (desiredPosition > ElevatorConstants.elevatorHeight || desiredPosition < 0) {
      System.err.println("İstenen pozisyon ya çok büyük ya da 0'dan küçük");
    
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

  public boolean isAtSetpoint() {
    return controller.atSetpoint();
  }
}
