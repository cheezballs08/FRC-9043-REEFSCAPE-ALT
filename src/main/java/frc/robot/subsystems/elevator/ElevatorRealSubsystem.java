package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.utils.CANCoderWrapper;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.LoggedMechanismLigament2dmLigament2d;

// TODO: Feedforward kullanımı ekle.
public class ElevatorRealSubsystem implements ElevatorSubsystem {
  
  SparkMax motor1, motor2;

  CANCoderWrapper encoder;

  ProfiledPIDController controller;

  ElevatorFeedforward feedforward;

  public ElevatorRealSubsystem() {
    this.motor1 = new SparkMax(ElevatorConstants.motor1ID, ElevatorConstants.motor1type);
    this.motor1.configure(ElevatorConstants.motor1Config, MotorConstants.resetMode, MotorConstants.persistMode);

    this.motor2 = new SparkMax(ElevatorConstants.motor2ID, ElevatorConstants.motor2type);
    this.motor1.configure(ElevatorConstants.motor2Config, MotorConstants.resetMode, MotorConstants.persistMode);
    
    this.encoder = new CANCoderWrapper(new CANcoder(ElevatorConstants.encoderID));

    this.encoder.setPositionConversionFactor(ElevatorConstants.encoderPositionConversionFactor);
    this.encoder.setVelocityConversionFactor(ElevatorConstants.encoderVelocityConversionFactor);

    this.controller = new ProfiledPIDController(
      ElevatorConstants.P, 
      ElevatorConstants.I, 
      ElevatorConstants.D, 
      ElevatorConstants.constraints
    );
    
    this.controller.setIZone(ElevatorConstants.IZ);
  
    this.feedforward = new ElevatorFeedforward(
      ElevatorConstants.S, 
      ElevatorConstants.G, 
      ElevatorConstants.V,
      ElevatorConstants.A
    );
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ElevatorSubsystem/Speeds/Motor1", motor1.get());
    Logger.recordOutput("ElevatorSubsystem/Speeds/Motor2", motor2.get());
    
    Logger.recordOutput("ElevatorSubsystem/Voltages/Motor1", motor1.getAppliedOutput());
    Logger.recordOutput("ElevatorSubsystem/Voltages/Motor2", motor2.getAppliedOutput());

    Logger.recordOutput("ElevatorSubsystem/Encoder/Position", encoder.getPosition());
    Logger.recordOutput("ElevatorSubsystem/Encoder/Velocity", encoder.getVelocity());

    Logger.recordOutput("ElevatorSubsystem/Controller/SetpointPosition", controller.getSetpoint().position);
    Logger.recordOutput("ElevatorSubsystem/Controller/SetpointVelocity", controller.getSetpoint().velocity);
    Logger.recordOutput("ElevatorSubsystem/Controller/PositionError", controller.getPositionError());
    Logger.recordOutput("ElevatorSubsystem/Controller/VelocityError", controller.getVelocityError());
    Logger.recordOutput("ElevatorSubsystem/Controller/AccumulatedError", controller.getAccumulatedError());
    Logger.recordOutput("ElevatorSubsystem/Controller/AtSetpoint", controller.atSetpoint());
  }

  public void setVoltages(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  public void setElevatorPosition(double desiredPosition) {

    if (desiredPosition > ElevatorConstants.elevatorHeight || desiredPosition < 0) {
      System.err.println("İstenen pozisyon ya çok büyük ya da 0'dan küçük");
    
      return;  
    }

    double currentPosition = encoder.getPosition();

    double output = controller.calculate(currentPosition, desiredPosition);

    output += feedforward.calculate(encoder.getVelocity());

    this.setVoltages(output);
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

  @Override
  public LoggedMechanismLigament2d getLigament() {
    throw new UnsupportedOperationException("Sim dışı kullanılmış metod 'getLigament'");   
  }
}
