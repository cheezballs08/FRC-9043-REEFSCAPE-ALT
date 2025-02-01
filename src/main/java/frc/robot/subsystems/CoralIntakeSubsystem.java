package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.utils.CANCoderWrapper;
import frc.robot.utils.PhotoelectricSensor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralIntakeSubsystem extends SubsystemBase {

  SparkMax intakeMotor1, intakeMotor2;

  SparkMax angleMotor;

  CANCoderWrapper angleEncoder;
  
  PIDController angleController;

  PhotoelectricSensor sensor;
  
  public CoralIntakeSubsystem() {
    this.intakeMotor1 = new SparkMax(CoralIntakeConstants.intakeMotor1ID, CoralIntakeConstants.motorType);
    this.intakeMotor1.configure(CoralIntakeConstants.intakeMotor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
    
    this.intakeMotor2 = new SparkMax(CoralIntakeConstants.intakeMotor2ID, CoralIntakeConstants.motorType);
    this.intakeMotor2.configure(CoralIntakeConstants.intakeMotor2Config, MotorConstants.resetMode, MotorConstants.persistMode);
    
    this.angleMotor = new SparkMax(CoralIntakeConstants.angleMotorID, CoralIntakeConstants.motorType);
    this.angleMotor.configure(CoralIntakeConstants.angleMotorConfig, MotorConstants.resetMode, MotorConstants.persistMode);
    

    this.angleEncoder = new CANCoderWrapper(new CANcoder(CoralIntakeConstants.encoderID));
    this.angleEncoder.setPositionConversionFactor(CoralIntakeConstants.positionConversionConstant);
    this.angleEncoder.setVelocityConversionFactor(CoralIntakeConstants.velocityConversionConstant);

    this.angleController = new PIDController(CoralIntakeConstants.P, CoralIntakeConstants.I, CoralIntakeConstants.D);
    this.angleController.setIZone(CoralIntakeConstants.IZ);
  
    this.sensor = new PhotoelectricSensor(CoralIntakeConstants.sensorID);
  
  }

  @Override
  public void periodic() {}

  public void setSpeeds(double speed) {
    this.intakeMotor1.set(speed);
    this.intakeMotor2.set(speed);
  }

  /** Radian girdili */
  public void setIntakeAngle(double angle) {
    this.angleMotor.set(this.angleController.calculate(this.angleEncoder.getPosition()));
  }

  public boolean getSensorState() {
    return this.sensor.isActivated();
  }

  public Trigger getSensorAsTrigger() {
    return this.sensor.asTrigger();
  }
}
