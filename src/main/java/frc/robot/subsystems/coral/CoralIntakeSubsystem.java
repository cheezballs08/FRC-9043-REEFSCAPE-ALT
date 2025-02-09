package frc.robot.subsystems.coral;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
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
  public void periodic() {
    DogLog.log("AlgeaIntake/Speeds/IntakeMotor1", intakeMotor1.get());
    DogLog.log("AlgeaIntake/Speeds/IntakeMotor2", intakeMotor2.get());
    DogLog.log("AlgeaIntake/Speeds/AngleMotor", angleMotor.get());
    
    DogLog.log("AlgeaIntake/Voltages/IntakeMotor1", intakeMotor1.getAppliedOutput());
    DogLog.log("AlgeaIntake/Voltages/IntakeMotor2", intakeMotor2.getAppliedOutput());
    DogLog.log("AlgeaIntake/Voltages/AngleMotor", angleMotor.getAppliedOutput());

    DogLog.log("AlgeaIntake/Encoder/Position", angleEncoder.getPosition());
    DogLog.log("AlgeaIntake/Encoder/Velocity", angleEncoder.getVelocity());

    DogLog.log("AlgeaIntake/Controller/SetpointPosition", angleController.getSetpoint());
    DogLog.log("AlgeaIntake/Controller/PositionError", angleController.getPositionError());
    DogLog.log("AlgeaIntake/Controller/VelocityError", angleController.getVelocityError());
    DogLog.log("AlgeaIntake/Controller/AccumulatedError", angleController.getAccumulatedError());
    DogLog.log("AlgeaIntake/Controller/AtSetpoint", angleController.atSetpoint());

    DogLog.log("AlgeaIntake/Sensor", this.isSensorActive());
  }

  public void setSpeeds(double speed) {
    this.intakeMotor1.set(speed);
    this.intakeMotor2.set(speed);
  }

  /** Radian girdili */
  public void setAngle(double angle) {
    this.angleMotor.set(this.angleController.calculate(this.angleEncoder.getPosition(), angle));
  }

  public void setPosition(double speed, double angle) {
    this.setSpeeds(speed);
    this.setAngle(angle);
  }

  public boolean isAtSetpoint() {
    return this.angleController.atSetpoint();
  }

  public boolean isSensorActive() {
    return this.sensor.isActivated();
  }

  public Trigger getSensorAsTrigger() {
    return this.sensor.asTrigger();
  }
}
