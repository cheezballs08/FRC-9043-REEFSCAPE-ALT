package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.utils.CANCoderWrapper;
import frc.robot.utils.PhotoelectricSensor;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralIntakeRealSubsystem extends SubsystemBase implements CoralIntakeSubsystem {

  SparkMax intakeMotor1, intakeMotor2;

  SparkMax angleMotor;

  CANCoderWrapper angleEncoder;
  
  ProfiledPIDController angleController;

  ArmFeedforward feedforward;

  PhotoelectricSensor sensor;
  
  public CoralIntakeRealSubsystem() {
    this.intakeMotor1 = new SparkMax(CoralIntakeConstants.intakeMotor1ID, CoralIntakeConstants.motorType);
    this.intakeMotor1.configure(CoralIntakeConstants.intakeMotor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
    
    this.intakeMotor2 = new SparkMax(CoralIntakeConstants.intakeMotor2ID, CoralIntakeConstants.motorType);
    this.intakeMotor2.configure(CoralIntakeConstants.intakeMotor2Config, MotorConstants.resetMode, MotorConstants.persistMode);
    
    this.angleMotor = new SparkMax(CoralIntakeConstants.angleMotorID, CoralIntakeConstants.motorType);
    this.angleMotor.configure(CoralIntakeConstants.angleMotorConfig, MotorConstants.resetMode, MotorConstants.persistMode);

    this.angleEncoder = new CANCoderWrapper(new CANcoder(CoralIntakeConstants.encoderID));
    this.angleEncoder.setPositionConversionFactor(CoralIntakeConstants.positionConversionConstant);
    this.angleEncoder.setVelocityConversionFactor(CoralIntakeConstants.velocityConversionConstant);

    this.angleController = new ProfiledPIDController(CoralIntakeConstants.P, CoralIntakeConstants.I, CoralIntakeConstants.D, CoralIntakeConstants.constraints);
    this.angleController.setIZone(CoralIntakeConstants.IZ);
  
    this.feedforward = new ArmFeedforward(
      CoralIntakeConstants.S, 
      CoralIntakeConstants.G,
      CoralIntakeConstants.V,
      CoralIntakeConstants.A 
    );

    this.sensor = new PhotoelectricSensor(CoralIntakeConstants.sensorID);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("CoralIntake/Speeds/IntakeMotor1", intakeMotor1.get());
    Logger.recordOutput("CoralIntake/Speeds/IntakeMotor2", intakeMotor2.get());
    Logger.recordOutput("CoralIntake/Speeds/AngleMotor", angleMotor.get());
    
    Logger.recordOutput("CoralIntake/Voltages/IntakeMotor1", intakeMotor1.getAppliedOutput());
    Logger.recordOutput("CoralIntake/Voltages/IntakeMotor2", intakeMotor2.getAppliedOutput());
    Logger.recordOutput("CoralIntake/Voltages/AngleMotor", angleMotor.getAppliedOutput());

    Logger.recordOutput("CoralIntake/Encoder/Position", angleEncoder.getPosition());
    Logger.recordOutput("CoralIntake/Encoder/Velocity", angleEncoder.getVelocity());

    Logger.recordOutput("CoralIntake/Controller/SetpointPosition", angleController.getSetpoint().position);
    Logger.recordOutput("CoralIntake/Controller/SetpointVelocity", angleController.getSetpoint().velocity);
    Logger.recordOutput("CoralIntake/Controller/PositionError", angleController.getPositionError());
    Logger.recordOutput("CoralIntake/Controller/VelocityError", angleController.getVelocityError());
    Logger.recordOutput("CoralIntake/Controller/AccumulatedError", angleController.getAccumulatedError());
    Logger.recordOutput("CoralIntake/Controller/AtSetpoint", angleController.atSetpoint());

    Logger.recordOutput("CoralIntake/Sensor", this.isSensorActive());
  }

  public void setSpeeds(double speed) {
    this.intakeMotor1.set(speed);
    this.intakeMotor2.set(speed);
  }

  /** Radian girdili */
  public void setAngle(double angle) {

    double position = this.angleEncoder.getPosition();
    
    double output = this.angleController.calculate(position, angle);

    output += this.feedforward.calculate(position, this.angleEncoder.getVelocity());

    this.angleMotor.set(output);
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

  public LoggedMechanismLigament2d getLigament() {
      throw new UnsupportedOperationException("Sim dışı kullanılan metod 'getLigament'");
  }

  @Override
  public Pose3d getPose() {
    throw new UnsupportedOperationException("Sim dışı kullanılan metod 'getPose'");
  }
}
