package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.MathConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.utils.SparkEncoder;
import frc.robot.utils.ThriftyEncoder;

public class SwerveModule {
  private SparkMax driveMotor;
  private SparkMax angleMotor;
  
  private SparkEncoder driveEncoder;
  private SparkEncoder angleEncoder;
  
  private PIDController driveController;
  private PIDController angleController;

  private ThriftyEncoder absoluteEncoder;

  public SwerveModule(
    int driveMotorID,
    SparkBaseConfig driveMotorConfig,
    boolean invertDriveEncoder,

    int angleMotorID,
    SparkBaseConfig angleMotorConfig,
    boolean invertAngleEncoder,

    double absoluteEncoderOffset,
    boolean invertAbsoluteEncoder
  ) {

    // ? Im assuming we don't have to burn the flash with persist parameters, I hope Im right
    driveMotor = new SparkMax(driveMotorID, ModuleConstants.driveMotorType);

    driveMotor.configure(driveMotorConfig, MotorConstants.resetMode, MotorConstants.persistMode);

    driveEncoder = new SparkEncoder(driveMotor.getEncoder());
    driveEncoder.setInverted(invertDriveEncoder);
    driveEncoder.setPositionConversionFactor(ModuleConstants.driveEncoderPositionConversionFactor);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.driveEncoderSpeedConversionFactor);

    driveController = new PIDController(ModuleConstants.PDrive, ModuleConstants.IDrive, ModuleConstants.DDrive);
    driveController.setIZone(ModuleConstants.IZDrive);
    
    angleMotor = new SparkMax(angleMotorID, ModuleConstants.angleMotorType);
    angleMotor.configure(angleMotorConfig, MotorConstants.resetMode, MotorConstants.persistMode);

    angleEncoder = new SparkEncoder(angleMotor.getEncoder());
    angleEncoder.setInverted(invertAngleEncoder);
    angleEncoder.setPositionConversionFactor(ModuleConstants.angleEncoderPositionConversionFactor);
    angleEncoder.setVelocityConversionFactor(ModuleConstants.angleEncoderSpeedConversionFactor);
  
    angleController = new PIDController(ModuleConstants.PAngle, ModuleConstants.IAngle, ModuleConstants.DAngle);
    angleController.setIZone(ModuleConstants.IZAngle);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Encoder'in alındığı motoru duruma göre değiştir. 
    absoluteEncoder = new ThriftyEncoder(angleMotor.getAnalog());
    absoluteEncoder.setInverted(invertAbsoluteEncoder);
    absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
    absoluteEncoder.setPositionConversionFactor(ModuleConstants.mangeticEncoderPositionConversionFactor);
    absoluteEncoder.setVelocityConversionFactor(ModuleConstants.mangeticEncoderSpeedConversionFactor);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoder.getVelocity(),
      new Rotation2d(angleEncoder.getPosition())
    );
  } 

  public void setDesiredState(SwerveModuleState desiredState) {
   
    // TODO: Sadece hıza bakmanın yetip yetmeyeceğine bak. Önemsiz ama neyse.
    if (Math.abs(desiredState.speedMetersPerSecond) < MathConstants.epsilon && Math.abs(desiredState.angle.getRadians()) < MathConstants.epsilon) {
      stop();
      return;
    }

    desiredState.optimize(this.getState().angle);

    double driveSpeed = driveController.calculate(this.getState().speedMetersPerSecond, desiredState.speedMetersPerSecond) / ModuleConstants.driveMaxSpeed;
    double angleSpeed = angleController.calculate(this.getState().angle.getRadians(), desiredState.angle.getRadians()) / ModuleConstants.angleMaxSpeed;

    driveMotor.set(driveSpeed);
    angleMotor.set(angleSpeed);
  }

  // TODO: Doğru mu değil mi bak.
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      new Rotation2d(angleEncoder.getPosition())
    );
  }

  public void stop() {
    driveMotor.set(0);
    angleMotor.set(0);
  }


}
