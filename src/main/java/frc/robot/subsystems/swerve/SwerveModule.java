package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MathConsants;
import frc.robot.Constants.ModuleConstants;
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

    int absoluteEncoderID,
    double absoluteEncoderOffset,
    boolean invertAbsoluteEncoder
  ) {

    // ? Im assuming we don't have to burn the flash with persist parameters, I hope Im right
    driveMotor = new SparkMax(driveMotorID, ModuleConstants.driveMotorType);

    driveMotor.configure(driveMotorConfig, ModuleConstants.resetMode, ModuleConstants.persistMode);

    driveEncoder = new SparkEncoder(driveMotor.getEncoder());
    driveEncoder.setInverted(invertDriveEncoder);
    driveEncoder.setPositionConversionConstant(ModuleConstants.driveEncoderSpeedConversionFactor);
    driveEncoder.setVelocityConversionConstant(ModuleConstants.driveEncoderAcclerationConversionFactor);

    driveController = new PIDController(ModuleConstants.PDrive, ModuleConstants.IDrive, ModuleConstants.DDrive);
    driveController.setIZone(ModuleConstants.IZDrive);
    
    angleMotor = new SparkMax(angleMotorID, ModuleConstants.angleMotorType);
    angleMotor.configure(angleMotorConfig, ModuleConstants.resetMode, ModuleConstants.persistMode);

    angleEncoder = new SparkEncoder(angleMotor.getEncoder());
    angleEncoder.setInverted(invertAngleEncoder);
    angleEncoder.setPositionConversionConstant(ModuleConstants.angleEncoderSpeedConversionFactor);
    angleEncoder.setVelocityConversionConstant(ModuleConstants.angleEncoderAccelerationConversionFactor);
  
    angleController = new PIDController(ModuleConstants.PAngle, ModuleConstants.IAngle, ModuleConstants.DAngle);
    angleController.setIZone(ModuleConstants.IZAngle);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    absoluteEncoder = new ThriftyEncoder(absoluteEncoderID);
    absoluteEncoder.setInverted(invertAbsoluteEncoder);
    absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoder.getVelocity(),
      new Rotation2d(angleEncoder.getPosition())
    );
  } 

  public void setDesiredState(SwerveModuleState desiredState) {
   
    // TODO: Check if only checking the speed will give the same result.
    if (Math.abs(desiredState.speedMetersPerSecond) < MathConsants.epsilon && Math.abs(desiredState.angle.getRadians()) < MathConsants.epsilon) {
      stop();
      return;
    }

    desiredState.optimize(this.getState().angle);

    double driveSpeed = driveController.calculate(this.getState().speedMetersPerSecond, desiredState.speedMetersPerSecond) / ModuleConstants.driveMaxSpeed;
    double angleSpeed = angleController.calculate(this.getState().angle.getRadians(), desiredState.angle.getRadians()) / ModuleConstants.angleMaxSpeed;

    driveMotor.set(driveSpeed);
    angleMotor.set(angleSpeed);
  }

  // TODO: Check if this is correct.
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
