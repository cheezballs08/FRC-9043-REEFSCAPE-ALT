package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MathConsants;
import frc.robot.Constants.ModuleConsants;
import frc.robot.utils.SparkEncoder;
import frc.robot.utils.ThriftyEncoder;

public class SwerveModule {
  SparkMax driveMotor;
  SparkMax angleMotor;
  
  SparkEncoder driveEncoder;
  SparkEncoder angleEncoder;
  
  PIDController driveController;
  PIDController angleController;

  ThriftyEncoder absoluteEncoder;

  public SwerveModule(
    int driveMotorID,
    MotorType driveMotorType,
    boolean invertDriveMotor,
    boolean invertDriveEncoder,

    int angleMotorID,
    MotorType angleMotorType,
    boolean invertAngleMotor,
    boolean invertAngleEncoder,

    int absoluteEncoderID,
    double absoluteEncoderOffset,
    boolean invertAbsoluteEncoder
  ) {

    // TODO: Configure all of the motors here.
    driveMotor = new SparkMax(driveMotorID, driveMotorType);
    
    driveEncoder = new SparkEncoder(driveMotor.getEncoder());
    driveEncoder.setInverted(invertDriveEncoder);
    driveEncoder.setPositionConversionConstant(ModuleConsants.driveEncoderSpeedConversionFactor);
    driveEncoder.setVelocityConversionConstant(ModuleConsants.driveEncoderAcclerationConversionFactor);

    driveController = new PIDController(ModuleConsants.PDrive, ModuleConsants.IDrive, ModuleConsants.DDrive);
    driveController.setIZone(ModuleConsants.IZDrive);
    
    angleMotor = new SparkMax(angleMotorID, angleMotorType);

    angleEncoder = new SparkEncoder(angleMotor.getEncoder());
    angleEncoder.setInverted(invertAngleEncoder);
    angleEncoder.setPositionConversionConstant(ModuleConsants.angleEncoderSpeedConversionFactor);
    angleEncoder.setVelocityConversionConstant(ModuleConsants.angleEncoderAccelerationConversionFactor);
  
    angleController = new PIDController(ModuleConsants.PAngle, ModuleConsants.IAngle, ModuleConsants.DAngle);
    angleController.setIZone(ModuleConsants.IZAngle);
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

    double driveSpeed = driveController.calculate(this.getState().speedMetersPerSecond, desiredState.speedMetersPerSecond) / ModuleConsants.driveMaxSpeed;
    double angleSpeed = angleController.calculate(this.getState().angle.getRadians(), desiredState.angle.getRadians()) / ModuleConsants.angleMaxSpeed;

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
