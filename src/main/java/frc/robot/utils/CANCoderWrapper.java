package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;

// TODO: Bu doğru yapılmış mı bak
public class CANCoderWrapper {
  CANcoder encoder;

  boolean inverted;

  double positionConversionConstant;
  double velocityConversionConstant;

  public CANCoderWrapper(CANcoder encoder) {
    this.encoder = encoder;
    this.inverted = false;

    this.positionConversionConstant = 1.0;
    this.velocityConversionConstant = 1.0;
  }

  public double getPosition() {
    // ! Özellike bu.
    return (inverted ? -1.0 : 1.0) * encoder.getPosition().getValueAsDouble() * positionConversionConstant;
  }

  public double getVelocity() {
    // ! Ve bu.
    return (inverted ? -1.0 : 1.0) * encoder.getVelocity().getValueAsDouble() * velocityConversionConstant;
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  public void setPositionConversionConstant(double positionConversionConstant) {
    this.positionConversionConstant = positionConversionConstant;
  }

  public void setVelocityConversionConstant(double velocityConversionConstant) {
    this.velocityConversionConstant = velocityConversionConstant;
  }
}
