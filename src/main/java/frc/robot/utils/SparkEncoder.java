package frc.robot.utils;

import com.revrobotics.RelativeEncoder;

// TODO: Bu doğru yapılmış mı bak
// eğer quadrature encoderi sparka takmak içinse iş görebilir gibi, eğer thrifty encoderleri sparka takmak içinse yanlış çünkü onlar absolute encoder ve analoglar
public class SparkEncoder {
  RelativeEncoder encoder;

  boolean inverted;

  double positionConversionConstant;
  double velocityConversionConstant;

  public SparkEncoder(RelativeEncoder encoder) {
    this.encoder = encoder;
    this.inverted = false;

    this.positionConversionConstant = 1.0;
    this.velocityConversionConstant = 1.0;
  }

  public double getPosition() {
    // ! Özellike bu.
    return (inverted ? -1.0 : 1.0) * encoder.getPosition() * positionConversionConstant;
  }

  public double getVelocity() {
    // ! Ve bu.
    return (inverted ? -1.0 : 1.0) * encoder.getVelocity() * velocityConversionConstant;
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
