package frc.robot.utils;

import com.revrobotics.spark.SparkAnalogSensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

// TODO: Bu sınıf doğru tanımlanmış mı bak
public class ThriftyEncoder {
	
  private SparkAnalogSensor analogSensor;

  private boolean inverted;

  private double positionOffset;

	public ThriftyEncoder(SparkAnalogSensor sensor) {
		this.analogSensor = sensor;
		this.inverted = false;
		this.positionOffset = 0.0;
	}

	/**
	 * Returns the current raw position of the absolute encoder.
	 *
	 * @return the current raw position of the absolute encoder in radians.
	 */
	public double getPosition() {
		// ! Özellike bu.
		//average filtreleme uygular değere kendi içinde, amaca göre ya oaky ya yanlış
		return (inverted ? -1.0 : 1.0) * ((analogSensor.getVoltage() / RobotController.getVoltage5V()) * (Math.PI * 2) - Math.PI);
	}

	/**
	 * Inverts the absolute encoder.
	 * 
	 * @param inverted flag indicating if inverted.
	 */
	public void setInverted(boolean inverted) {
		this.inverted = inverted;       
	}

	/**
	 * Sets the position offset between the raw position and the virtual position.
	 * 
	 * @param offset offset in radians
	 */
	public void setPositionOffset(double offset) {
		positionOffset = offset;
	}

	/**
	 * Returns the position offset between the raw position and the virtual position.
	 *
	 * @return the position offset in radians.
	 */
	public double getPositionOffset() {
		return positionOffset;
	}

	/**
	 * Returns the virtual position of the absolute encoder (raw position minus offset).
	 *
	 * @return the virtual position in radians.
	 */
	public double getVirtualPosition() {
		return this.getPosition() - positionOffset;
	}

	/**
	 * Resets the virtual position to the current raw position.
	 */
	public void resetVirtualPosition() {
		positionOffset = this.getPosition();
	}

}