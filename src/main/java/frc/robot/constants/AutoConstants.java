package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AutoConstants {

  public static final String autoName = "auto";

  public static final double PDrive = 3;
  public static final double IDrive = 0;
  public static final double IZDrive = 0;
  public static final double DDrive = 0;

  public static final double PAngle = 3;
  public static final double IAngle = 0.015;
  public static final double IZAngle = 0.02;
  public static final double DAngle = 0;

  // 1 0.25
  public static final Constraints driveConstraints = new Constraints(10000, 10000);

  public static final Constraints angleConstraints = new Constraints(10000, 10000);

  public static final double distanceTolerance = 0.01;
  public static final double angleTolerance = 0.04;
}
