package frc.robot.Constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
	public static final Matrix<N3, N1> moduleMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
	public static final Matrix<N3, N1> visionMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 5, 5, 100);

	public static final double maxTimeUntilFallbackToOdometry = 1.0;
	public static final Transform3d LeftFacingCameraPose =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(8.966), // X 9.325
							Units.inchesToMeters(7.951), // Y .3505
							Units.inchesToMeters(8.25)), // Z 5.85
					new Rotation3d(
							Units.degreesToRadians(0),
							Units.degreesToRadians(-20),
							Units.degreesToRadians(40))); // 0 0 0
	//   new Transform3d(
	//       new Translation3d(
	//           Units.inchesToMeters(10.382), // X 9.325
	//           Units.inchesToMeters(-11.941), // Y .3505
	//           Units.inchesToMeters(8.419)), // Z 5.85
	//       new Rotation3d(
	//           Units.degreesToRadians(0),
	//           Units.degreesToRadians(-20),
	//           Units.degreesToRadians(30))); // 0 0 0
	public static final Transform3d RightFacingCameraPose =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(7.693), // X 7.262
							Units.inchesToMeters(10.815), // Y 10.201
							Units.inchesToMeters(8.25)), // Z 6.638
					new Rotation3d(
							Units.degreesToRadians(0),
							Units.degreesToRadians(-20),
							Units.degreesToRadians(-20))); // 0 0 0
	public static final Transform3d HPCameraPose =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(0.863),
							Units.inchesToMeters(-8.367),
							Units.inchesToMeters(40.759)),
					new Rotation3d(
							Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(125)));
	public static final Transform3d CageDetectCameraPose =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(.863),
							Units.inchesToMeters(-7.387),
							Units.inchesToMeters(34.334)),
					new Rotation3d(
							Units.degreesToRadians(0), Units.degreesToRadians(160), Units.degreesToRadians(160)));
	public static final Transform3d BranchCameraPose =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
					new Rotation3d(
							Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
	// todo: check values at comp
	public static final double finalYawSetpointLeft = 13.8;
	public static final double finalPitchSetpointLeft = -3.37;
	public static final double finalYawSetpointRight = -7.97;
	public static final double finalPitchSetpointRight = -7.88;

	public static final double minTagAreaThreshold = 0.15;
	public static final double maxTagDistanceThreshold = 1.5;
}
