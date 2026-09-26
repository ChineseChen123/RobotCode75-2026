package frc.robot.Constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
	// Standard deviations of odometry/vision pose measurements to supply to Kalman filter
	public static final Matrix<N3, N1> wheelOdometryStdevs =
			MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
	public static final Matrix<N3, N1> visionOdometryStdevs =
			MatBuilder.fill(Nat.N3(), Nat.N1(), 4, 4, 100);

	public static final boolean useFomWeighting = false;

	public static final String topLeftLLName = "limelight-bright";
	public static final String topRightLLName = "limelight-tright";

	/*
	 * x - forward positive
	 * y - right positive
	 * z - up positive
	 * pitch - up positive
	 * yaw - counterclockwise positive
	 */

	public static final Pose3d topLeftLLPose =
			new Pose3d(
					Units.inchesToMeters(12.1761),
					Units.inchesToMeters(-11.8696),
					Units.inchesToMeters(8.27),
					new Rotation3d(
							Units.degreesToRadians(0), Units.degreesToRadians(18), Units.degreesToRadians(42.3)));

	public static final Pose3d topRightLLPose =
			new Pose3d(
					Units.inchesToMeters(12.1761),
					Units.inchesToMeters(11.8696),
					Units.inchesToMeters(8.27),
					new Rotation3d(
							Units.degreesToRadians(0),
							Units.degreesToRadians(18),
							Units.degreesToRadians(-42.3)));

	public static final int[] validMT2Tags = {
		1, 2, 5, 7, 8, 9, 10, 12, 15, 16, 18, 21, 23, 24, 25, 26, 27, 28, 31, 32
	};

	// Thresholds for filtering out vision odometry estimates
	public static final double maxTagDistanceThreshold = 1.5;
	public static final double minAmbiguityThreshold = 0.1;
}
