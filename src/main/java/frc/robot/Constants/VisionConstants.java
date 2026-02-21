package frc.robot.Constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
	// Standard deviations of odometry/vision pose measurements to supply to Kalman filter
	public static final Matrix<N3, N1> wheelOdometryStdevs =
			MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
	public static final Matrix<N3, N1> visionOdometryStdevs =
			MatBuilder.fill(Nat.N3(), Nat.N1(), 5, 5, 100);

	public static final boolean useFomWeighting = false;

	public static final String topLeftLLName = "bright";
	public static final String topRightLLName = "tright";

	public static final Pose3d topLeftLLPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
	public static final Pose3d topRightLLPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

	public static final int[] validMT2Tags = {
		2, 5, 7, 8, 9, 10, 12, 15, 16, 18, 21, 23, 24, 25, 26, 27, 28, 31, 32
	};

	// Thresholds for filtering out vision odometry estimates
	public static final double minTagAreaThreshold = 0.15;
	public static final double maxTagDistanceThreshold = 1.5;
	public static final double ambiguityThreshold = 0.15;
	public static final double multiTagAmbiguityThreshold = 0.3;
	public static final double reprojectionErrorThreshold = 0.5;
}
