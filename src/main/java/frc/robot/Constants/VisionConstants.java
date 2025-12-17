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

	public static final Transform3d LimelightPose =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(3),
							Units.inchesToMeters(-0.25), // quarter inch left
							Units.inchesToMeters(8.25)), // height from ground
					new Rotation3d(
							Units.degreesToRadians(0),
							Units.degreesToRadians(15), // positive up
							Units.degreesToRadians(0)));

	public static final String llName = "limelight";
	public static final int[] validMT2Tags = {1, 6, 7, 8, 9, 10, 11};

	public static final double minTagAreaThreshold = 0.15;
	public static final double maxTagDistanceThreshold = 1.5;
	public static final double ambiguityThreshold = 0.15;
	public static final double multiTagAmbiguityThreshold = 0.3;
	public static final double reprojectionErrorThreshold = 0.5;
}
