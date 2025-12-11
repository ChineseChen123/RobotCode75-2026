package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map;

public class AutoConstants {
	// limits during auto slightly higher than normal
	public static final double kMaxSpeed = 5.0;
	public static final double kMaxAcceleration = 3.0;
	public static final double kPXController = 1.0;
	public static final double kPThetaController = 0.5;

	// start poses
	public static final Map<String, Pose2d> blueStartPositions =
			Map.of(
					"st", new Pose2d(7.58, 6.82, Rotation2d.fromDegrees(180)),
					"sm", new Pose2d(7.58, 4.02, Rotation2d.fromDegrees(180)),
					"sb", new Pose2d(7.58, 1.23, Rotation2d.fromDegrees(180)));
	public static final Map<String, Pose2d> redStartPositions =
			Map.of(
					"st", new Pose2d(9.96, 1.23, Rotation2d.fromDegrees(0)),
					"sm", new Pose2d(9.96, 4.02, Rotation2d.fromDegrees(0)),
					"sb", new Pose2d(9.96, 6.82, Rotation2d.fromDegrees(0)));
}
