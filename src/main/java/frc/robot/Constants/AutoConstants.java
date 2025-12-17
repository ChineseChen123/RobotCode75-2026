package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;

public class AutoConstants {
	// limits during auto slightly higher than normal
	public static final double kMaxSpeed = 5.0;
	public static final double kMaxAcceleration = 3.0;
	public static final double kPXController = 1.0;
	public static final double kPThetaController = 0.5;

	// start poses
	public static final Map<String, Pose2d> blueStartPositions = Map.of();
	public static final Map<String, Pose2d> redStartPositions = Map.of();
}
