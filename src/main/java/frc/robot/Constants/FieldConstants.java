package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// import frc.lib.util.FieldPose.FieldElement;

public class FieldConstants {
	// public static final Map<Integer, FieldElement> tagIDToFieldElement = Map.ofEntries();

	// // Map of field elements and tags on blue field
	// public static final Map<FieldElement, Integer> blueTags = Map.ofEntries();

	// // Map of field elements and tags on red field
	// public static final Map<FieldElement, Integer> redTags = Map.ofEntries();

	public static final Pose2d blueHub = new Pose2d(0, 0, Rotation2d.kZero);
	public static final Pose2d redHub = new Pose2d(12, 4, Rotation2d.kZero);
}
