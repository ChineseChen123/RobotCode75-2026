package frc.robot.Constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FieldPose.FieldElement;
import java.util.Map;

public class FieldConstants {
	public static final Map<Integer, FieldElement> tagIDToFieldElement = Map.ofEntries();

	// Map of field elements and tags on blue field
	public static final Map<FieldElement, Integer> blueTags = Map.ofEntries();

	// Map of field elements and tags on red field
	public static final Map<FieldElement, Integer> redTags = Map.ofEntries();

	public static final Pose2d blueHub =
			new Pose2d(Meters.of(4.626), Meters.of(4.035), Rotation2d.kZero);
	public static final Pose2d redHub =
			new Pose2d(Meters.of(11.915), Meters.of(4.035), Rotation2d.kZero);

	public static final Distance hubEntranceHeight = Feet.of(6);


	// TODO: fill values
	public static final Pose2d blueBumpLeft = 
			new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero);
	public static final Pose2d blueBumpRight= 
			new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero);

	public static final Pose2d redBumpLeft = 
			new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero);
	public static final Pose2d redBumpRight = 
			new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero);

	public static final Distance bumpWidth = Feet.of(0); // like not left to right but like forward and back
	public static final Distance bumpLength = Feet.of(0); // like not left to right but like forward and back


}
