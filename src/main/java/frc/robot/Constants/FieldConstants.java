package frc.robot.Constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FieldPose.FieldElement;
import java.util.Map;

public class FieldConstants {
	public static final Map<Integer, FieldElement> tagIDToFieldElement = Map.ofEntries();

	// Map of field elements and tags on blue field
	public static final Map<FieldElement, Integer> blueTags = Map.ofEntries();

	// Map of field elements and tags on red field
	public static final Map<FieldElement, Integer> redTags = Map.ofEntries();

	public static final Pose3d blueHub =
			new Pose3d(Meters.of(4.626), Meters.of(4.035), Feet.of(6), Rotation3d.kZero);

	public static final Pose3d redHub =
			new Pose3d(Meters.of(11.915), Meters.of(4.035), Feet.of(6), Rotation3d.kZero);

	// TODO: fill values
	public static final Pose2d blueBumpLeft =
			new Pose2d(blueHub.getMeasureX(), Meters.of(5.465127944946289), Rotation2d.kZero);
	public static final Pose2d blueBumpRight =
			new Pose2d(blueHub.getMeasureX(), Meters.of(2.513244390487671), Rotation2d.kZero);
	public static final Pose2d blueCornerDepot =
			new Pose2d(Meters.of(0.828855574131012), Meters.of(7.3211541175842285), Rotation2d.kZero);
	public static final Pose2d redCornerDepot =
			new Pose2d(Meters.of(15.863212585449219), Meters.of(0.7376258969306946), Rotation2d.kZero);
	public static final Pose2d blueCornerOutpost =
			new Pose2d(Meters.of(0.828855574131012), Meters.of(0.7376258969306946), Rotation2d.kZero);
	public static final Pose2d redCornerOutpost =
			new Pose2d(Meters.of(15.863212585449219), Meters.of(7.3211541175842285), Rotation2d.kZero);

	public static final Pose2d redBumpLeft =
			new Pose2d(redHub.getMeasureX(), Meters.of(2.513244390487671), Rotation2d.kZero);
	public static final Pose2d redBumpRight =
			new Pose2d(redHub.getMeasureX(), Meters.of(5.465127944946289), Rotation2d.kZero);

	public static final Distance bumpWidth =
			Feet.of(0); // like not left to right but like forward and back
	public static final Distance bumpLength =
			Feet.of(0); // like not left to right but like forward and back

	// Trench Align
	public static final double trenchY = 0.6683171391487122;

	public static final Pose2d blueTrenchLeft =
			new Pose2d(blueHub.getMeasureX(), Meters.of(8.02 - trenchY), Rotation2d.kZero);
	public static final Pose2d blueTrenchRight =
			new Pose2d(blueHub.getMeasureX(), Meters.of(trenchY), Rotation2d.kZero);
	public static final Pose2d redTrenchLeft =
			new Pose2d(redHub.getMeasureX(), Meters.of(trenchY), Rotation2d.kZero);
	public static final Pose2d redTrenchRight =
			new Pose2d(redHub.getMeasureX(), Meters.of(8.02 - trenchY), Rotation2d.kZero);
}
