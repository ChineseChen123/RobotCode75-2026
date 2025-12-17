package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import frc.lib.util.FieldPose.FieldElement;
import java.util.Map;

public class FieldConstants {
	public static final Map<Integer, FieldElement> tagIDToFieldElement = Map.ofEntries();

	// Map of field elements and tags on blue field
	public static final Map<FieldElement, Integer> blueTags = Map.ofEntries();

	// Map of field elements and tags on red field
	public static final Map<FieldElement, Integer> redTags = Map.ofEntries();
}
