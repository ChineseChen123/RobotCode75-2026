package frc.lib.dashboard;

import static frc.robot.Constants.RobotConstants.TUNING_MODE;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;

/**
 * Tunable Number takes in a number type and publishes an entry to a topic called "tuning" to be
 * used with advantagescope
 */
public class TunableString {

	private NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private NetworkTable table = instance.getTable("Tuning");
	private StringEntry entry;
	private String defaultValue;

	public TunableString(String path, String string) {
		entry = table.getStringTopic('/' + path).getEntry(string);
		entry.set(string);
		defaultValue = string;
	}

	public String getString() {
		return TUNING_MODE ? entry.get() : defaultValue;
	}
}
