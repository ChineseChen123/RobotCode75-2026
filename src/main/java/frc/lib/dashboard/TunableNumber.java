package frc.lib.dashboard;

import static frc.robot.Constants.RobotConstants.TUNING_MODE;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Tunable Number takes in a number type and publishes an entry to a topic called "tuning" to be
 * used with advantagescope
 */
public class TunableNumber {
	// TODO: make annotation instead of class

	private NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private NetworkTable table = instance.getTable("Tuning");
	private DoubleEntry entry;
	private double defaultValue;

	public TunableNumber(String path, double defaultVal) {
		entry = table.getDoubleTopic('/' + path).getEntry(defaultVal);
		entry.set(defaultVal);
		defaultValue = defaultVal;
	}

	/** gets and returns current value from NT */
	public double getNumber() {
		return TUNING_MODE ? entry.get() : defaultValue;
	}
}
