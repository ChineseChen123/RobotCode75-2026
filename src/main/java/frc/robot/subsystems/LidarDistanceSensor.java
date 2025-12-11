package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.EndEffectorConstants.algaeLidarSensorPort;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class LidarDistanceSensor {

	private double threshold = 0;
	DutyCycle sensorCycle;

	public LidarDistanceSensor(Distance threshold) {
		sensorCycle = new DutyCycle(new DigitalInput(algaeLidarSensorPort));
		this.threshold = threshold.in(Inches);
	}

	private double getTimeNanoSeconds() {
		return sensorCycle.getHighTimeNanoseconds();
	}

	private double getDistanceMM() {
		// magic numbers to get the distance
		return 4 * (getTimeNanoSeconds() - 1e6) / 1000;
	}

	public double getDistanceIN() {
		return Units.metersToInches(getDistanceMM() / 1000.0);
	}

	public boolean belowThreshold() {
		return getDistanceIN() <= threshold && getDistanceIN() >= 0;
	}
}
