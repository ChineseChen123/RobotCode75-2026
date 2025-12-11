package frc.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Conversions {
	/**
	 * @param motorRot Motor Position: (in Rotations)
	 * @param gearRatio Gear Ratio between Motor and Mechanism
	 * @return Mechanism Position: (in Degrees)
	 */
	public static Angle talonToDegrees(Angle motorRot, double gearRatio) {
		Angle mechDeg = Degrees.of(motorRot.in(Degrees) / gearRatio);
		return mechDeg;
	}

	/**
	 * @param degrees Mechanism Position: (in Degrees)
	 * @param gearRatio Gear Ratio between Motor and Mechanism
	 * @return Motor Rotation: (in Rotations)
	 */
	public static Angle degreesToTalon(Angle mechDeg, double gearRatio) {
		Angle motorRotations = Rotations.of(mechDeg.in(Rotations) * gearRatio);
		return motorRotations;
	}

	/**
	 * @param motorRPS Motor Velocity: (in Rotations per Second)
	 * @param circumference Wheel Circumference: (in Meters)
	 * @param gearRatio Gear Ratio between Motor and Mechanism
	 * @return Wheel Velocity: (in Meters per Second)
	 */
	public static LinearVelocity talonToMPS(
			AngularVelocity motorRPS, Distance circumference, double gearRatio) {
		LinearVelocity wheelMPS =
				MetersPerSecond.of(motorRPS.in(RotationsPerSecond) / gearRatio * circumference.in(Meters));
		return wheelMPS;
	}

	/**
	 * @param wheelRPM Wheel Velocity: (in Meters per Second)
	 * @param circumference Wheel Circumference: (in Meters)
	 * @param gearRatio Gear Ratio between Motor and Wheel
	 * @return Motor Velocity: (in Rotations per Second)
	 */
	public static AngularVelocity MPSToTalon(
			LinearVelocity wheelMPS, Distance circumference, double gearRatio) {
		AngularVelocity motorRPS =
				RotationsPerSecond.of(wheelMPS.in(MetersPerSecond) / circumference.in(Meters) * gearRatio);
		return motorRPS;
	}

	/**
	 * @param motorRot Motor Position: (in Rotations)
	 * @param circumference Wheel Circumference: (in Meters)
	 * @param gearRatio Gear Ratio between Motor and Wheel
	 * @return Wheel Distance: (in Meters)
	 */
	public static Distance talonToMeters(Angle motorRot, Distance circumference, double gearRatio) {
		Distance wheelMeters = Meters.of(motorRot.in(Rotations) / gearRatio * circumference.in(Meters));
		return wheelMeters;
	}

	/**
	 * @param wheelMeters Wheel Distance: (in Meters)
	 * @param circumference Wheel Circumference: (in Meters)
	 * @param gearRatio Gear Ratio between Motor and Wheel
	 * @return Motor Position: (in Rotations)
	 */
	public static Angle MetersToTalon(
			Distance wheelMeters, Distance circumference, double gearRatio) {
		Angle motorRotations =
				Rotations.of(wheelMeters.in(Meters) / circumference.in(Meters) * gearRatio);
		return motorRotations;
	}
}
