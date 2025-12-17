package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPose {

	/** set of all possible field elements */
	/** T and B = top and bottom in Choreo view, L and R respectively from driver view */
	/** A = reef face closest to driver, B-F incrementally clockwise */
	public enum FieldElement {}

	/** returns enum associated with string */
	public static FieldElement fromString(String reefPoint) {
		switch (reefPoint.toUpperCase()) {
			default:
				return null;
		}
	}

	/** generic offset for auto align (e.g. left/right branch or reef algae) */
	public enum Offset {
		LEFT,
		MID,
		RIGHT
	}

	/** 3 required components of a FieldPose */
	public Alliance alliance;

	public FieldElement fieldElement;
	public Offset offset;

	public FieldPose(Alliance alliance, FieldElement fieldElement, Offset offset) {
		this.alliance = alliance;
		this.fieldElement = fieldElement;
		this.offset = offset;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj instanceof FieldPose) {
			FieldPose other = (FieldPose) obj;
			return this.alliance == other.alliance
					&& this.fieldElement == other.fieldElement
					&& this.offset == other.offset;
		}
		return false;
	}

	@Override
	public String toString() {
		return "FieldPose [alliance="
				+ alliance
				+ ", fieldElement="
				+ fieldElement
				+ ", offset="
				+ offset
				+ "]";
	}
}
