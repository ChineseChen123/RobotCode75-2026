package frc.lib.util.RaiderLog;

public enum LogField {
	Swerve("swerve"),
	Shooter("shooter"),
// ...
;

	public final String key;

	LogField(String key) {
		this.key = key;
	}
}
