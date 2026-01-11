package frc.lib.util.RaiderLog;

import dev.doglog.DogLog;
import edu.wpi.first.util.struct.StructSerializable;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RaiderLog extends DogLog {

	/** all possible logging modes */
	public enum LogMode {
		BASIC, // DogLog outputs only
		COMP, // WPILOG for competition, inputs too
		REPLAY // Replay from WPILOG
	}

	/** how important a field is to log */
	public enum Importance {
		DEBUG(0), // for fixing problems only
		INFO(1), // general useful data
		CRITICAL(2); // necessary to operate

		public final int level;

		Importance(int level) {
			this.level = level;
		}
	}

	/** entry class representing a method that returns a value */
	private static final class MethodEntry {
		final Object target;
		final Method method;
		final String key;

		MethodEntry(Object target, Method method, String key) {
			this.target = target;
			this.method = method;
			this.key = key;
		}
	}

	/** entry class representing a field in a class */
	private static final class FieldEntry {
		final Object target;
		final Field field;
		final String key;

		FieldEntry(Object target, Field field, String key) {
			this.target = target;
			this.field = field;
			this.key = key;
		}
	}

	/** converts registered input into AKit log and vice versa */
	private static class RaiderAutoLog implements LoggableInputs {

		public void toLog(LogTable table) {
			for (FieldEntry e : INPUT_ENTRIES) {
				try {
					RaiderLog.putInput(table, e.key, e.field.get(e.target));
				} catch (Exception ex) {
					System.err.println("[Logged] " + e.field + " threw: " + ex.getMessage());
				}
			}
		}

		public void fromLog(LogTable table) {
			for (FieldEntry e : INPUT_ENTRIES) {
				try {
					e.field.set(e.target, RaiderLog.getInput(table, e.key, e.field.get(e.target)));
				} catch (Exception ex) {
					System.err.println("[Logged] " + e.field + " threw: " + ex.getMessage());
				}
			}
		}
	}

	private static final List<MethodEntry> METHOD_ENTRIES = new CopyOnWriteArrayList<>();
	private static final List<FieldEntry> FIELD_ENTRIES = new CopyOnWriteArrayList<>();
	private static final List<FieldEntry> INPUT_ENTRIES = new CopyOnWriteArrayList<>();
	private static final RaiderAutoLog autoLog = new RaiderAutoLog();
	private static boolean isInit = false;
	private static Importance minImportance = Importance.DEBUG;
	private static LogMode logMode = LogMode.BASIC;

	/** initializes RaiderLog with specified options - call once in robot constructor */
	public static void init(Importance level, LogMode mode) {
		isInit = true;
		minImportance = level;
		logMode = mode;

		Logger.recordMetadata("ProjectName", "RoboRaiders2025");
		DogLog.setEnabled(logMode != LogMode.COMP);

		switch (logMode) {
			case BASIC -> {
				Logger.addDataReceiver(new NT4Publisher());
			}
			case COMP -> {
				Logger.addDataReceiver(new WPILOGWriter());
			}
			case REPLAY -> {
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new NT4Publisher());
			}
		}

		// AKit disabled for now - too heavy on NT
		// Logger.start();

		System.out.println(
				"[RaiderLog] Initialized with " + minImportance + " importance and " + logMode + " mode.");
	}

	/** call once per object you want to log from (e.g., in subsystem constructor) */
	/** logs every method/field marked with @Logged in that object */
	public static void register(String className, Object obj) {
		if (!isInit) {
			System.err.println(
					"[RaiderLog.register()] RaiderLog not initialized! Call RaiderLog.init() first.");
			return;
		}
		for (Field f : obj.getClass().getFields()) {
			if (f.isAnnotationPresent(Logged.class)) {
				if (!f.canAccess(obj)) f.setAccessible(true);

				Logged ann = f.getAnnotation(Logged.class);
				if (ann.importance().level < minImportance.level) continue;
				String key = "Telemetry/" + className + "/" + ann.key(); // e.g. "swerve/speed"
				FIELD_ENTRIES.add(new FieldEntry(obj, f, key));
			} else if (f.isAnnotationPresent(Input.class)) {
				if (!f.canAccess(obj)) f.setAccessible(true);

				Input ann = f.getAnnotation(Input.class);
				String key = "Telemetry/" + className + "/" + ann.key(); // e.g. "swerve/speed"
				INPUT_ENTRIES.add(new FieldEntry(obj, f, key));
			}
		}
		for (Method m : obj.getClass().getMethods()) {
			if (!m.isAnnotationPresent(Logged.class)) continue;
			if (m.getParameterCount() != 0) {
				System.err.println("[Logged] Skipping " + m + " (must have 0 params).");
				continue;
			}
			// Ensure accessible
			if (!m.canAccess(obj)) m.setAccessible(true);

			Logged ann = m.getAnnotation(Logged.class);
			if (ann.importance().level < minImportance.level) continue;
			String key = "Telemetry/" + className + "/" + ann.key(); // e.g. "swerve/speed"
			METHOD_ENTRIES.add(new MethodEntry(obj, m, key));
		}
	}

	/** update all logged inputs and outputs and push to log; call from robotPeriodic() */
	public static void logAll() {
		if (!isInit) {
			System.err.println(
					"[RaiderLog.logAll()] RaiderLog not initialized! Call RaiderLog.init() first.");
			return;
		}
		if (INPUT_ENTRIES.size() > 0 && logMode == LogMode.COMP) {
			Logger.processInputs("Robot", autoLog);
		}
		for (FieldEntry e : FIELD_ENTRIES) {
			try {
				Object out = e.field.get(e.target);
				RaiderLog.logOutput(e.key, out);
			} catch (Exception ex) {
				// Don't spam—minimal noise in match logs:
				// You can throttle if desired.
				System.err.println("[Logged] " + e.field + " threw: " + ex.getMessage());
			}
		}
		for (MethodEntry e : METHOD_ENTRIES) {
			try {
				Object out = e.method.invoke(e.target);
				RaiderLog.logOutput(e.key, out);
			} catch (Exception ex) {
				// Don't spam—minimal noise in match logs:
				// You can throttle if desired.
				System.err.println("[Logged] " + e.method + " threw: " + ex.getMessage());
			}
		}
	}

	/** log output to DogLog by type */
	private static void logOutput(String key, Object value) {
		if (value instanceof Number) {
			DogLog.log(key, ((Number) value).doubleValue());
		} else if (value instanceof String) {
			DogLog.log(key, (String) value);
		} else if (value instanceof Boolean) {
			DogLog.log(key, (Boolean) value);
		} /*else if (value instanceof Enum<?>) {
			DogLog.log(key, (Enum<?>) value);
		}*/ else if (value instanceof StructSerializable) {
			DogLog.log(key, (StructSerializable) value);
		} else if (value instanceof double[]) {
			DogLog.log(key, (double[]) value);
		} else if (value instanceof int[]) {
			DogLog.log(key, (int[]) value);
		} else if (value instanceof boolean[]) {
			DogLog.log(key, (boolean[]) value);
		} else if (value.getClass().isArray() && Array.get(value, 0) instanceof StructSerializable) {
			// possibly change to log full array instead of each element
			for (int i = 0; i < Array.getLength(value); i++) {
				DogLog.log(key + "[]/" + i, (StructSerializable) Array.get(value, i));
			}
		} else {
			DogLog.log(key, value.toString());
		}
	}

	/** put input values to NT */
	private static void putInput(LogTable table, String key, Object value) {
		if (value instanceof Number) {
			table.put(key, ((Number) value).doubleValue());
		} else if (value instanceof String) {
			table.put(key, (String) value);
		} else if (value instanceof Boolean) {
			table.put(key, (Boolean) value);
		} else if (value instanceof StructSerializable) {
			table.put(key, (StructSerializable) value);
		} else if (value instanceof double[]) {
			table.put(key, (double[]) value);
		} else if (value instanceof int[]) {
			table.put(key, (int[]) value);
		} else if (value instanceof boolean[]) {
			table.put(key, (boolean[]) value);
		} else if (value.getClass().isArray() && Array.get(value, 0) instanceof StructSerializable) {
			for (int i = 0; i < Array.getLength(value); i++) {
				table.put(key + "[]/" + i, ((StructSerializable) Array.get(value, i)));
			}
		} else {
			table.put(key, value.toString());
		}
	}

	/** get input values from NT (for tuning and other adjustable fields) */
	private static Object getInput(LogTable table, String key, Object defaultValue) {
		if (defaultValue instanceof Number) {
			return table.get(key, ((Number) defaultValue).doubleValue());
		} else if (defaultValue instanceof String) {
			return table.get(key, (String) defaultValue);
		} else if (defaultValue instanceof Boolean) {
			return table.get(key, (Boolean) defaultValue);
		} else if (defaultValue instanceof StructSerializable) {
			return table.get(key, (StructSerializable) defaultValue);
		} else if (defaultValue instanceof double[]) {
			return table.get(key, (double[]) defaultValue);
		} else if (defaultValue instanceof int[]) {
			return table.get(key, (int[]) defaultValue);
		} else if (defaultValue instanceof boolean[]) {
			return table.get(key, (boolean[]) defaultValue);
		} else if (defaultValue.getClass().isArray()
				&& Array.get(defaultValue, 0) instanceof StructSerializable) {
			List<StructSerializable> list = new CopyOnWriteArrayList<>();
			for (int i = 0; i < Array.getLength(defaultValue); i++) {
				list.add(table.get(key + "[]/" + i, ((StructSerializable) Array.get(defaultValue, i))));
			}
			return list.toArray();
		} else {
			return table.get(key, defaultValue.toString());
		}
	}
}
