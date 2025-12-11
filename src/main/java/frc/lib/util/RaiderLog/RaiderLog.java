package frc.lib.util.RaiderLog;

import dev.doglog.DogLog;
import edu.wpi.first.util.struct.StructSerializable;
import frc.lib.util.RaiderLog.Logged.Importance;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public class RaiderLog extends DogLog {

	private static final class Entry {
		final Object target;
		final Method method;
		final String dogName;

		Entry(Object target, Method method, String dogName) {
			this.target = target;
			this.method = method;
			this.dogName = dogName;
		}
	}

	private static final List<Entry> ENTRIES = new CopyOnWriteArrayList<>();
	private static Importance minImportance = Importance.DEBUG;

	public static void setMinImportance(Importance level) {
		minImportance = level;
	}

	/** Call this once per object you want to log from (e.g., in subsystem ctor). */
	public static void register(String name, Object obj) {
		for (Field f : obj.getClass().getFields()) {
			// TODO log fields
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
			String dogName = "Telemetry/" + name + "/" + ann.name(); // e.g. "swerve/speed"
			ENTRIES.add(new Entry(obj, m, dogName));
		}
	}

	/** Invoke all logged methods and push to DogLog. Call from robotPeriodic(). */
	public static void logAll() {
		for (Entry e : ENTRIES) {
			try {
				Object out = e.method.invoke(e.target);
				RaiderLog.log(e.dogName, out);
			} catch (Exception ex) {
				// Don't spam—minimal noise in match logs:
				// You can throttle if desired.
				System.err.println("[Logged] " + e.method + " threw: " + ex.getMessage());
			}
		}
	}

	public static void log(String key, Object value) {
		if (value == null) {
			DogLog.log(key, "null");
			return;
		}
		if (value instanceof Number) {
			DogLog.log(key, ((Number) value).doubleValue());
		} else if (value instanceof String) {
			DogLog.log(key, (String) value);
		} else if (value instanceof Boolean) {
			DogLog.log(key, (Boolean) value);
		} else if (value instanceof Enum<?>) {
			DogLog.log(key, (Enum<?>) value);
		} else if (value instanceof StructSerializable) {
			DogLog.log(key, (StructSerializable) value);
		} else if (value instanceof double[]) {
			DogLog.log(key, (double[]) value);
		} else if (value instanceof int[]) {
			DogLog.log(key, (int[]) value);
		} else if (value instanceof boolean[]) {
			DogLog.log(key, (boolean[]) value);
		} else if (value.getClass().isArray() && Array.get(value, 0) instanceof StructSerializable) {
			DogLog.log(key, (StructSerializable[]) value);
			// for (int i = 0; i < Array.getLength(value); i++) {
			// 	Object elem = Array.get(value, i);
			// 	DogLog.log(key + "/" + i, (StructSerializable) elem);
			// }
		} else {
			DogLog.log(key, value.toString());
		}
	}
}
