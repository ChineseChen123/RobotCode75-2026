package frc.lib.util.RaiderLog;

import frc.lib.util.RaiderLog.RaiderLog.Importance;
import java.lang.annotation.*;

/**
 * Example usage: @Logged(key = "speed", importance = Importance.INFO) private double speed = 0.0;
 */
@Target({ElementType.METHOD, ElementType.FIELD})
@Retention(RetentionPolicy.RUNTIME)
public @interface Logged {
	String key(); // e.g. "speed"

	Importance importance() default Importance.DEBUG;
}
