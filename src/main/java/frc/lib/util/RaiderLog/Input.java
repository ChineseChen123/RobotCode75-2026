package frc.lib.util.RaiderLog;

import frc.lib.util.RaiderLog.RaiderLog.Importance;
import java.lang.annotation.*;

/** Example usage:
 * @Input(key = "joystickX", importance = Importance.INFO)
 * private double joystickX = 0.0;
 */

@Target(ElementType.FIELD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Input {
	String key(); // e.g. "speed"

	Importance importance() default Importance.DEBUG;
}
