package frc.lib.util.RaiderLog;

import java.lang.annotation.*;

@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Logged {

	public enum Importance {
		DEBUG(0),
		INFO(1),
		CRITICAL(2);

		public final int level;

		Importance(int level) {
			this.level = level;
		}
	}

	String name(); // e.g. "speed"

	Importance importance() default Importance.DEBUG;
}
