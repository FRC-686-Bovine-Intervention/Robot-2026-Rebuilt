package frc.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a subsystem unit that requires a periodic callback but not require a hardware mutex.
 */
public abstract class VirtualSubsystem {
	private static List<VirtualSubsystem> subsystems = new ArrayList<>();

	public VirtualSubsystem() {
		subsystems.add(this);
	}

	/** Calls {@link #periodic()} on all virtual subsystems. */
	public static void periodicAll() {
		for (var subsystem : subsystems) {
			subsystem.periodic();
		}
	}
	public static void postCommandPeriodicAll() {
		for (var subsystem : subsystems) {
			subsystem.postCommandPeriodic();
		}
	}

	/** This method is called periodically once per loop cycle. */
	public abstract void periodic();
	/** This method is called periodically once per loop cycle after all commands execute. */
	public void postCommandPeriodic() {}
}
