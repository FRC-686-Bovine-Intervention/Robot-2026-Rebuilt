package frc.util;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class LoggedTracer {
	private static final boolean enabled = true;

	private static double[] startTimes = new double[1];

	public static void reset() {
		if (!enabled) {return;}
		var now = Timer.getFPGATimestamp();
		Arrays.fill(startTimes, now);
	}

	public static void resetFor(String epochName) {
		if (!enabled) {return;}
		var now = Timer.getFPGATimestamp();
		var epochPath = epochName.split("/");
		for (int i = epochPath.length - 1; i < startTimes.length; i++) {
			startTimes[i] = now;
		}
	}

	public static void logEpoch(String epochName) {
		if (!enabled) {return;}
		var now = Timer.getFPGATimestamp();
		var epochPath = epochName.split("/");

		if (epochPath.length > startTimes.length) {
			var newArray = Arrays.copyOf(startTimes, epochPath.length);
			for (int i = startTimes.length; i < newArray.length; i++) {
				newArray[i] = startTimes[startTimes.length - 1];
			}
			startTimes = newArray;
		}

		var lastTime = startTimes[epochPath.length - 1];
		for (int i = epochPath.length - 1; i < startTimes.length; i++) {
			startTimes[i] = now;
		}

		Logger.recordOutput("LoggedTracer/" + epochName, (now - lastTime) * 1000.0);
	}
}
