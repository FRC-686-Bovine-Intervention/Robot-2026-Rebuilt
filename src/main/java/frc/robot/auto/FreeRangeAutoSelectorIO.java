package frc.robot.auto;

import org.littletonrobotics.junction.AutoLog;

public interface FreeRangeAutoSelectorIO {
	@AutoLog
	public static class FreeRangeAutoSelectorIOInputs {
		public boolean shouldKeepOut = true;
		public boolean shouldClimb = true;
		public boolean shouldNeutralZone = false;
		public boolean shouldAvoidBump = false;
		public boolean shouldNotAutoIntake = false;
		public boolean shouldStopToShoot = false;

		public double[] keepOutTopLeft = new double[0];
		public double[] keepOutBottomRight = new double[0];
	}

	public default void updateInputs(FreeRangeAutoSelectorIOInputs inputs) {}
}
