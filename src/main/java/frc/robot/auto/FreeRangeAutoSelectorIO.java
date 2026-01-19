package frc.robot.auto;

import org.littletonrobotics.junction.AutoLog;

public interface FreeRangeAutoSelectorIO {
    @AutoLog
    public static class FreeRangeAutoSelectorIOInputs {
        public boolean shouldKeepOut = true;
        public boolean shouldClimb = true;

        public double[] keepOutTopLeft = new double[0];
        public double[] keepOutTopRight = new double[0];
    }

    public default void updateInputs(FreeRangeAutoSelectorIOInputs inputs) {}
}
