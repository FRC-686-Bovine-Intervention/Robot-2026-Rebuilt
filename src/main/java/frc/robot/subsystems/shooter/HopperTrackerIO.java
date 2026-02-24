package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HopperTrackerIO {
    @AutoLog
    public static class HopperTrackerIOInputs {
        public int ballCount;
        public boolean intakeDeployed;
    }

    public default void updateInputs(HopperTrackerIOInputs inputs) {}

    default void setBallCount(int balls) {}
    default void setIntakeDeployed(boolean deployed) {}
}
