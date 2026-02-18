package frc.robot.subsystems.rollers.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
	@AutoLog
	public static class IndexerIOInputs {

	}

	public default void updateInputs(IndexerIOInputs inputs) {}
}
