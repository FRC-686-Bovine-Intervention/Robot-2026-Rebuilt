package frc.robot.auto.routines;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.util.flipping.AllianceFlipped;

public class ResetPosition extends AutoRoutine {

	private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<>("Start Position") {
		private final Map.Entry<String, AllianceFlipped<Pose2d>> startCenter = AutoQuestion.Settings.option("Center", AutoConstants.startCenter);
		private final Map.Entry<String, AllianceFlipped<Pose2d>> startInLeftTrench = AutoQuestion.Settings.option("In L Trench", AutoConstants.startInsideLeftTrench);
		private final Map.Entry<String, AllianceFlipped<Pose2d>> startInRightTrench = AutoQuestion.Settings.option("In R Trench", AutoConstants.startInsideRightTrench);

		@Override
		protected AutoQuestion.Settings<AllianceFlipped<Pose2d>> generateSettings() {
			return AutoQuestion.Settings.from(this.startCenter, this.startInLeftTrench, this.startCenter, this.startInRightTrench);
		}
	};

	public ResetPosition() {
		super(
			"Reset Position",
			List.of(
				startPosition
			)
		);
	}

	@Override
	public Command generateCommand() {
		final var startPosition = ResetPosition.startPosition.getResponse();
		return AutoCommons.setOdometryFlipped(startPosition);
	}
}
