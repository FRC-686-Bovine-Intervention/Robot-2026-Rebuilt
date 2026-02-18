package frc.robot.automations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.util.EdgeDetector;
import frc.util.controllers.Joystick.Axis;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class BumpMitigation implements Runnable {
	private final Drive drive;
	private final Axis turnAxis;
	private final Command command;

	private static final LoggedTunable<double[]> targetAnglesTunable = new LoggedTunable<>() {
		private final LoggedTunable<Angle> targetAngleOffset = LoggedTunable.from("Automations/Bump Mitigation/Target Angle", Degrees::of, 15.0);

		private double[] cache = this.calculateTargetAngles(this.targetAngleOffset.get().in(Radians));

		@Override
		public boolean hasChanged(int id) {
			return this.targetAngleOffset.hasChanged(id);
		}

		@Override
		public double[] get() {
			if (this.targetAngleOffset.hasChanged(this.hashCode())) {
				this.cache = this.calculateTargetAngles(this.targetAngleOffset.get().in(Radians));
			}
			return this.cache;
		}

		private double[] calculateTargetAngles(double offset) {
			return new double[] {
				0 - offset,
				0 + offset,
				Math.PI/2 - offset,
				Math.PI/2 + offset,
				Math.PI - offset,
				Math.PI + offset,
				-Math.PI/2 - offset,
				-Math.PI/2 + offset,
			};
		}
	};
	private static final LoggedTunable<Time> lookahead = LoggedTunable.from("Automations/Bump Mitigation/Lookahead Time", Seconds::of, 0.5);

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public BumpMitigation(Drive drive, Axis turnAxis) {
		this.drive = drive;
		this.turnAxis = turnAxis;
		this.command = this.drive.rotationalSubsystem.pidControlledHeading(() -> {
			var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
			var robotRotation = robotPose.getRotation().getRadians();

			var targetAngles = targetAnglesTunable.get();

			double targetAngleRads = 0;
			double lowestOffset = Math.PI;
			for (double targetAngleCandidate : targetAngles) {
				double candidateOffset = Math.abs(targetAngleCandidate - robotRotation);
				if (candidateOffset < lowestOffset) {
					targetAngleRads = targetAngleCandidate;
					lowestOffset = candidateOffset;
				}
			}

			return new Rotation2d(targetAngleRads);
		});
	}

	@Override
	public void run() {
		var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
		var lookaheadTrans = robotPose.getTranslation().plus(new Translation2d(
			this.drive.getFieldMeasuredSpeeds().vxMetersPerSecond * lookahead.get().in(Seconds),
			this.drive.getFieldMeasuredSpeeds().vyMetersPerSecond * lookahead.get().in(Seconds)
		));
		this.edgeDetector.update(
			(
				FieldConstants.anyBump.getOurs().withinBounds(robotPose.getTranslation())
				|| FieldConstants.anyBump.getOurs().withinBounds(lookaheadTrans)
				|| FieldConstants.anyBump.getTheirs().withinBounds(robotPose.getTranslation())
				|| FieldConstants.anyBump.getTheirs().withinBounds(lookaheadTrans)
			)
			&& this.turnAxis.getAsDouble() == 0.0
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
