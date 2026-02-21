package frc.robot.automations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

	private final double[] staticBoxTopBlue = new double[] {
		FieldConstants.bumpInnerX.getBlue().in(Meters),
		FieldConstants.bumpOuterX.getBlue().in(Meters),
		FieldConstants.topBumpBottomY.in(Meters),
		FieldConstants.topBumpTopY.in(Meters)
	};

	private final double[] staticBoxBottomBlue = new double[] {
		FieldConstants.bumpInnerX.getBlue().in(Meters),
		FieldConstants.bumpOuterX.getBlue().in(Meters),
		FieldConstants.bottomBumpTopY.in(Meters),
		FieldConstants.bottomBumpBottomY.in(Meters)
	};

	private final double[] staticBoxTopRed = new double[] {
		FieldConstants.bumpOuterX.getRed().in(Meters),
		FieldConstants.bumpInnerX.getRed().in(Meters),
		FieldConstants.topBumpBottomY.in(Meters),
		FieldConstants.topBumpTopY.in(Meters)
	};

	private final double[] staticBoxBottomRed = new double[] {
		FieldConstants.bumpOuterX.getRed().in(Meters),
		FieldConstants.bumpInnerX.getRed().in(Meters),
		FieldConstants.bottomBumpTopY.in(Meters),
		FieldConstants.bottomBumpBottomY.in(Meters)
	};

	// Dynamic boxes should be independent copies of the static boxes so
	// we don't accidentally mutate the static originals.
	private double[] dynamicBoxTopBlue = java.util.Arrays.copyOf(staticBoxTopBlue, staticBoxTopBlue.length);
	private double[] dynamicBoxBottomBlue = java.util.Arrays.copyOf(staticBoxBottomBlue, staticBoxBottomBlue.length);
	private double[] dynamicBoxTopRed = java.util.Arrays.copyOf(staticBoxTopRed, staticBoxTopRed.length);
	private double[] dynamicBoxBottomRed = java.util.Arrays.copyOf(staticBoxBottomRed, staticBoxBottomRed.length);

	private static final LoggedTunable<double[]> targetAnglesTunable = new LoggedTunable<>() {
		private final LoggedTunable<Angle> targetAngleOffset = LoggedTunable.from("Automations/Bump Mitigation/Target Angle", Degrees::of, 45.0);

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

	//Meters per meters per second
	private static final LoggedTunable<Time> boxScalingFactor = LoggedTunable.from("Automations/Bump Mitigation/Box Scaling Factor", Seconds::of, 0.5);

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
		var fieldSpeeds = drive.getFieldMeasuredSpeeds();
		updateBoundingBox(dynamicBoxTopBlue, staticBoxTopBlue, fieldSpeeds);
		updateBoundingBox(dynamicBoxBottomBlue, staticBoxBottomBlue, fieldSpeeds);
		updateBoundingBox(dynamicBoxTopRed, staticBoxTopRed, fieldSpeeds);
		updateBoundingBox(dynamicBoxBottomRed, staticBoxBottomRed, fieldSpeeds);
		this.edgeDetector.update(
			(
				withinBounds(dynamicBoxTopBlue, robotPose.getTranslation())
				|| withinBounds(dynamicBoxBottomBlue, robotPose.getTranslation())
				|| withinBounds(dynamicBoxTopRed, robotPose.getTranslation())
				|| withinBounds(dynamicBoxBottomRed, robotPose.getTranslation())
			)
			&& this.turnAxis.getAsDouble() == 0.0
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}

	private void updateBoundingBox(double[] box, double[] original, ChassisSpeeds fieldSpeeds) {
		//Left
		//Right
		//Top
		//Bottom
		Logger.recordOutput("DEBUG/Bump Mitigation", fieldSpeeds.vxMetersPerSecond);

		if (fieldSpeeds.vxMetersPerSecond < 0) {
			box[1] = original[1] + fieldSpeeds.vxMetersPerSecond * boxScalingFactor.get().in(Seconds);
		} else if (fieldSpeeds.vxMetersPerSecond > 0) {
			box[0] = original[0] - fieldSpeeds.vxMetersPerSecond * boxScalingFactor.get().in(Seconds);
		} else {
			// Reset the contents of the dynamic box back to the original
			// values. Assigning to the parameter (box = original) only
			// changes the local reference and does not mutate the caller's
			// array; use element-wise copies instead.
			box[0] = original[0];
			box[1] = original[1];
			box[2] = original[2];
			box[3] = original[3];
		}
	}

	private boolean withinBounds(double[] box, Translation2d position) {
		return (box[0] < position.getX()) && (position.getX() < box[1]) && (box[2] > position.getY()) && (position.getY() > box[3]);
	}
}
