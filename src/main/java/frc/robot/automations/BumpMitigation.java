package frc.robot.automations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.util.EdgeDetector;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class BumpMitigation implements Runnable {
	private final Drive drive;
	private final Command command;

	private static final double[] staticBoxTopBlue = new double[] {
		FieldConstants.bumpInnerX.getBlue().in(Meters),
		FieldConstants.bumpOuterX.getBlue().in(Meters),
		FieldConstants.topBumpBottomY.in(Meters),
		FieldConstants.topBumpTopY.in(Meters)
	};

	private static final double[] staticBoxBottomBlue = new double[] {
		FieldConstants.bumpInnerX.getBlue().in(Meters),
		FieldConstants.bumpOuterX.getBlue().in(Meters),
		FieldConstants.bottomBumpTopY.in(Meters),
		FieldConstants.bottomBumpBottomY.in(Meters)
	};

	private static final double[] staticBoxTopRed = new double[] {
		FieldConstants.bumpOuterX.getRed().in(Meters),
		FieldConstants.bumpInnerX.getRed().in(Meters),
		FieldConstants.topBumpBottomY.in(Meters),
		FieldConstants.topBumpTopY.in(Meters)
	};

	private static final double[] staticBoxBottomRed = new double[] {
		FieldConstants.bumpOuterX.getRed().in(Meters),
		FieldConstants.bumpInnerX.getRed().in(Meters),
		FieldConstants.bottomBumpTopY.in(Meters),
		FieldConstants.bottomBumpBottomY.in(Meters)
	};

	private double[] dynamicBoxTopBlue =    Arrays.copyOf(BumpMitigation.staticBoxTopBlue,    BumpMitigation.staticBoxTopBlue.length);
	private double[] dynamicBoxBottomBlue = Arrays.copyOf(BumpMitigation.staticBoxBottomBlue, BumpMitigation.staticBoxBottomBlue.length);
	private double[] dynamicBoxTopRed =     Arrays.copyOf(BumpMitigation.staticBoxTopRed,     BumpMitigation.staticBoxTopRed.length);
	private double[] dynamicBoxBottomRed =  Arrays.copyOf(BumpMitigation.staticBoxBottomRed,  BumpMitigation.staticBoxBottomRed.length);

	private static final LoggedTunable<double[]> targetAnglesTunable = LoggedTunable.from("Automations/Bump Mitigation/Target Angle", Degrees::of, 45.0).map(
		(offset) -> new double[] {
			0 - offset.in(Radians),
			0 + offset.in(Radians),
			Math.PI/2 - offset.in(Radians),
			Math.PI/2 + offset.in(Radians),
			Math.PI - offset.in(Radians),
			Math.PI + offset.in(Radians),
			-Math.PI/2 - offset.in(Radians),
			-Math.PI/2 + offset.in(Radians),
		}
	);

	private static final LoggedTunable<Time> velocityLookaheadTime = LoggedTunable.from("Automations/Bump Mitigation/Velocity Lookahead Time", Seconds::of, 0.5);

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public BumpMitigation(Drive drive) {
		this.drive = drive;
		this.command = this.drive.rotationalSubsystem.pidControlledHeading(() -> {
			var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
			var robotRotation = robotPose.getRotation().getRadians();

			var targetAngles = BumpMitigation.targetAnglesTunable.get();

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
		var fieldSpeeds = this.drive.getFieldMeasuredSpeeds();
		this.updateBoundingBox(dynamicBoxTopBlue, BumpMitigation.staticBoxTopBlue, fieldSpeeds);
		this.updateBoundingBox(dynamicBoxBottomBlue, BumpMitigation.staticBoxBottomBlue, fieldSpeeds);
		this.updateBoundingBox(dynamicBoxTopRed, BumpMitigation.staticBoxTopRed, fieldSpeeds);
		this.updateBoundingBox(dynamicBoxBottomRed, BumpMitigation.staticBoxBottomRed, fieldSpeeds);
		this.edgeDetector.update(
			(
				this.withinBounds(dynamicBoxTopBlue, robotPose.getTranslation())
				|| this.withinBounds(dynamicBoxBottomBlue, robotPose.getTranslation())
				|| this.withinBounds(dynamicBoxTopRed, robotPose.getTranslation())
				|| this.withinBounds(dynamicBoxBottomRed, robotPose.getTranslation())
			)
			&& this.drive.rotationalSubsystem.getCurrentCommand() == null
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
			box[1] = original[1] - fieldSpeeds.vxMetersPerSecond * BumpMitigation.velocityLookaheadTime.get().in(Seconds);
		} else if (fieldSpeeds.vxMetersPerSecond > 0) {
			box[0] = original[0] - fieldSpeeds.vxMetersPerSecond * BumpMitigation.velocityLookaheadTime.get().in(Seconds);
		} else {
			box[0] = original[0];
			box[1] = original[1];
			box[2] = original[2];
			box[3] = original[3];
		}
	}

	private boolean withinBounds(double[] box, Translation2d position) {
		return
			(box[0] < position.getX())
			&& (position.getX() < box[1])
			&& (box[2] > position.getY())
			&& (position.getY() > box[3])
		;
	}
}
