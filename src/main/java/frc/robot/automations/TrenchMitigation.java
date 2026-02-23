package frc.robot.automations;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.util.EdgeDetector;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class TrenchMitigation implements Runnable {
	private final Drive drive;
	private final IntakeSlam slam;
	private final ExtensionSystem extensionSystem;
	private final Hood hood;
	private final Command command;

	private final double[] staticBoxTopBlue = new double[] {
		FieldConstants.trenchInnerX.getBlue().in(Meters),
		FieldConstants.trenchOuterX.getBlue().in(Meters),
		FieldConstants.topTrenchBottomY.in(Meters),
		FieldConstants.topTrenchTopY.in(Meters)
	};

	private final double[] staticBoxBottomBlue = new double[] {
		FieldConstants.trenchInnerX.getBlue().in(Meters),
		FieldConstants.trenchOuterX.getBlue().in(Meters),
		FieldConstants.bottomTrenchTopY.in(Meters),
		FieldConstants.bottomTrenchBottomY.in(Meters)
	};

	private final double[] staticBoxTopRed = new double[] {
		FieldConstants.trenchOuterX.getRed().in(Meters),
		FieldConstants.trenchInnerX.getRed().in(Meters),
		FieldConstants.topTrenchBottomY.in(Meters),
		FieldConstants.topTrenchTopY.in(Meters)
	};

	private final double[] staticBoxBottomRed = new double[] {
		FieldConstants.trenchOuterX.getRed().in(Meters),
		FieldConstants.trenchInnerX.getRed().in(Meters),
		FieldConstants.bottomTrenchTopY.in(Meters),
		FieldConstants.bottomTrenchBottomY.in(Meters)
	};

	private double[] dynamicBoxTopBlue = java.util.Arrays.copyOf(staticBoxTopBlue, staticBoxTopBlue.length);
	private double[] dynamicBoxBottomBlue = java.util.Arrays.copyOf(staticBoxBottomBlue, staticBoxBottomBlue.length);
	private double[] dynamicBoxTopRed = java.util.Arrays.copyOf(staticBoxTopRed, staticBoxTopRed.length);
	private double[] dynamicBoxBottomRed = java.util.Arrays.copyOf(staticBoxBottomRed, staticBoxBottomRed.length);

	private static final LoggedTunable<Time> boxScalingFactor = LoggedTunable.from("Automations/Trench Mitigation/Box Scaling Factor", Seconds::of, 0.5);

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public TrenchMitigation(Drive drive, IntakeSlam slam, ExtensionSystem extensionSystem, Hood hood) {
		this.drive = drive;
		this.slam = slam;
		this.extensionSystem = extensionSystem;
		this.hood = hood;
		this.command = this.drive.rotationalSubsystem.pidControlledHeading(() -> {
			return new Rotation2d(AllianceFlipUtil.getAlliance() == Alliance.Blue ? 0 : Math.PI);
		}).alongWith(this.slam.deploy(extensionSystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
		.alongWith(this.hood.idle()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
			(((withinBounds(dynamicBoxTopBlue, robotPose.getTranslation()) || withinBounds(dynamicBoxBottomBlue, robotPose.getTranslation())) && AllianceFlipUtil.getAlliance() == Alliance.Blue)
			||
			((withinBounds(dynamicBoxTopRed, robotPose.getTranslation()) || withinBounds(dynamicBoxBottomRed, robotPose.getTranslation())) && AllianceFlipUtil.getAlliance() == Alliance.Red))
			&& this.drive.getFieldMeasuredSpeeds().vxMetersPerSecond * (AllianceFlipUtil.getAlliance() == Alliance.Blue ? 1 : -1) < 0
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
		Logger.recordOutput("DEBUG/Trench Mitigation", fieldSpeeds.vxMetersPerSecond);

		if (fieldSpeeds.vxMetersPerSecond < 0) {
			box[1] = original[1] + fieldSpeeds.vxMetersPerSecond * boxScalingFactor.get().in(Seconds);
		} else if (fieldSpeeds.vxMetersPerSecond > 0) {
			box[0] = original[0] - fieldSpeeds.vxMetersPerSecond * boxScalingFactor.get().in(Seconds);
		} else {
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
