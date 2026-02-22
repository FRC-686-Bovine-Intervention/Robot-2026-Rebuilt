package frc.util.flipping;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.util.geometry.GeomUtil;

public class AllianceFlipUtil {
	public static Alliance getAlliance() {
		return DriverStation.getAlliance().orElse(Alliance.Blue);
	}

	public static boolean shouldFlip() {
		return AllianceFlipUtil.getAlliance() == Alliance.Red;
	}

	public static enum FieldFlipType {
		CenterPointRotation,
		CenterLineMirror,
		XenterLineMirror,
	}
	public static final FieldFlipType defaultFlipType = FieldFlipType.CenterPointRotation;

	public static <T extends AllianceFlippable<T>> T apply(T flippable) {
		return AllianceFlipUtil.apply(flippable, defaultFlipType);
	}
	public static <T extends AllianceFlippable<T>> T apply(T flippable, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return flippable;}
		return AllianceFlipUtil.flip(flippable, flipType);
	}
	public static <T extends AllianceFlippable<T>> T flip(T flippable) {
		return AllianceFlipUtil.flip(flippable, AllianceFlipUtil.defaultFlipType);
	}
	public static <T extends AllianceFlippable<T>> T flip(T flippable, FieldFlipType flipType) {
		return flippable.flip(flipType);
	}

	public static double applyXPosMeters(double xPosMeters) {
		return AllianceFlipUtil.applyXPosMeters(xPosMeters, AllianceFlipUtil.defaultFlipType);
	}
	public static double applyXPosMeters(double xPosMeters, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return xPosMeters;}
		return AllianceFlipUtil.flipXPosMeters(xPosMeters, flipType);
	}
	public static double flipXPosMeters(double xPosMeters) {
		return AllianceFlipUtil.flipXPosMeters(xPosMeters, AllianceFlipUtil.defaultFlipType);
	}
	public static double flipXPosMeters(double xPosMeters, FieldFlipType flipType) {
		return switch (flipType) {
			case CenterPointRotation -> FieldConstants.fieldLength.in(Meters) - xPosMeters;
			case CenterLineMirror    -> FieldConstants.fieldLength.in(Meters) - xPosMeters;
			case XenterLineMirror    -> xPosMeters;
		};
	}
	public static double applyYPosMeters(double yPosMeters) {
		return AllianceFlipUtil.applyYPosMeters(yPosMeters, AllianceFlipUtil.defaultFlipType);
	}
	public static double applyYPosMeters(double yPosMeters, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return yPosMeters;}
		return AllianceFlipUtil.flipYPosMeters(yPosMeters, flipType);
	}
	public static double flipYPosMeters(double yPosMeters) {
		return AllianceFlipUtil.flipYPosMeters(yPosMeters, AllianceFlipUtil.defaultFlipType);
	}
	public static double flipYPosMeters(double yPosMeters, FieldFlipType flipType) {
		return switch (flipType) {
			case CenterPointRotation -> FieldConstants.fieldWidth.in(Meters) - yPosMeters;
			case CenterLineMirror    -> yPosMeters;
			case XenterLineMirror    -> FieldConstants.fieldWidth.in(Meters) - yPosMeters;
		};
	}

	public static Translation2d apply(Translation2d translation) {
		return AllianceFlipUtil.apply(translation, AllianceFlipUtil.defaultFlipType);
	}
	public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return translation;}
		return AllianceFlipUtil.flip(translation, flipType);
	}
	public static Translation2d flip(Translation2d translation) {
		return AllianceFlipUtil.flip(translation, AllianceFlipUtil.defaultFlipType);
	}
	public static Translation2d flip(Translation2d translation, FieldFlipType flipType) {
		return new Translation2d(
			AllianceFlipUtil.flipXPosMeters(translation.getX(), flipType),
			AllianceFlipUtil.flipYPosMeters(translation.getY(), flipType)
		);
	}

	public static Rotation2d apply(Rotation2d rotation) {
		return AllianceFlipUtil.apply(rotation, AllianceFlipUtil.defaultFlipType);
	}
	public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return rotation;}
		return AllianceFlipUtil.flip(rotation, flipType);
	}
	public static Rotation2d flip(Rotation2d rotation) {
		return AllianceFlipUtil.flip(rotation, AllianceFlipUtil.defaultFlipType);
	}
	public static Rotation2d flip(Rotation2d rotation, FieldFlipType flipType) {
		return switch (flipType) {
			case CenterPointRotation -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
			case CenterLineMirror    -> new Rotation2d(-rotation.getCos(), +rotation.getSin());
			case XenterLineMirror    -> new Rotation2d(+rotation.getCos(), -rotation.getSin());
		};
	}

	public static Pose2d apply(Pose2d pose) {
		return AllianceFlipUtil.apply(pose, AllianceFlipUtil.defaultFlipType);
	}
	public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return pose;}
		return AllianceFlipUtil.flip(pose, flipType);
	}
	public static Pose2d flip(Pose2d pose) {
		return AllianceFlipUtil.flip(pose, AllianceFlipUtil.defaultFlipType);
	}
	public static Pose2d flip(Pose2d pose, FieldFlipType flipType) {
		return new Pose2d(AllianceFlipUtil.flip(
			pose.getTranslation(), flipType),
			AllianceFlipUtil.flip(pose.getRotation(), flipType)
		);
	}

	public static Transform2d apply(Transform2d transform) {
		return AllianceFlipUtil.apply(transform, AllianceFlipUtil.defaultFlipType);
	}
	public static Transform2d apply(Transform2d transform, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return transform;}
		return AllianceFlipUtil.flip(transform, flipType);
	}
	public static Transform2d flip(Transform2d transform) {
		return AllianceFlipUtil.flip(transform, AllianceFlipUtil.defaultFlipType);
	}
	public static Transform2d flip(Transform2d transform, FieldFlipType flipType) {
		return new Transform2d(
			AllianceFlipUtil.flip(transform.getTranslation(), flipType),
			AllianceFlipUtil.flip(transform.getRotation(), flipType)
		);
	}

	public static Translation3d apply(Translation3d translation) {
		return AllianceFlipUtil.apply(translation, AllianceFlipUtil.defaultFlipType);
	}
	public static Translation3d apply(Translation3d translation, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return translation;}
		return AllianceFlipUtil.flip(translation, flipType);
	}
	public static Translation3d flip(Translation3d translation) {
		return AllianceFlipUtil.flip(translation, AllianceFlipUtil.defaultFlipType);
	}
	public static Translation3d flip(Translation3d translation, FieldFlipType flipType) {
		return new Translation3d(
			AllianceFlipUtil.flipXPosMeters(translation.getX(), flipType),
			AllianceFlipUtil.flipYPosMeters(translation.getY(), flipType),
			translation.getZ()
		);
	}

	public static Rotation3d apply(Rotation3d rotation) {
		return AllianceFlipUtil.apply(rotation, AllianceFlipUtil.defaultFlipType);
	}
	public static Rotation3d apply(Rotation3d rotation, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return rotation;}
		return AllianceFlipUtil.flip(rotation, flipType);
	}
	public static Rotation3d flip(Rotation3d rotation) {
		return AllianceFlipUtil.flip(rotation, AllianceFlipUtil.defaultFlipType);
	}
	public static Rotation3d flip(Rotation3d rotation, FieldFlipType flipType) {
		return switch (flipType) {
			case CenterPointRotation -> rotation.rotateBy(GeomUtil.rotate180Transform3d.getRotation());
			case CenterLineMirror    -> throw new UnsupportedOperationException("Rotation3d flipping for CenterLineMirror not implemented yet");
			case XenterLineMirror    -> throw new UnsupportedOperationException("Rotation3d flipping for XenterLineMirror not implemented yet");
		};
	}

	public static Pose3d apply(Pose3d pose) {
		return AllianceFlipUtil.apply(pose, AllianceFlipUtil.defaultFlipType);
	}
	public static Pose3d apply(Pose3d pose, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return pose;}
		return AllianceFlipUtil.flip(pose, flipType);
	}
	public static Pose3d flip(Pose3d pose) {
		return AllianceFlipUtil.flip(pose, AllianceFlipUtil.defaultFlipType);
	}
	public static Pose3d flip(Pose3d pose, FieldFlipType flipType) {
		return new Pose3d(
			AllianceFlipUtil.flip(pose.getTranslation(), flipType),
			AllianceFlipUtil.flip(pose.getRotation(), flipType)
		);
	}

	public static Transform3d apply(Transform3d transform) {
		return AllianceFlipUtil.apply(transform, AllianceFlipUtil.defaultFlipType);
	}
	public static Transform3d apply(Transform3d transform, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return transform;}
		return AllianceFlipUtil.flip(transform, flipType);
	}
	public static Transform3d flip(Transform3d transform) {
		return AllianceFlipUtil.flip(transform, AllianceFlipUtil.defaultFlipType);
	}
	public static Transform3d flip(Transform3d transform, FieldFlipType flipType) {
		return new Transform3d(
			AllianceFlipUtil.flip(transform.getTranslation(), flipType),
			AllianceFlipUtil.flip(transform.getRotation(), flipType)
		);
	}

	public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds) {
		return AllianceFlipUtil.applyFieldRelative(speeds, AllianceFlipUtil.defaultFlipType);
	}
	public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return speeds;}
		return AllianceFlipUtil.flipFieldRelative(speeds, flipType);
	}
	public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds) {
		return AllianceFlipUtil.flipFieldRelative(speeds, AllianceFlipUtil.defaultFlipType);
	}
	public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
		return switch (flipType) {
			case CenterPointRotation -> new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, +speeds.omegaRadiansPerSecond);
			case CenterLineMirror    -> new ChassisSpeeds(-speeds.vxMetersPerSecond, +speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
			case XenterLineMirror    -> new ChassisSpeeds(+speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
		};
	}

	public static SwerveSample apply(SwerveSample sample) {
		return AllianceFlipUtil.apply(sample, AllianceFlipUtil.defaultFlipType);
	}
	public static SwerveSample apply(SwerveSample sample, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return sample;}
		return AllianceFlipUtil.flip(sample, flipType);
	}
	public static SwerveSample flip(SwerveSample sample) {
		return AllianceFlipUtil.flip(sample, AllianceFlipUtil.defaultFlipType);
	}
	public static SwerveSample flip(SwerveSample sample, FieldFlipType flipType) {
		var headingCos = Math.cos(sample.heading);
		var headingSin = Math.sin(sample.heading);
		return switch (flipType) {
			case CenterPointRotation -> new SwerveSample(
				sample.t,
				AllianceFlipUtil.flipXPosMeters(sample.x, flipType),
				AllianceFlipUtil.flipYPosMeters(sample.y, flipType),
				Math.atan2(-headingSin, -headingCos),
				-sample.vx,
				-sample.vy,
				+sample.omega,
				-sample.ax,
				-sample.ay,
				+sample.alpha,
				new double[] {-sample.moduleForcesX()[0], -sample.moduleForcesX()[1], -sample.moduleForcesX()[2], -sample.moduleForcesX()[3]},
				new double[] {-sample.moduleForcesY()[0], -sample.moduleForcesY()[1], -sample.moduleForcesY()[2], -sample.moduleForcesY()[3]}
			);
			case CenterLineMirror -> new SwerveSample(
				sample.t,
				AllianceFlipUtil.flipXPosMeters(sample.x, flipType),
				AllianceFlipUtil.flipYPosMeters(sample.y, flipType),
				Math.atan2(+headingSin, -headingCos),
				-sample.vx,
				+sample.vy,
				-sample.omega,
				-sample.ax,
				+sample.ay,
				-sample.alpha,
				new double[] {-sample.moduleForcesX()[0], -sample.moduleForcesX()[1], -sample.moduleForcesX()[2], -sample.moduleForcesX()[3]},
				new double[] {+sample.moduleForcesY()[0], +sample.moduleForcesY()[1], +sample.moduleForcesY()[2], +sample.moduleForcesY()[3]}
			);
			case XenterLineMirror -> new SwerveSample(
				sample.t,
				AllianceFlipUtil.flipXPosMeters(sample.x, flipType),
				AllianceFlipUtil.flipYPosMeters(sample.y, flipType),
				Math.atan2(-headingSin, +headingCos),
				+sample.vx,
				-sample.vy,
				-sample.omega,
				+sample.ax,
				-sample.ay,
				-sample.alpha,
				new double[] {+sample.moduleForcesX()[0], +sample.moduleForcesX()[1], +sample.moduleForcesX()[2], +sample.moduleForcesX()[3]},
				new double[] {-sample.moduleForcesY()[0], -sample.moduleForcesY()[1], -sample.moduleForcesY()[2], -sample.moduleForcesY()[3]}
			);
		};
	}

	public static Trajectory<SwerveSample> apply(Trajectory<SwerveSample> trajectory) {
		return AllianceFlipUtil.apply(trajectory, AllianceFlipUtil.defaultFlipType);
	}
	public static Trajectory<SwerveSample> apply(Trajectory<SwerveSample> trajectory, FieldFlipType flipType) {
		if (!AllianceFlipUtil.shouldFlip()) {return trajectory;}
		return AllianceFlipUtil.flip(trajectory, flipType);
	}
	public static Trajectory<SwerveSample> flip(Trajectory<SwerveSample> trajectory) {
		return AllianceFlipUtil.flip(trajectory, AllianceFlipUtil.defaultFlipType);
	}
	public static Trajectory<SwerveSample> flip(Trajectory<SwerveSample> trajectory, FieldFlipType flipType) {
		var flippedSamples = new ArrayList<SwerveSample>(trajectory.samples().size());
		for (var sample : trajectory.samples()) {
			flippedSamples.add(AllianceFlipUtil.flip(sample, flipType));
		}
		return new Trajectory<>(trajectory.name(), flippedSamples, trajectory.splits(), trajectory.events());
	}
}
