package frc.util.flipping;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Function;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;

public class AllianceFlipped<T> {
	private final T blue;
	private final T red;

	public AllianceFlipped(T blue, T red) {
		this.blue = blue;
		this.red = red;
	}

	public T getBlue() {
		return this.blue;
	}
	public T getRed() {
		return this.red;
	}
	public T get(Alliance alliance) {
		return switch (alliance) {
			case Blue -> this.getBlue();
			case Red -> this.getRed();
		};
	}

	public T getOurs() {
		return switch (AllianceFlipUtil.getAlliance()) {
			case Blue -> this.getBlue();
			case Red -> this.getRed();
		};
	}
	public T getTheirs() {
		return switch (AllianceFlipUtil.getAlliance()) {
			case Blue -> this.getRed();
			case Red -> this.getBlue();
		};
	}

	public <U> AllianceFlipped<U> map(Function<T, U> mappingFunction) {
		return new AllianceFlipped<>(mappingFunction.apply(this.blue), mappingFunction.apply(this.red));
	}
	public AllianceFlipped<T> invert() {
		return new AllianceFlipped<>(this.red, this.blue);
	}

	public static <T> AllianceFlipped<T> fromFunction(Function<Alliance, T> generator) {
		return new AllianceFlipped<>(generator.apply(Alliance.Blue), generator.apply(Alliance.Red));
	}

	public static AllianceFlipped<Distance> fromBlueXPos(Distance blue) {
		return AllianceFlipped.fromBlueXPos(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Distance> fromBlueXPos(Distance blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(Meters.of(blue.in(Meters)), Meters.of(AllianceFlipUtil.flipXPosMeters(blue.in(Meters), flipType)));
	}
	public static AllianceFlipped<Distance> fromRedXPos(Distance blue) {
		return AllianceFlipped.fromRedXPos(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Distance> fromRedXPos(Distance blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(Meters.of(AllianceFlipUtil.flipXPosMeters(blue.in(Meters), flipType)), Meters.of(blue.in(Meters)));
	}

	public static AllianceFlipped<Distance> fromBlueYPos(Distance blue) {
		return AllianceFlipped.fromBlueXPos(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Distance> fromBlueYPos(Distance blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(Meters.of(blue.in(Meters)), Meters.of(AllianceFlipUtil.flipYPosMeters(blue.in(Meters), flipType)));
	}
	public static AllianceFlipped<Distance> fromRedYPos(Distance blue) {
		return AllianceFlipped.fromRedXPos(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Distance> fromRedYPos(Distance blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(Meters.of(AllianceFlipUtil.flipYPosMeters(blue.in(Meters), flipType)), Meters.of(blue.in(Meters)));
	}

	public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromBlue(T blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromBlue(T blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromRed(T red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromRed(T red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Translation2d> fromBlue(Translation2d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Translation2d> fromBlue(Translation2d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Translation2d> fromRed(Translation2d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Translation2d> fromRed(Translation2d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Rotation2d> fromBlue(Rotation2d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Rotation2d> fromBlue(Rotation2d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Rotation2d> fromRed(Rotation2d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Rotation2d> fromRed(Rotation2d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Pose2d> fromBlue(Pose2d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Pose2d> fromBlue(Pose2d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Pose2d> fromRed(Pose2d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Pose2d> fromRed(Pose2d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Transform2d> fromBlue(Transform2d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Transform2d> fromBlue(Transform2d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Transform2d> fromRed(Transform2d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Transform2d> fromRed(Transform2d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Translation3d> fromBlue(Translation3d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Translation3d> fromBlue(Translation3d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Translation3d> fromRed(Translation3d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Translation3d> fromRed(Translation3d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Rotation3d> fromBlue(Rotation3d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Rotation3d> fromBlue(Rotation3d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Rotation3d> fromRed(Rotation3d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Rotation3d> fromRed(Rotation3d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Pose3d> fromBlue(Pose3d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Pose3d> fromBlue(Pose3d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Pose3d> fromRed(Pose3d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Pose3d> fromRed(Pose3d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Transform3d> fromBlue(Transform3d blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Transform3d> fromBlue(Transform3d blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Transform3d> fromRed(Transform3d red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Transform3d> fromRed(Transform3d red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<ChassisSpeeds> fromBlue(ChassisSpeeds blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<ChassisSpeeds> fromBlue(ChassisSpeeds blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flipFieldRelative(blue, flipType));
	}
	public static AllianceFlipped<ChassisSpeeds> fromRed(ChassisSpeeds red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<ChassisSpeeds> fromRed(ChassisSpeeds red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flipFieldRelative(red, flipType), red);
	}

	public static AllianceFlipped<SwerveSample> fromBlue(SwerveSample blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<SwerveSample> fromBlue(SwerveSample blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<SwerveSample> fromRed(SwerveSample red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<SwerveSample> fromRed(SwerveSample red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}

	public static AllianceFlipped<Trajectory<SwerveSample>> fromBlue(Trajectory<SwerveSample> blue) {
		return AllianceFlipped.fromBlue(blue, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Trajectory<SwerveSample>> fromBlue(Trajectory<SwerveSample> blue, FieldFlipType flipType) {
		return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
	}
	public static AllianceFlipped<Trajectory<SwerveSample>> fromRed(Trajectory<SwerveSample> red) {
		return AllianceFlipped.fromRed(red, AllianceFlipUtil.defaultFlipType);
	}
	public static AllianceFlipped<Trajectory<SwerveSample>> fromRed(Trajectory<SwerveSample> red, FieldFlipType flipType) {
		return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
	}
}
