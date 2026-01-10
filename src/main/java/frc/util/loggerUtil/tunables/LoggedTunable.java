package frc.util.loggerUtil.tunables;

import java.util.function.DoubleFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.RobotConstants;

public interface LoggedTunable<T> {
	public static final String TABLE_KEY = "/Tuning";

	/**
	 * Get the current value, from dashboard if available and in tuning mode.
	 *
	 * @return The current value
	 */
	public T get();
	/**
	 * Checks whether the value has changed since our last check
	 *
	 * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
	 *     objects. Recommended approach is to pass the result of "hashCode()"
	 * @return True if the value has changed since the last time this method was called, false
	 *     otherwise.
	 */
	public boolean hasChanged(int id);

	/**
	 * Checks whether any of the provided tunables have changed since our last check
	 *
	 * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
	 *     objects. Recommended approach is to pass the result of "hashCode()"
	 * @param tunables An array of tunables to check
	 * @return True if any of the tunables have changed since the last time this method was called, false
	 *     otherwise.
	 * @implSpec The non-shortcircuiting nature of this method is intentional
	 */
	public static boolean hasChanged(int id, LoggedTunable<?>... tunables) {
		if (!RobotConstants.tuningMode) {
			return false;
		}
		var out = false;
		for (var tunable : tunables) {
			if (tunable.hasChanged(id)) {
				out = true;
			}
		}
		return out;
	}

	/**
	 * Creates a new {@link LoggedTunableNumber} using the provided key and default value
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunableNumber}
	 */
	public static LoggedTunableNumber from(String key, double defaultValue) {
		return new LoggedTunableNumber(key, defaultValue);
	}

	/**
	 * Creates a new {@link LoggedTunable} using the provided key and default value that implements {@link Tunable}
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable}
	 */
	public static <T, U extends Tunable<T>> LoggedTunable<T> from(String key, U defaultValue) {
		return defaultValue.makeTunable(key);
	}

	/**
	 * Creates a new {@link LoggedTunable} using the provided key, constructor, and default value
	 * @param key Key on dashboard
	 * @param constructor Function to convert the number on the dashboard into the value this {@link LoggedTunable} returns
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable}
	 */
	public static <T> LoggedTunable<T> from(String key, DoubleFunction<T> constructor, double defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunableNumber tunableNumber = LoggedTunable.from(key, defaultValue);

			private T cache = constructor.apply(defaultValue);

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.tunableNumber);
			}

			@Override
			public T get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = constructor.apply(this.tunableNumber.getAsDouble());
				}
				return this.cache;
			}
		};
	}

	/**
	 * Creates a new {@link LoggedTunable} for the {@link #TrapezoidProfile.Constraints constraints} of a {@link TrapezoidProfile} using the provided key and default value
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<TrapezoidProfile.Constraints> from(String key, TrapezoidProfile.Constraints defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunableNumber maxVelocity = LoggedTunable.from(key + "/Max Velocity", defaultValue.maxVelocity);
			private final LoggedTunableNumber maxAcceleration = LoggedTunable.from(key + "/Max Acceleration", defaultValue.maxAcceleration);

			private TrapezoidProfile.Constraints cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.maxVelocity, this.maxAcceleration);
			}

			@Override
			public TrapezoidProfile.Constraints get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new TrapezoidProfile.Constraints(
						this.maxVelocity.getAsDouble(),
						this.maxAcceleration.getAsDouble()
					);
				}
				return this.cache;
			}
		};
	}
	/**
	 * Creates a new {@link LoggedTunable} for the {@link #TrapezoidProfile.Constraints constraints} of an angular {@link TrapezoidProfile} using the provided key and default value
	 * <p> Interprets the provided default value as if it had the units of the provided constraint units, and will convert the default value into the dashboard units before putting them on the dashboard
	 * @param key Key on dashboard
	 * @param velocityDashboardUnit Unit to use for the max velocity on the dashboard
	 * @param accelerationDashboardUnit Unit to use for the max acceleration on the dashboard
	 * @param velocityConstraintUnit Unit to use for the max velocity in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param accelerationConstraintUnit Unit to use for the max acceleration in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param defaultValue Default value, assumed to have the units of the provided constraint units
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<TrapezoidProfile.Constraints> fromConstraintUnits(
		String key,
		AngularVelocityUnit velocityDashboardUnit,
		AngularAccelerationUnit accelerationDashboardUnit,
		AngularVelocityUnit velocityConstraintUnit,
		AngularAccelerationUnit accelerationConstraintUnit,
		TrapezoidProfile.Constraints defaultValue
	) {
		return new LoggedTunable<>() {
			private final LoggedTunable<AngularVelocity> maxVelocity = LoggedTunable.from(key + "/Max Velocity", velocityDashboardUnit::of, velocityDashboardUnit.convertFrom(defaultValue.maxVelocity, velocityConstraintUnit));
			private final LoggedTunable<AngularAcceleration> maxAcceleration = LoggedTunable.from(key + "/Max Acceleration", accelerationDashboardUnit::of, accelerationDashboardUnit.convertFrom(defaultValue.maxAcceleration, accelerationConstraintUnit));

			private TrapezoidProfile.Constraints cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.maxVelocity, this.maxAcceleration);
			}

			@Override
			public TrapezoidProfile.Constraints get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new TrapezoidProfile.Constraints(
						this.maxVelocity.get().in(velocityConstraintUnit),
						this.maxAcceleration.get().in(accelerationConstraintUnit)
					);
				}
				return this.cache;
			}
		};
	}
	/**
	 * Creates a new {@link LoggedTunable} for the {@link #TrapezoidProfile.Constraints constraints} of an angular {@link TrapezoidProfile} using the provided key and default value
	 * <p> Interprets the provided default value as if it had the units of the provided dashboard units, and will convert the default value into the constraint units before caching the value
	 * @param key Key on dashboard
	 * @param velocityDashboardUnit Unit to use for the max velocity on the dashboard
	 * @param accelerationDashboardUnit Unit to use for the max acceleration on the dashboard
	 * @param velocityConstraintUnit Unit to use for the max velocity in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param accelerationConstraintUnit Unit to use for the max acceleration in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param defaultValue Default value, assumed to have the units of the provided dashboard units
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<TrapezoidProfile.Constraints> fromDashboardUnits(
		String key,
		AngularVelocityUnit velocityDashboardUnit,
		AngularAccelerationUnit accelerationDashboardUnit,
		AngularVelocityUnit velocityConstraintUnit,
		AngularAccelerationUnit accelerationConstraintUnit,
		TrapezoidProfile.Constraints defaultValue
	) {
		return LoggedTunable.fromConstraintUnits(
			key,
			velocityDashboardUnit,
			accelerationDashboardUnit,
			velocityConstraintUnit,
			accelerationConstraintUnit,
			new TrapezoidProfile.Constraints(
				velocityConstraintUnit.convertFrom(defaultValue.maxVelocity, velocityDashboardUnit),
				accelerationConstraintUnit.convertFrom(defaultValue.maxAcceleration, accelerationDashboardUnit)
			)
		);
	}
	/**
	 * Creates a new {@link LoggedTunable} for the {@link #TrapezoidProfile.Constraints constraints} of an angular {@link TrapezoidProfile} using the provided key and default value
	 * <p> Interprets the provided default value as if it had the units of the provided constraint units, and will convert the default value into the dashboard units before putting them on the dashboard
	 * @param key Key on dashboard
	 * @param velocityDashboardUnit Unit to use for the max velocity on the dashboard
	 * @param accelerationDashboardUnit Unit to use for the max acceleration on the dashboard
	 * @param velocityConstraintUnit Unit to use for the max velocity in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param accelerationConstraintUnit Unit to use for the max acceleration in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param defaultValue Default value, assumed to have the units of the provided constraint units
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<TrapezoidProfile.Constraints> fromConstraintUnits(
		String key,
		LinearVelocityUnit velocityDashboardUnit,
		LinearAccelerationUnit accelerationDashboardUnit,
		LinearVelocityUnit velocityConstraintUnit,
		LinearAccelerationUnit accelerationConstraintUnit,
		TrapezoidProfile.Constraints defaultValue
	) {
		return new LoggedTunable<>() {
			private final LoggedTunable<LinearVelocity> maxVelocity = LoggedTunable.from(key + "/Max Velocity", velocityDashboardUnit::of, velocityDashboardUnit.convertFrom(defaultValue.maxVelocity, velocityConstraintUnit));
			private final LoggedTunable<LinearAcceleration> maxAcceleration = LoggedTunable.from(key + "/Max Acceleration", accelerationDashboardUnit::of, accelerationDashboardUnit.convertFrom(defaultValue.maxAcceleration, accelerationConstraintUnit));

			private TrapezoidProfile.Constraints cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.maxVelocity, this.maxAcceleration);
			}

			@Override
			public TrapezoidProfile.Constraints get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new TrapezoidProfile.Constraints(
						this.maxVelocity.get().in(velocityConstraintUnit),
						this.maxAcceleration.get().in(accelerationConstraintUnit)
					);
				}
				return this.cache;
			}
		};
	}
	/**
	 * Creates a new {@link LoggedTunable} for the {@link #TrapezoidProfile.Constraints constraints} of an angular {@link TrapezoidProfile} using the provided key and default value
	 * <p> Interprets the provided default value as if it had the units of the provided dashboard units, and will convert the default value into the constraint units before caching the value
	 * @param key Key on dashboard
	 * @param velocityDashboardUnit Unit to use for the max velocity on the dashboard
	 * @param accelerationDashboardUnit Unit to use for the max acceleration on the dashboard
	 * @param velocityConstraintUnit Unit to use for the max velocity in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param accelerationConstraintUnit Unit to use for the max acceleration in the returned {@link #TrapezoidProfile.Constraints constraints}
	 * @param defaultValue Default value, assumed to have the units of the provided dashboard units
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<TrapezoidProfile.Constraints> fromDashboardUnits(
		String key,
		LinearVelocityUnit velocityDashboardUnit,
		LinearAccelerationUnit accelerationDashboardUnit,
		LinearVelocityUnit velocityConstraintUnit,
		LinearAccelerationUnit accelerationConstraintUnit,
		TrapezoidProfile.Constraints defaultValue
	) {
		return LoggedTunable.fromConstraintUnits(
			key,
			velocityDashboardUnit,
			accelerationDashboardUnit,
			velocityConstraintUnit,
			accelerationConstraintUnit,
			new TrapezoidProfile.Constraints(
				velocityConstraintUnit.convertFrom(defaultValue.maxVelocity, velocityDashboardUnit),
				accelerationConstraintUnit.convertFrom(defaultValue.maxAcceleration, accelerationDashboardUnit)
			)
		);
	}

	/**
	 * Creates a new {@link LoggedTunable} for a {@link Translation2d} using the provided key and default value
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<Translation2d> from(String key, Translation2d defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunableNumber x = LoggedTunable.from(key + "/x", defaultValue.getX());
			private final LoggedTunableNumber y = LoggedTunable.from(key + "/y", defaultValue.getY());

			private Translation2d cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.x, this.y);
			}

			@Override
			public Translation2d get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new Translation2d(this.x.get(), this.y.get());
				}
				return this.cache;
			}
		};
	}
	/**
	 * Creates a new {@link LoggedTunable} for a {@link Translation2d} using the provided key and default value
	 * @param key Key on dashboard
	 * @param dashboardUnit Unit to use on the dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<Translation2d> from(String key, DistanceUnit dashboardUnit, Translation2d defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunable<Distance> x = LoggedTunable.from(key + "/x", dashboardUnit::of, defaultValue.getMeasureX().in(dashboardUnit));
			private final LoggedTunable<Distance> y = LoggedTunable.from(key + "/y", dashboardUnit::of, defaultValue.getMeasureY().in(dashboardUnit));

			private Translation2d cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.x, this.y);
			}

			@Override
			public Translation2d get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new Translation2d(this.x.get(), this.y.get());
				}
				return this.cache;
			}
		};
	}

	/**
	 * Creates a new {@link LoggedTunable} for a {@link Rotation2d} using the provided key and default value
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<Rotation2d> from(String key, Rotation2d defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunableNumber theta = LoggedTunable.from(key + "/theta", defaultValue.getRadians());

			private Rotation2d cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.theta);
			}

			@Override
			public Rotation2d get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new Rotation2d(this.theta.get());
				}
				return this.cache;
			}
		};
	}
	/**
	 * Creates a new {@link LoggedTunable} for a {@link Rotation2d} using the provided key and default value
	 * @param key Key on dashboard
	 * @param dashboardUnit Unit to use on the dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<Rotation2d> from(String key, AngleUnit dashboardUnit, Rotation2d defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunable<Angle> theta = LoggedTunable.from(key + "/theta", dashboardUnit::of, defaultValue.getMeasure().in(dashboardUnit));

			private Rotation2d cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.theta);
			}

			@Override
			public Rotation2d get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new Rotation2d(this.theta.get());
				}
				return this.cache;
			}
		};
	}

	/**
	 * Creates a new {@link LoggedTunable} for a {@link Pose2d} using the provided key and default value
	 * @param key Key on dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<Pose2d> from(String key, Pose2d defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunable<Translation2d> translation = LoggedTunable.from(key + "/translation", defaultValue.getTranslation());
			private final LoggedTunable<Rotation2d> rotation = LoggedTunable.from(key + "/rotation", defaultValue.getRotation());

			private Pose2d cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.translation, this.rotation);
			}

			@Override
			public Pose2d get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new Pose2d(this.translation.get(), this.rotation.get());
				}
				return this.cache;
			}
		};
	}
	/**
	 * Creates a new {@link LoggedTunable} for a {@link Pose2d} using the provided key and default value
	 * @param key Key on dashboard
	 * @param translationDashboardUnit Unit to use for the {@link #Translation2d translation} on the dashboard
	 * @param rotationDashboardUnit Unit to use for the {@link #Rotation2d rotation} on the dashboard
	 * @param defaultValue Default value
	 * @return A new {@link LoggedTunable LoggedTunable}
	 */
	public static LoggedTunable<Pose2d> from(String key, DistanceUnit translationDashboardUnit, AngleUnit rotationDashboardUnit, Pose2d defaultValue) {
		return new LoggedTunable<>() {
			private final LoggedTunable<Translation2d> translation = LoggedTunable.from(key + "/translation", translationDashboardUnit, defaultValue.getTranslation());
			private final LoggedTunable<Rotation2d> rotation = LoggedTunable.from(key + "/rotation", rotationDashboardUnit, defaultValue.getRotation());

			private Pose2d cache = defaultValue;

			@Override
			public boolean hasChanged(int id) {
				return LoggedTunable.hasChanged(id, this.translation, this.rotation);
			}

			@Override
			public Pose2d get() {
				if (this.hasChanged(this.hashCode())) {
					this.cache = new Pose2d(this.translation.get(), this.rotation.get());
				}
				return this.cache;
			}
		};
	}
}
