package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.modules.Module;
import frc.robot.subsystems.drive.modules.ModuleIO;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIO;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIOInputsAutoLogged;
import frc.util.Environment;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.VirtualSubsystem;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.Root;

public class Drive extends VirtualSubsystem {
	private final OdometryTimestampIO odometryTimestampIO;
	private final OdometryTimestampIOInputsAutoLogged odometryTimestamps = new OdometryTimestampIOInputsAutoLogged();

	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	public final Module[] modules = new Module[DriveConstants.moduleConstants.length];

	private static final LoggedTunableNumber rotationCorrection = LoggedTunable.from("Drive/Rotation Correction", 0.125);
	public static final Supplier<DoubleSupplier> maxDriveSpeedEnvCoef = Environment.switchVar(
		() -> 1.0,
		new LoggedNetworkNumber("Demo Constraints/Max Translational Percentage", 0.25)::get
	);
	public static final Supplier<DoubleSupplier> maxTurnRateEnvCoef = Environment.switchVar(
		() -> 1.0,
		new LoggedNetworkNumber("Demo Constraints/Max Rotational Percentage", 0.5)::get
	);

	public final Root structureRoot = new Root();

	private SwerveModulePosition[] lastMeasuredPositions = null;
	private SwerveModuleState[] measuredStates = new SwerveModuleState[] {
		new SwerveModuleState(),
		new SwerveModuleState(),
		new SwerveModuleState(),
		new SwerveModuleState()
	};
	private ChassisSpeeds robotMeasuredSpeeds = new ChassisSpeeds();
	private ChassisSpeeds fieldMeasuredSpeeds = new ChassisSpeeds();

	private ChassisSpeeds desiredRobotSpeeds = new ChassisSpeeds();
	private Translation2d centerOfRotation = new Translation2d();
	private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
		new SwerveModuleState(),
		new SwerveModuleState(),
		new SwerveModuleState(),
		new SwerveModuleState()
	};

	public Drive(OdometryTimestampIO odometryTimestampIO, GyroIO gyroIO, ModuleIO... moduleIOs) {
		System.out.println("[Init Drive] Instantiating Drive");
		this.odometryTimestampIO = odometryTimestampIO;
		this.gyroIO = gyroIO;
		System.out.println("[Init Drive] Gyro IO: " + this.gyroIO.getClass().getSimpleName());
		for(int i = 0; i < DriveConstants.moduleConstants.length; i++) {
			var config = DriveConstants.moduleConstants[i];
			System.out.println("[Init Drive] Instantiating Module " + config.name + " with Module IO: " + moduleIOs[i].getClass().getSimpleName());
			var module = new Module(moduleIOs[i], config);
			module.periodic();
			this.modules[i] = module;
		}

		OdometryThread.getInstance().start();

		this.translationSubsystem = new Translational(this);
		this.rotationalSubsystem = new Rotational(this);

		var routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				null,
				null,
				(state) -> {
					Logger.recordOutput("SysID/Drive/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(volts) -> {
					Arrays.stream(this.modules).forEach((module) -> module.runVolts(volts.in(Volts), Rotation2d.kZero));
				},
				(log) -> {
					Arrays.stream(this.modules).forEach((module) -> {
						Logger.recordOutput("SysID/Drive/" + module.config.name + "/Position", module.getWheelAngularPositionRads());
						Logger.recordOutput("SysID/Drive/" + module.config.name + "/Velocity", module.getWheelAngularVelocityRadsPerSec());
						Logger.recordOutput("SysID/Drive/" + module.config.name + "/Voltage", module.getDriveAppliedVolts());
					});
				},
				this.translationSubsystem
			)
		);

		SmartDashboard.putData("SysID/Drive/Quasi Forward", routine.quasistatic(Direction.kForward).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Quasistatic Forward").asProxy());
		SmartDashboard.putData("SysID/Drive/Quasi Reverse", routine.quasistatic(Direction.kReverse).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Quasistatic Reverse").asProxy());
		SmartDashboard.putData("SysID/Drive/Dynamic Forward", routine.dynamic(Direction.kForward).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Dynamic Forward").asProxy());
		SmartDashboard.putData("SysID/Drive/Dynamic Reverse", routine.dynamic(Direction.kReverse).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Dynamic Reverse").asProxy());
	}

	private static final SwerveModuleState[] emptyStates = new SwerveModuleState[0];
	private static final ChassisSpeeds emptySpeeds = new ChassisSpeeds();

	@Override
	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Before");
		OdometryThread.getInstance().odometryLock.lock();
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Acquire Odometry Lock");

		this.odometryTimestampIO.updateInputs(this.odometryTimestamps);
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Update Timestamp Inputs");
		Logger.processInputs("Inputs/Drive/Timestamps", this.odometryTimestamps);
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Process Timestamp Inputs");

		this.gyroIO.updateInputs(this.gyroInputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Update Gyro Inputs");
		Logger.processInputs("Inputs/Drive/Gyro", this.gyroInputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Process Gyro Inputs");

		for (var module : this.modules) {
			module.periodic();
		}
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic");

		OdometryThread.getInstance().odometryLock.unlock();

		var sampleTimestamps = this.odometryTimestamps.timestamps;
		var modulePositions = new SwerveModulePosition[this.modules.length];
		for (int sampleI = 0; sampleI < sampleTimestamps.length; sampleI++) {
			for (int i = 0; i < this.modules.length; i++) {
				modulePositions[i] = this.modules[i].getModulePositionSamples()[sampleI];
			}

			if (this.lastMeasuredPositions != null) {
				RobotState.getInstance().addOdometryObservation(new OdometryObservation(
					sampleTimestamps[sampleI],
					(this.gyroInputs.connected) ? (
						Optional.of(this.gyroInputs.odometryGyroRotation[sampleI])
					) : (
						Optional.empty()
					),
					this.lastMeasuredPositions,
					modulePositions
				));
			} else {
				this.lastMeasuredPositions = new SwerveModulePosition[modulePositions.length];
			}
			for (int i = 0; i < modulePositions.length; i++) {
				this.lastMeasuredPositions[i] = new SwerveModulePosition(
					modulePositions[i].distanceMeters,
					modulePositions[i].angle
				);
			}
		}

		for (int i = 0; i < this.modules.length; i++) {
			this.measuredStates[i] = this.modules[i].getModuleState();
		}
		Logger.recordOutput("Drive/Swerve States/Measured", this.measuredStates);

		this.robotMeasuredSpeeds = DriveConstants.kinematics.toChassisSpeeds(this.measuredStates);
		if (this.gyroInputs.connected) {
			this.robotMeasuredSpeeds.omegaRadiansPerSecond = this.gyroInputs.yawVelocityRadsPerSec;
		}

		Logger.recordOutput("Drive/Chassis Speeds/Measured", this.robotMeasuredSpeeds);
		// RobotState.getInstance().addDriveMeasurement(this.gyroAngle, this.getModulePositions());

		this.fieldMeasuredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this.robotMeasuredSpeeds, RobotState.getInstance().getEstimatedGlobalPose().getRotation());

		// Skid Detection
		// SwerveModuleState[] rotationalStates = new SwerveModuleState[DriveConstants.modules.length];
		// SwerveModuleState[] translationalStates = new SwerveModuleState[DriveConstants.modules.length];

		// for (int i = 0; i < DriveConstants.modules.length; i++) {
		//     var rotationalState = new SwerveModuleState(-gyroInputs.yawVelocity.in(RadiansPerSecond) * DriveConstants.modules[i].moduleTranslation.getNorm(), MathExtraUtil.rotationFromVector(DriveConstants.modules[i].positiveRotVec));
		//     var rotational = new Translation2d(rotationalState.speedMetersPerSecond, rotationalState.angle);
		//     rotationalStates[i] = rotationalState;
		//     var measured = new Translation2d(measuredStates[i].speedMetersPerSecond, measuredStates[i].angle);
		//     var translational = measured.minus(rotational);
		//     translationalStates[i] = new SwerveModuleState(translational.getNorm(), translational.getAngle());
		// }

		// Logger.recordOutput("Drive/SwerveStates/Rotational States", rotationalStates);
		// Logger.recordOutput("Drive/SwerveStates/Translational States", translationalStates);

		// var minTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).min().orElse(0);
		// var maxTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).max().orElse(0);
		// var averageTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).average().orElse(0);
		// var maxDistanceFromAverage = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).map((a) -> averageTranslational - a).map(Math::abs).average().orElse(0);
		// Logger.recordOutput("Drive/Skid Detection/Min Translational Speed", minTranslational);
		// Logger.recordOutput("Drive/Skid Detection/Max Translational Speed", maxTranslational);
		// Logger.recordOutput("Drive/Skid Detection/MaxMin Ratio", maxTranslational / minTranslational);
		// Logger.recordOutput("Drive/Skid Detection/Largest From Average", maxDistanceFromAverage);

		// Clearing log fields
		Logger.recordOutput("Drive/Chassis Speeds/Setpoint", emptySpeeds);
		Logger.recordOutput("Drive/Swerve States/Setpoints", emptyStates);
		Logger.recordOutput("Drive/Swerve States/Setpoints Optimized", emptyStates);

		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Clear Log Fields");
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive");
	}

	public void postCommandPeriodic() {
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Drive/Before");
		// if (DriverStation.isDisabled()) {
		//     // TODO: UNCOMMENT IF DRIVE MOVES WITH NO COMMAND AFTER ENABLE
		//     // for (var module : modules) {
		//     //     module.stop();
		//     // }
		// } else
		if (this.translationSubsystem.needsPostProcessing || this.rotationalSubsystem.needsPostProcessing) {
			this.runRobotSpeeds(this.desiredRobotSpeeds);
		}
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Drive/Periodic");
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Drive");
	}

	public void runSetpoints(SwerveModuleState... states) {
		this.translationSubsystem.needsPostProcessing = false;
		this.rotationalSubsystem.needsPostProcessing = false;
		this.setpointStates = states;
		Logger.recordOutput("Drive/Swerve States/Setpoints", this.setpointStates);
		IntStream.range(0, this.modules.length).forEach((i) -> this.modules[i].runSetpoint(this.setpointStates[i]));
		Logger.recordOutput("Drive/Swerve States/Setpoints Optimized", this.setpointStates);
	}

	private static final LoggedTunable<LinearAcceleration> forwardAccelLimitTunable = LoggedTunable.from("Drive/Accel Limits/Forward Accel Limit", MetersPerSecondPerSecond::of, 5000);
	private static final LoggedTunable<LinearAcceleration> skidAccelLimitTunable = LoggedTunable.from("Drive/Accel Limits/Skid Accel Limit", MetersPerSecondPerSecond::of, 60);

	public static final LoggedTunable<TiltAccelerationLimits> normalTiltLimitTunable = LoggedTunable.from("Drive/Accel Limits/Tilt Limits/Normal", new TiltAccelerationLimits(500, 500, 500, 500));
	public static final LoggedTunable<TiltAccelerationLimits> extendedTiltLimitTunable = LoggedTunable.from("Drive/Accel Limits/Tilt Limits/Extended", new TiltAccelerationLimits(10, 12, 20, 20));
	private TiltAccelerationLimits tiltLimits = normalTiltLimitTunable.get();
	public void setTiltLimits(TiltAccelerationLimits tiltLimits) {
		this.tiltLimits = tiltLimits;
	}
	public void runRobotSpeeds(ChassisSpeeds robotSpeeds) {
		this.runRobotSpeeds(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, robotSpeeds.omegaRadiansPerSecond);
	}
	public void runRobotSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadsPerSecond) {
		this.desiredRobotSpeeds.vxMetersPerSecond = vxMetersPerSecond;
		this.desiredRobotSpeeds.vyMetersPerSecond = vyMetersPerSecond;
		this.desiredRobotSpeeds.omegaRadiansPerSecond = omegaRadsPerSecond;
		Logger.recordOutput("Drive/Chassis Speeds/Desired Speed", this.desiredRobotSpeeds);

		var desiredDelta = this.desiredRobotSpeeds.minus(this.robotMeasuredSpeeds);
		var desiredAccel = desiredDelta.div(RobotConstants.rioUpdatePeriodSecs);
		Logger.recordOutput("Drive/Chassis Speeds/Desired Delta", desiredDelta);
		Logger.recordOutput("Drive/Chassis Speeds/Desired Accel", desiredAccel);

		var limitedAccel = desiredAccel;

		// Forward Accel Limit
		var maxMeasuredModuleSpeed = Math.hypot(this.robotMeasuredSpeeds.vxMetersPerSecond, this.robotMeasuredSpeeds.vyMetersPerSecond) + Math.abs(this.robotMeasuredSpeeds.omegaRadiansPerSecond * DriveConstants.driveBaseRadius.in(Meters));
		var maxDesiredModuleAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond) + Math.abs(limitedAccel.omegaRadiansPerSecond * DriveConstants.driveBaseRadius.in(Meters));
		var forwardAccelLimit = /* (1 - (maxMeasuredModuleSpeed / DriveConstants.maxModuleSpeed.in(MetersPerSecond))) *  */forwardAccelLimitTunable.get().in(MetersPerSecondPerSecond);
		var forwardAccelLimitingFactor = forwardAccelLimit / Math.max(maxDesiredModuleAccel, forwardAccelLimit);
		Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Max Measured Module Speed", maxMeasuredModuleSpeed);
		Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Max Desired Module Accel", maxDesiredModuleAccel);
		Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Forward Accel Limit", forwardAccelLimit);
		Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Limiting Factor", forwardAccelLimitingFactor);
		limitedAccel = limitedAccel.times(forwardAccelLimitingFactor);

		// Tilt Accel Limit
		var desiredTiltAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond);
		var desiredTiltAccelHeading = Math.atan2(limitedAccel.vyMetersPerSecond, limitedAccel.vxMetersPerSecond);
		var tiltAccelLimit = this.tiltLimits.getMaxTiltAccelerationMPSS(desiredTiltAccelHeading);
		var tiltAccelLimitingFactor = tiltAccelLimit / Math.max(desiredTiltAccel, tiltAccelLimit);
		Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Desired Tilt Accel", desiredTiltAccel);
		Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Desired Tilt Accel Heading", desiredTiltAccelHeading);
		Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Tilt Accel Limit", tiltAccelLimit);
		Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Limiting Factor", tiltAccelLimitingFactor);
		limitedAccel = new ChassisSpeeds(
			limitedAccel.vxMetersPerSecond * tiltAccelLimitingFactor,
			limitedAccel.vyMetersPerSecond * tiltAccelLimitingFactor,
			limitedAccel.omegaRadiansPerSecond
		);

		// Skid Accel Limit
		var desiredSkidAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond);
		var skidAccelLimit = skidAccelLimitTunable.get().in(MetersPerSecondPerSecond);
		var skidAccelLimitingFactor = skidAccelLimit / Math.max(desiredSkidAccel, skidAccelLimit);
		Logger.recordOutput("Drive/Chassis Speeds/Skid Limit/Desired Skid Accel", desiredSkidAccel);
		Logger.recordOutput("Drive/Chassis Speeds/Skid Limit/Skid Accel Limit", skidAccelLimit);
		Logger.recordOutput("Drive/Chassis Speeds/Skid Limit/Limiting Factor", skidAccelLimitingFactor);
		limitedAccel = new ChassisSpeeds(
			limitedAccel.vxMetersPerSecond * skidAccelLimitingFactor,
			limitedAccel.vyMetersPerSecond * skidAccelLimitingFactor,
			limitedAccel.omegaRadiansPerSecond
		);


		var limitedDelta = limitedAccel.times(RobotConstants.rioUpdatePeriodSecs);
		var limitedSpeeds = this.robotMeasuredSpeeds.plus(limitedDelta);
		// Logger.recordOutput("Drive/Chassis Speeds/Limiting Factor", limitingFactor);
		Logger.recordOutput("Drive/Chassis Speeds/Limited Accel", limitedAccel);
		Logger.recordOutput("Drive/Chassis Speeds/Limited Delta", limitedDelta);
		Logger.recordOutput("Drive/Chassis Speeds/Limited Speed", limitedSpeeds);

		ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(limitedSpeeds, rotationCorrection.get());
		this.setpointStates = DriveConstants.kinematics.toSwerveModuleStates(correctedSpeeds, this.centerOfRotation);
		SwerveDriveKinematics.desaturateWheelSpeeds(this.setpointStates, DriveConstants.maxDriveSpeed);
		this.runSetpoints(this.setpointStates);
	}
	public void runFieldSpeeds(ChassisSpeeds fieldSpeeds) {
		this.runRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, RobotState.getInstance().getEstimatedGlobalPose().getRotation()));
	}


	public Command coast() {
		final var drive = this;
		return new Command() {
			{
				this.addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);
				this.setName("Coast");
			}
			@Override
			public void initialize() {
				for (var module : drive.modules) {
					module.stopDrive(NeutralMode.COAST);
				}
			}
			@Override
			public void end(boolean interrupted) {
				for (var module : drive.modules) {
					module.stopDrive(NeutralMode.DEFAULT);
				}
			}
			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}

	// public Command followBlueTrajectory(Trajectory<SwerveSample> trajectory) {
	//     return new FollowTrajectoryCommand(trajectory);
	// }
	// public Command followExactTrajectory(Trajectory<SwerveSample> trajectory) {
	//     return new FollowTrajectoryCommand(trajectory);
	// }

	public void setCenterOfRotation(Translation2d cor) {
		this.centerOfRotation = cor;
		Logger.recordOutput("Drive/Center of Rotation", RobotState.getInstance().getEstimatedGlobalPose().transformBy(new Transform2d(this.centerOfRotation, Rotation2d.kZero)));
	}

	/** Stops the drive. */
	public void stop() {
		for (var module : this.modules) {
			module.stopDrive(NeutralMode.DEFAULT);
			module.stopTurn(NeutralMode.DEFAULT);
		}
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement.
	 * The modules will
	 * return to their normal orientations the next time a nonzero velocity is
	 * requested.
	 */
	public void stopWithX() {
		for (var module : this.modules) {
			module.stopDrive(NeutralMode.DEFAULT);
			// module.config.moduleTranslation;
		}
		IntStream.range(0, DriveConstants.moduleConstants.length).forEach((i) -> {
			this.setpointStates[i] = new SwerveModuleState(
				0,
				DriveConstants.moduleConstants[i].moduleTranslation.getAngle()
			);
		});
	}

	/** Returns the current pitch velocity (Y rotation) in radians per second. */
	public double getYawVelocityRadsPerSec() {
		return this.gyroInputs.yawVelocityRadsPerSec;
	}

	/** Returns the current pitch velocity (Y rotation) in radians per second. */
	public double getPitchVelocityRadsPerSec() {
		return this.gyroInputs.pitchVelocityRadsPerSec;
	}

	/** Returns the current roll velocity (X rotation) in radians per second. */
	public double getRollVelocityRadsPerSec() {
		return this.gyroInputs.rollVelocityRadsPerSec;
	}

	/** Returns an array of module positions. */
	public SwerveModulePosition[] getModulePositions() {
		return this.lastMeasuredPositions;
	}

	public ChassisSpeeds getRobotMeasuredSpeeds() {
		return this.robotMeasuredSpeeds;
	}

	public ChassisSpeeds getFieldMeasuredSpeeds() {
		return this.fieldMeasuredSpeeds;
	}

	public final Translational translationSubsystem;
	public static class Translational extends SubsystemBase {
		public final Drive drive;
		private boolean needsPostProcessing = false;

		private Translational(Drive drive) {
			this.drive = drive;
			this.setName("Drive/Translational");
		}

		public void driveVelocity(double vx, double vy) {
			this.needsPostProcessing = true;
			this.drive.desiredRobotSpeeds.vxMetersPerSecond = vx;
			this.drive.desiredRobotSpeeds.vyMetersPerSecond = vy;
		}
		public void driveVelocity(ChassisSpeeds speeds) {
			this.driveVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
		}

		public void stop() {
			this.needsPostProcessing = false;
			this.driveVelocity(0.0, 0.0);
		}

		public Command fieldRelative(Supplier<ChassisSpeeds> speeds) {
			final var translational = this;
			return new Command() {
				{
					this.addRequirements(translational);
					this.setName("Field Relative");
				}
				@Override
				public void execute() {
					translational.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), RobotState.getInstance().getEstimatedGlobalPose().getRotation()));
				}
				@Override
				public void end(boolean interrupted) {
					translational.stop();
				}
			};
		}

		public Command simplePIDTo(Supplier<Translation2d> target) {
			final var translational = this;
			return new Command() {
				private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
					"Drive/Translational/Simple PID",
					new PIDConstants(
						3.0,
						0.0,
						0.0
					)
				);
				private final PIDController pid = new PIDController(pidConsts.get().kP(), pidConsts.get().kI(), pidConsts.get().kD());

				{
					this.addRequirements(translational);
					this.setName("Simple PID To");
				}

				@Override
				public void execute() {
					var targetPose = target.get();
					var errX = targetPose.getX() - RobotState.getInstance().getEstimatedGlobalPose().getTranslation().getX();
					var errY = targetPose.getY() - RobotState.getInstance().getEstimatedGlobalPose().getTranslation().getY();

					var errDist = Math.hypot(errX, errY);

					var pidOut = -this.pid.calculate(errDist, 0.0);

					var pidX = errX / errDist * pidOut;
					var pidY = errY / errDist * pidOut;

					var fieldRotation = RobotState.getInstance().getEstimatedGlobalPose().getRotation();

					var pidRobotX = pidX * +fieldRotation.getCos() + pidY * +fieldRotation.getSin();
					var pidRobotY = pidX * -fieldRotation.getSin() + pidY * +fieldRotation.getCos();

					translational.driveVelocity(pidRobotX, pidRobotY);
				}

				@Override
				public void end(boolean interrupted) {
					translational.stop();
				}
			};
		}
	}
	public final Rotational rotationalSubsystem;
	public static class Rotational extends SubsystemBase {
		public final Drive drive;
		private boolean needsPostProcessing = false;

		private Rotational(Drive drive) {
			this.drive = drive;
			this.setName("Drive/Rotational");
		}

		public void driveVelocity(double omega) {
			this.needsPostProcessing = true;
			this.drive.desiredRobotSpeeds.omegaRadiansPerSecond = omega;
		}
		public void driveVelocity(ChassisSpeeds speeds) {
			this.driveVelocity(speeds.omegaRadiansPerSecond);
		}
		public void stop() {
			this.needsPostProcessing = false;
			this.driveVelocity(0);
		}

		public Command spin(DoubleSupplier omega) {
			return Commands.runEnd(() -> driveVelocity(omega.getAsDouble()), this::stop, this);
		}

		// public Command defenseSpin(Joystick joystick) {
		//     var subsystem = this;
		//     return new Command() {
		//         {
		//             this.addRequirements(subsystem);
		//             this.setName("Defense Spin");
		//         }
		//         private static final LoggedTunableNumber defenseSpinLinearThreshold = LoggedTunable.from("Drive/Defense Spin Linear Threshold", 0.125);
		//         private static final Matrix<N2, N2> perpendicularMatrix =
		//             MatBuilder.fill(
		//                 Nat.N2(), Nat.N2(),
		//                 +0,-1,
		//                 +1,+0
		//             )
		//         ;
		//         @Override
		//         public void execute() {
		//             // Leds.getInstance().defenseSpin.setFlag(true);
		//             var joyVec = Perspective.getCurrent().toField(joystick.toVector());
		//             var desiredLinear = VecBuilder.fill(drive.desiredRobotSpeeds.vxMetersPerSecond, drive.desiredRobotSpeeds.vyMetersPerSecond);
		//             var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.desiredRobotSpeeds, RobotState.getInstance().getEstimatedGlobalPose().getRotation());
		//             var perpendicularLinear = new Vector<N2>(perpendicularMatrix.times(
		//                 VecBuilder.fill(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
		//             ));
		//             var dot = joystick.x().getAsDouble();
		//             if(desiredLinear.norm() > defenseSpinLinearThreshold.get()) {
		//                 dot = -joyVec.dot(perpendicularLinear);
		//             }
		//             var omega = dot
		//                 * DriveConstants.maxTurnRate.in(RadiansPerSecond)
		//                 * maxTurnRateEnvCoef.getAsDouble() * 0.25
		//             ;
		//             driveVelocity(omega);
		//             if(desiredLinear.norm() <= defenseSpinLinearThreshold.get()) {
		//                 drive.setCenterOfRotation(new Translation2d());
		//                 return;
		//             }
		//             var rotateAround = GeomUtil.vectorFromRotation(
		//                 GeomUtil.rotationFromVector(desiredLinear)
		//                 // .plus(Rotation2d.fromDegrees(45 * Math.signum(velo)))
		//             );
		//             drive.setCenterOfRotation(
		//                 Arrays.stream(DriveConstants.moduleTranslations)
		//                 .map((t) -> new Translation2d(t.toVector().unit().times(RobotConstants.centerToBumperCorner.in(Meters))))
		//                 .sorted((a, b) ->
		//                     (int) Math.signum(
		//                         b.toVector().unit().dot(rotateAround) - a.toVector().unit().dot(rotateAround)
		//                     )
		//                 )
		//                 .findFirst()
		//                 .orElse(new Translation2d())
		//             );
		//         }
		//         @Override
		//         public void end(boolean interrupted) {
		//             stop();
		//             drive.setCenterOfRotation(new Translation2d());
		//             // Leds.getInstance().defenseSpin.setFlag(false);
		//         }
		//     };
		// }

		private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
			"Drive/Rotational/PID",
			new PIDConstants(
				0.2,
				0.0,
				0.0
			)
		);
		private static final LoggedTunable<Angle> headingTolerance = LoggedTunable.from("Drive/Rotational/Heading Tolerance", Degrees::of, 1.0);
		private static final LoggedTunable<AngularVelocity> omegaTolerance = LoggedTunable.from("Drive/Rotational/Heading Tolerance", DegreesPerSecond::of, 1.0);

		public Command pidControlledOptionalHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
			var subsystem = this;
			return new Command() {
				private final ProfiledPIDController headingPID = new ProfiledPIDController(
					pidConsts.get().kP(),
					pidConsts.get().kI(),
					pidConsts.get().kD(),
					new Constraints(
						DriveConstants.maxTurnRate.in(RadiansPerSecond),
						5000
					)
				);

				{
					this.addRequirements(subsystem);
					this.setName("PID Controlled Heading");
					this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
					this.headingPID.setTolerance(headingTolerance.get().in(Radians), omegaTolerance.get().in(RadiansPerSecond));
				}

				private Rotation2d desiredHeading;
				private boolean headingSet;

				@Override
				public void initialize() {
					this.desiredHeading = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
					this.headingPID.reset(this.desiredHeading.getRadians());
					this.headingPID.setTolerance(headingTolerance.get().in(Radians), omegaTolerance.get().in(RadiansPerSecond));
					pidConsts.get().update(this.headingPID);
				}

				@Override
				public void execute() {
					var heading = headingSupplier.get();
					this.headingSet = heading.isPresent();
					heading.ifPresent((r) -> desiredHeading = r);
					double turnInput = this.headingPID.calculate(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(), this.desiredHeading.getRadians());
					turnInput = this.headingPID.atSetpoint() ? 0 : turnInput + this.headingPID.getSetpoint().velocity;
					turnInput = MathUtil.clamp(
						turnInput,
						-0.5 * maxTurnRateEnvCoef.get().getAsDouble(),
						+0.5 * maxTurnRateEnvCoef.get().getAsDouble()
					);
					driveVelocity(turnInput * DriveConstants.maxTurnRate.in(RadiansPerSecond));
				}

				@Override
				public void end(boolean interrupted) {
					stop();
				}

				@Override
				public boolean isFinished() {
					return !headingSet && headingPID.atSetpoint();
				}
			};
		}
		public Command pidControlledHeading(Supplier<Rotation2d> headingSupplier) {
			var subsystem = this;
			return new Command() {
				private final ProfiledPIDController headingPID = new ProfiledPIDController(
					pidConsts.get().kP(),
					pidConsts.get().kI(),
					pidConsts.get().kD(),
					new Constraints(
						DriveConstants.maxTurnRate.in(RadiansPerSecond),
						5000
					)
				);

				{
					this.addRequirements(subsystem);
					this.setName("PID Controlled Heading");
					this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
					this.headingPID.setTolerance(headingTolerance.get().in(Radians), omegaTolerance.get().in(RadiansPerSecond));
				}

				@Override
				public void initialize() {
					this.headingPID.reset(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians());
					this.headingPID.setTolerance(headingTolerance.get().in(Radians), omegaTolerance.get().in(RadiansPerSecond));
					pidConsts.get().update(this.headingPID);
				}

				@Override
				public void execute() {
					var desiredHeading = headingSupplier.get();
					double turnInput = this.headingPID.calculate(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(), desiredHeading.getRadians());
					turnInput = this.headingPID.atSetpoint() ? 0 : turnInput + this.headingPID.getSetpoint().velocity;
					turnInput = MathUtil.clamp(
						turnInput,
						-0.5 * maxTurnRateEnvCoef.get().getAsDouble(),
						+0.5 * maxTurnRateEnvCoef.get().getAsDouble()
					);
					driveVelocity(turnInput * DriveConstants.maxTurnRate.in(RadiansPerSecond));
				}

				@Override
				public void end(boolean interrupted) {
					stop();
				}
			};
		}

		// public Command headingFromJoystick(Joystick joystick, Rotation2d[] snapPoints, Supplier<Rotation2d> forwardDirectionSupplier) {
		//     return pidControlledOptionalHeading(
		//         new LazyOptional<Rotation2d>() {
		//             private final Timer preciseTurnTimer = new Timer();
		//             private final double preciseTurnTimeThreshold = 0.5;
		//             private Optional<Rotation2d> outputMap(Rotation2d i) {
		//                 return Optional.of(i.minus(forwardDirectionSupplier.get()));
		//             }
		//             @Override
		//             public Optional<Rotation2d> get() {
		//                 if(joystick.magnitude() == 0) {
		//                     preciseTurnTimer.restart();
		//                     return Optional.empty();
		//                 }
		//                 var joyHeading = GeomUtil.rotationFromVector(Perspective.getCurrent().toField(joystick.toVector()));
		//                 if(preciseTurnTimer.hasElapsed(preciseTurnTimeThreshold)) {
		//                     return outputMap(joyHeading);
		//                 }
		//                 int smallestDistanceIndex = 0;
		//                 double smallestDistance = Double.MAX_VALUE;
		//                 for(int i = 0; i < snapPoints.length; i++) {
		//                     var dist = Math.abs(joyHeading.minus(AllianceFlipUtil.apply(snapPoints[i])).getRadians());
		//                     if(dist < smallestDistance) {
		//                         smallestDistance = dist;
		//                         smallestDistanceIndex = i;
		//                     }
		//                 }
		//                 return outputMap(AllianceFlipUtil.apply(snapPoints[smallestDistanceIndex]));
		//             }
		//         }
		//     );
		// }

		public Command pointTo(Supplier<Optional<Translation2d>> posToPointTo, Supplier<Rotation2d> forward) {
			return pidControlledOptionalHeading(
				() -> posToPointTo.get().map((pointTo) -> {
					var FORR = pointTo.minus(RobotState.getInstance().getEstimatedGlobalPose().getTranslation());
					return new Rotation2d(FORR.getX(), FORR.getY()).minus(forward.get());
				})
			);
		}
	}
}
