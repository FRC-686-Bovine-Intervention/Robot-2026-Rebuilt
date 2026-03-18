package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.util.FFGains;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.VirtualSubsystem;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.Root;

public class Drive extends VirtualSubsystem {
	private static final LoggedTunable<Time> discretizationCorrection = LoggedTunable.from("Drive/Discretization Correction", Seconds::of, 0.125);
	public static final Supplier<DoubleSupplier> maxDriveSpeedEnvCoef = Environment.switchVar(
		() -> 1.0,
		new LoggedNetworkNumber("Demo Constraints/Max Translational Percentage", 0.25)::get
	);
	public static final Supplier<DoubleSupplier> maxTurnRateEnvCoef = Environment.switchVar(
		() -> 1.0,
		new LoggedNetworkNumber("Demo Constraints/Max Rotational Percentage", 0.5)::get
	);

	private static final LoggedTunable<FFGains> translationalFFGains = LoggedTunable.from("Drive/FF/Translational",
		new FFGains(
			0.14097,
			0,
			1.9406,
			0.042101
		)
	);
	private static final LoggedTunable<FFGains> rotationalFFGains = LoggedTunable.from("Drive/FF/Rotational",
		new FFGains(
			0,
			0,
			0.91433,
			0.25931
		)
	);
	private final SimpleMotorFeedforward translationalFF = translationalFFGains.get().update(new SimpleMotorFeedforward(0.0, 0.0, 0.0));
	private final SimpleMotorFeedforward rotationalFF = rotationalFFGains.get().update(new SimpleMotorFeedforward(0.0, 0.0, 0.0));

	public final Module[] modules = new Module[DriveConstants.moduleConstants.length];

	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	private final OdometryTimestampIO odometryTimestampIO;
	private final OdometryTimestampIOInputsAutoLogged odometryTimestamps = new OdometryTimestampIOInputsAutoLogged();

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

		var moduleInitialPositions = new double[DriveConstants.moduleConstants.length];
		var translationalRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				null,
				null,
				(state) -> {
					Logger.recordOutput("SysID/Drive/Translational/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(volts) -> {
					for (var module : this.modules) {
						module.runVolts(volts.in(Volts), Rotation2d.kZero);
					}
				},
				(log) -> {
					var position = 0.0;
					var velocity = 0.0;
					var voltage = 0.0;
					for (int i = 0; i < this.modules.length; i++) {
						position += (this.modules[i].getWheelLinearPositionMeters() - moduleInitialPositions[i]) / DriveConstants.moduleConstants.length;
						velocity += this.modules[i].getWheelLinearVelocityMetersPerSec() / DriveConstants.moduleConstants.length;
						voltage += this.modules[i].getDriveAppliedVolts() / DriveConstants.moduleConstants.length;
					}
					Logger.recordOutput("SysID/Drive/Translational/Position", position, Meters);
					Logger.recordOutput("SysID/Drive/Translational/Velocity", velocity, MetersPerSecond);
					Logger.recordOutput("SysID/Drive/Translational/Voltage", voltage, Volts);
				},
				this.translationSubsystem
			)
		);
		var rotationalRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				null,
				null,
				(state) -> {
					Logger.recordOutput("SysID/Drive/Rotational/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(volts) -> {
					for (var module : this.modules) {
						module.runVolts(volts.in(Volts), module.config.positiveRotVec);
					}
				},
				(log) -> {
					var position = 0.0;
					var velocity = 0.0;
					var voltage = 0.0;
					for (int i = 0; i < this.modules.length; i++) {
						position += (this.modules[i].getWheelLinearPositionMeters() - moduleInitialPositions[i]) / this.modules[i].config.moduleTransform.getTranslation().getNorm() / DriveConstants.moduleConstants.length;
						velocity += this.modules[i].getWheelLinearVelocityMetersPerSec() / this.modules[i].config.moduleTransform.getTranslation().getNorm() / DriveConstants.moduleConstants.length;
						voltage += this.modules[i].getDriveAppliedVolts() / DriveConstants.moduleConstants.length;
					}
					Logger.recordOutput("SysID/Drive/Rotational/Position", position, Radians);
					Logger.recordOutput("SysID/Drive/Rotational/Velocity", velocity, RadiansPerSecond);
					Logger.recordOutput("SysID/Drive/Rotational/Voltage", voltage, Volts);
				},
				this.rotationalSubsystem
			)
		);

		Function<Command, Command> translationalCommandMap = (sysIDCommand) ->
			Commands.sequence(
				this.translationSubsystem.run(() -> {
					for (var module : this.modules) {
						module.runVolts(0.0, Rotation2d.kZero);
					}
				}).withTimeout(0.5),
				Commands.runOnce(() -> {
					for (int i = 0; i < DriveConstants.moduleConstants.length; i++) {
						moduleInitialPositions[i] = this.modules[i].getWheelLinearPositionMeters();
					}
				}),
				sysIDCommand
			)
			.alongWith(this.rotationalSubsystem.idle())
			.finallyDo(() -> {
				for (var module : this.modules) {
					module.stopDrive(NeutralMode.BRAKE);
				}
			})
			.withName("SysID Translational")
		;
		SmartDashboard.putData("SysID/Drive/Translational/Quasi Forward", translationalCommandMap.apply(translationalRoutine.quasistatic(SysIdRoutine.Direction.kForward)));
		SmartDashboard.putData("SysID/Drive/Translational/Quasi Reverse", translationalCommandMap.apply(translationalRoutine.quasistatic(SysIdRoutine.Direction.kReverse)));
		SmartDashboard.putData("SysID/Drive/Translational/Dynamic Forward", translationalCommandMap.apply(translationalRoutine.dynamic(SysIdRoutine.Direction.kForward)));
		SmartDashboard.putData("SysID/Drive/Translational/Dynamic Reverse", translationalCommandMap.apply(translationalRoutine.dynamic(SysIdRoutine.Direction.kReverse)));
		Function<Command, Command> rotationalCommandMap = (sysIDCommand) ->
			Commands.sequence(
				this.rotationalSubsystem.run(() -> {
					for (var module : this.modules) {
						module.runVolts(0.0, module.config.positiveRotVec);
					}
				}).withTimeout(0.5),
				Commands.runOnce(() -> {
					for (int i = 0; i < DriveConstants.moduleConstants.length; i++) {
						moduleInitialPositions[i] = this.modules[i].getWheelLinearPositionMeters();
					}
				}),
				sysIDCommand
			)
			.alongWith(this.translationSubsystem.idle())
			.finallyDo(() -> {
				for (var module : this.modules) {
					module.stopDrive(NeutralMode.BRAKE);
				}
			})
			.withName("SysID Rotational")
		;
		SmartDashboard.putData("SysID/Drive/Rotational/Quasi Forward", rotationalCommandMap.apply(rotationalRoutine.quasistatic(SysIdRoutine.Direction.kForward)));
		SmartDashboard.putData("SysID/Drive/Rotational/Quasi Reverse", rotationalCommandMap.apply(rotationalRoutine.quasistatic(SysIdRoutine.Direction.kReverse)));
		SmartDashboard.putData("SysID/Drive/Rotational/Dynamic Forward", rotationalCommandMap.apply(rotationalRoutine.dynamic(SysIdRoutine.Direction.kForward)));
		SmartDashboard.putData("SysID/Drive/Rotational/Dynamic Reverse", rotationalCommandMap.apply(rotationalRoutine.dynamic(SysIdRoutine.Direction.kReverse)));
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

		var sampleCount = this.odometryTimestamps.odometrySampleCount;
		var modulePositions = new SwerveModulePosition[this.modules.length];
		for (int sampleI = 0; sampleI < sampleCount; sampleI++) {
			for (int i = 0; i < this.modules.length; i++) {
				modulePositions[i] = this.modules[i].getModulePositionSamples()[sampleI];
			}

			if (this.lastMeasuredPositions != null) {
				RobotState.getInstance().addOdometryObservation(new OdometryObservation(
					this.odometryTimestamps.timestamps[sampleI],
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
		Logger.recordOutput("Subsystems/Drive/Swerve States/Measured", this.measuredStates);

		this.robotMeasuredSpeeds = DriveConstants.kinematics.toChassisSpeeds(this.measuredStates);
		if (this.gyroInputs.connected) {
			this.robotMeasuredSpeeds.omegaRadiansPerSecond = this.gyroInputs.yawVelocityRadsPerSec;
		}

		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Measured", this.robotMeasuredSpeeds);

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

		// Logger.recordOutput("Subsystems/Drive/SwerveStates/Rotational States", rotationalStates);
		// Logger.recordOutput("Subsystems/Drive/SwerveStates/Translational States", translationalStates);

		// var minTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).min().orElse(0);
		// var maxTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).max().orElse(0);
		// var averageTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).average().orElse(0);
		// var maxDistanceFromAverage = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).map((a) -> averageTranslational - a).map(Math::abs).average().orElse(0);
		// Logger.recordOutput("Subsystems/Drive/Skid Detection/Min Translational Speed", minTranslational);
		// Logger.recordOutput("Subsystems/Drive/Skid Detection/Max Translational Speed", maxTranslational);
		// Logger.recordOutput("Subsystems/Drive/Skid Detection/MaxMin Ratio", maxTranslational / minTranslational);
		// Logger.recordOutput("Subsystems/Drive/Skid Detection/Largest From Average", maxDistanceFromAverage);

		// Clearing log fields
		// Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Setpoint", emptySpeeds);
		// Logger.recordOutput("Subsystems/Drive/Swerve States/Setpoints", emptyStates);
		// Logger.recordOutput("Subsystems/Drive/Swerve States/Setpoints Optimized", emptyStates);

		// LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Clear Log Fields");
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
		} else if (this.translationSubsystem.getCurrentCommand() == null && this.rotationalSubsystem.getCurrentCommand() == null) {
			this.stop();
		}
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Drive/Periodic");
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Drive");
	}

	public void calculateFieldVelocity() {
		this.fieldMeasuredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this.robotMeasuredSpeeds, RobotState.getInstance().getEstimatedGlobalPose().getRotation());
	}

	public void runSetpoints(SwerveModuleState... states) {
		this.translationSubsystem.needsPostProcessing = false;
		this.rotationalSubsystem.needsPostProcessing = false;
		this.setpointStates = states;
		Logger.recordOutput("Subsystems/Drive/Swerve States/Setpoints", this.setpointStates);
		for (int i = 0; i < this.modules.length; i++) {
			this.modules[i].runSetpoint(this.setpointStates[i]);
		}
		Logger.recordOutput("Subsystems/Drive/Swerve States/Setpoints Optimized", this.setpointStates);
	}

	private static final LoggedTunable<LinearAcceleration> forwardAccelLimitTunable = LoggedTunable.from("Subsystems/Drive/Accel Limits/Forward Accel Limit", MetersPerSecondPerSecond::of, 5000);
	private static final LoggedTunable<LinearAcceleration> skidAccelLimitTunable = LoggedTunable.from("Subsystems/Drive/Accel Limits/Skid Accel Limit", MetersPerSecondPerSecond::of, 60);

	public static final LoggedTunable<TiltAccelerationLimits> normalTiltLimitTunable = LoggedTunable.from(
		"Subsystems/Drive/Accel Limits/Tilt Limits/Normal",
		new TiltAccelerationLimits(
			500.0,
			500.0,
			500.0,
			500.0
		)
	);
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
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Desired Speed", this.desiredRobotSpeeds);

		var desiredDelta = this.desiredRobotSpeeds.minus(this.robotMeasuredSpeeds);
		var desiredAccel = desiredDelta.div(RobotConstants.rioUpdatePeriodSecs);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Desired Delta", desiredDelta);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Desired Accel", desiredAccel);

		var limitedAccel = desiredAccel;

		// Forward Accel Limit
		var maxMeasuredModuleSpeed = Math.hypot(this.robotMeasuredSpeeds.vxMetersPerSecond, this.robotMeasuredSpeeds.vyMetersPerSecond) + Math.abs(this.robotMeasuredSpeeds.omegaRadiansPerSecond * DriveConstants.driveBaseRadius.in(Meters));
		var maxDesiredModuleAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond) + Math.abs(limitedAccel.omegaRadiansPerSecond * DriveConstants.driveBaseRadius.in(Meters));
		var forwardAccelLimit = /* (1 - (maxMeasuredModuleSpeed / DriveConstants.maxModuleSpeed.in(MetersPerSecond))) *  */forwardAccelLimitTunable.get().in(MetersPerSecondPerSecond);
		var forwardAccelLimitingFactor = forwardAccelLimit / Math.max(maxDesiredModuleAccel, forwardAccelLimit);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Forward Limit/Max Measured Module Speed", maxMeasuredModuleSpeed);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Forward Limit/Max Desired Module Accel", maxDesiredModuleAccel);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Forward Limit/Forward Accel Limit", forwardAccelLimit);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Forward Limit/Limiting Factor", forwardAccelLimitingFactor);
		limitedAccel = limitedAccel.times(forwardAccelLimitingFactor);

		// Tilt Accel Limit
		var desiredTiltAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond);
		var desiredTiltAccelHeading = Math.atan2(limitedAccel.vyMetersPerSecond, limitedAccel.vxMetersPerSecond);
		var tiltAccelLimit = this.tiltLimits.getMaxTiltAccelerationMPSS(desiredTiltAccelHeading);
		var tiltAccelLimitingFactor = tiltAccelLimit / Math.max(desiredTiltAccel, tiltAccelLimit);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Tilt Limit/Desired Tilt Accel", desiredTiltAccel);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Tilt Limit/Desired Tilt Accel Heading", desiredTiltAccelHeading);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Tilt Limit/Tilt Accel Limit", tiltAccelLimit);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Tilt Limit/Limiting Factor", tiltAccelLimitingFactor);
		limitedAccel = new ChassisSpeeds(
			limitedAccel.vxMetersPerSecond * tiltAccelLimitingFactor,
			limitedAccel.vyMetersPerSecond * tiltAccelLimitingFactor,
			limitedAccel.omegaRadiansPerSecond
		);

		// Skid Accel Limit
		var desiredSkidAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond);
		var skidAccelLimit = skidAccelLimitTunable.get().in(MetersPerSecondPerSecond);
		var skidAccelLimitingFactor = skidAccelLimit / Math.max(desiredSkidAccel, skidAccelLimit);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Skid Limit/Desired Skid Accel", desiredSkidAccel);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Skid Limit/Skid Accel Limit", skidAccelLimit);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Skid Limit/Limiting Factor", skidAccelLimitingFactor);
		limitedAccel = new ChassisSpeeds(
			limitedAccel.vxMetersPerSecond * skidAccelLimitingFactor,
			limitedAccel.vyMetersPerSecond * skidAccelLimitingFactor,
			limitedAccel.omegaRadiansPerSecond
		);


		var limitedDelta = limitedAccel.times(RobotConstants.rioUpdatePeriodSecs);
		var limitedSpeeds = this.robotMeasuredSpeeds.plus(limitedDelta);
		// Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Limiting Factor", limitingFactor);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Limited Accel", limitedAccel);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Limited Delta", limitedDelta);
		Logger.recordOutput("Subsystems/Drive/Chassis Speeds/Limited Speed", limitedSpeeds);

		ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(limitedSpeeds, discretizationCorrection.get().in(Seconds));
		this.setpointStates = DriveConstants.kinematics.toSwerveModuleStates(correctedSpeeds, this.centerOfRotation);
		SwerveDriveKinematics.desaturateWheelSpeeds(this.setpointStates, DriveConstants.maxDriveSpeed);
		this.runSetpoints(this.setpointStates);
	}

	public void runRobotSpeeds(
		double vXMetersPerSecond,
		double vYMetersPerSecond,
		double omegaRadiansPerSecond,
		double aXMetersPerSecondSqr,
		double aYMetersPerSecondSqr,
		double alphaRadiansPerSecondSqr
	) {
		this.translationSubsystem.needsPostProcessing = false;
		this.rotationalSubsystem.needsPostProcessing = false;
		for (var module : this.modules) {
			var moduleVXMPS = vXMetersPerSecond + omegaRadiansPerSecond * module.config.moduleTransform.getTranslation().getNorm() * module.config.moduleTransform.getRotation().getCos();
			var moduleVYMPS = vYMetersPerSecond + omegaRadiansPerSecond * module.config.moduleTransform.getTranslation().getNorm() * module.config.moduleTransform.getRotation().getSin();

			var moduleAXMPSS = aXMetersPerSecondSqr + alphaRadiansPerSecondSqr * module.config.moduleTransform.getTranslation().getNorm() * module.config.moduleTransform.getRotation().getCos();
			var moduleAYMPSS = aYMetersPerSecondSqr + alphaRadiansPerSecondSqr * module.config.moduleTransform.getTranslation().getNorm() * module.config.moduleTransform.getRotation().getSin();

			var robotFFXVolts = vXMetersPerSecond * translationalFF.getKv() + aXMetersPerSecondSqr * translationalFF.getKa();
			var robotFFYVolts = vYMetersPerSecond * translationalFF.getKv() + aYMetersPerSecondSqr * translationalFF.getKa();
			var robotFFRotVolts = omegaRadiansPerSecond * rotationalFF.getKv() + alphaRadiansPerSecondSqr * rotationalFF.getKa();

			var moduleFFXVolts = robotFFXVolts + robotFFRotVolts * module.config.moduleTransform.getRotation().getCos();
			var moduleFFYVolts = robotFFYVolts + robotFFRotVolts * module.config.moduleTransform.getRotation().getSin();

			module.runSetpoint(
				moduleVXMPS,
				moduleVYMPS,
				moduleAXMPSS,
				moduleAYMPSS,
				moduleFFXVolts,
				moduleFFYVolts
			);
		}

		Logger.recordOutput("Drive/FF Drive/Module Velos", this.modules[0].driveVelo, this.modules[1].driveVelo, this.modules[2].driveVelo, this.modules[3].driveVelo);
		Logger.recordOutput("Drive/FF Drive/Module Accels", this.modules[0].driveAccel, this.modules[1].driveAccel, this.modules[2].driveAccel, this.modules[3].driveAccel);
		Logger.recordOutput("Drive/FF Drive/Module FFs", this.modules[0].driveFF, this.modules[1].driveFF, this.modules[2].driveFF, this.modules[3].driveFF);
		Logger.recordOutput("Drive/FF Drive/Module Scaled Velos", this.modules[0].driveScaledVelo, this.modules[1].driveScaledVelo, this.modules[2].driveScaledVelo, this.modules[3].driveScaledVelo);
		Logger.recordOutput("Drive/FF Drive/Module Scaled Accels", this.modules[0].driveScaledAccel, this.modules[1].driveScaledAccel, this.modules[2].driveScaledAccel, this.modules[3].driveScaledAccel);
		Logger.recordOutput("Drive/FF Drive/Module Scaled FFs", this.modules[0].driveScaledFF, this.modules[1].driveScaledFF, this.modules[2].driveScaledFF, this.modules[3].driveScaledFF);
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
		Logger.recordOutput("Subsystems/Drive/Center of Rotation", RobotState.getInstance().getEstimatedGlobalPose().transformBy(new Transform2d(this.centerOfRotation, Rotation2d.kZero)));
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
		for (int i = 0; i < this.modules.length; i++) {
			this.modules[i].runSetpoint(new SwerveModuleState(0.0, this.modules[i].config.moduleTransform.getTranslation().getAngle()));
		}
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
			if (!this.drive.rotationalSubsystem.needsPostProcessing) {
				this.drive.stop();
			}
		}
		public void cancelPostProcessing() {
			this.needsPostProcessing = false;
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
				private static final LoggedTunable<PIDGains> pidGains = LoggedTunable.from(
					"Subsystems/Drive/Translational/Simple PID",
					new PIDGains(
						3.0,
						0.0,
						0.0
					)
				);
				private final PIDController pid = pidGains.get().update(new PIDController(0.0, 0.0, 0.0));

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
			this.driveVelocity(0.0);
			if (!this.drive.translationSubsystem.needsPostProcessing) {
				this.drive.stop();
			}
		}
		public void cancelPostProcessing() {
			this.needsPostProcessing = false;
		}

		public Command spin(DoubleSupplier omega) {
			return Commands.runEnd(() -> driveVelocity(omega.getAsDouble()), this::stop, this);
		}

		public Command genHeadingPIDCommand(String name, LoggedTunable<PIDGains> pidGains, DoubleSupplier measuredHeadingRadsSupplier, DoubleSupplier targetHeadingRadsSupplier) {
			final var rotational = this;
			return new Command() {
				private final PIDController pid = new PIDController(pidGains.get().kP(), pidGains.get().kI(), pidGains.get().kD());

				{
					this.setName(name);
					this.addRequirements(rotational);

					this.pid.enableContinuousInput(-Math.PI, Math.PI);
				}

				@Override
				public void initialize() {
					if (pidGains.hasChanged(this.hashCode())) {
						pidGains.get().update(this.pid);
					}
				}

				@Override
				public void execute() {
					var measuredHeadingRads = measuredHeadingRadsSupplier.getAsDouble();
					var targetHeadingRads = targetHeadingRadsSupplier.getAsDouble();
					rotational.driveVelocity(this.pid.calculate(measuredHeadingRads, targetHeadingRads));
				}

				@Override
				public void end(boolean interrupted) {
					rotational.stop();
				}
			};
		}

		// public Command defenseSpin(Joystick joystick) {
		//     var subsystem = this;
		//     return new Command() {
		//         {
		//             this.addRequirements(subsystem);
		//             this.setName("Defense Spin");
		//         }
		//         private static final LoggedTunableNumber defenseSpinLinearThreshold = LoggedTunable.from("Subsystems/Drive/Defense Spin Linear Threshold", 0.125);
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

		private static final LoggedTunable<PIDGains> pidGains = LoggedTunable.from(
			"Subsystems/Drive/Rotational/PID",
			new PIDGains(
				0.2,
				0.0,
				0.0
			)
		);
		private static final LoggedTunable<Constraints> profileConstraints = LoggedTunable.from(
			"Subsystems/Drive/Rotational/Profile",
			new Constraints(
				DriveConstants.maxTurnRate.in(RadiansPerSecond),
				5000
			)
		);
		private static final LoggedTunable<Angle> headingTolerance = LoggedTunable.from("Subsystems/Drive/Rotational/Heading Tolerance", Degrees::of, 1.0);
		private static final LoggedTunable<AngularVelocity> omegaTolerance = LoggedTunable.from("Subsystems/Drive/Rotational/Heading Tolerance", DegreesPerSecond::of, 1.0);

		public Command pidControlledOptionalHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
			var subsystem = this;
			return new Command() {
				private final ProfiledPIDController headingPID = pidGains.get().update(new ProfiledPIDController(0.0, 0.0, 0.0, profileConstraints.get()));

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
					if (pidGains.hasChanged(this.hashCode())) {
						pidGains.get().update(this.headingPID);
					}
					if (profileConstraints.hasChanged(this.hashCode())) {
						this.headingPID.setConstraints(profileConstraints.get());
					}
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
					pidGains.get().kP(),
					pidGains.get().kI(),
					pidGains.get().kD(),
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
					pidGains.get().update(this.headingPID);
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
