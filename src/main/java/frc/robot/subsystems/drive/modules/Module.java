package frc.robot.subsystems.drive.modules;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import frc.util.FFGains;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Module {
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	public final ModuleConstants config;

	private double wheelAngularPositionRads = 0.0;
	private double wheelAngularVelocityRadsPerSec = 0.0;
	private final SwerveModulePosition modulePosition = new SwerveModulePosition();
	private final SwerveModuleState moduleState = new SwerveModuleState();

	private final SwerveModulePosition[] modulePositionSampleBuffer = new SwerveModulePosition[OdometryThread.MAX_BUFFER_SIZE];
	private SwerveModulePosition[] modulePositionSamples = new SwerveModulePosition[0];

	private static final LoggedTunable<LinearVelocity> brakeModeThreshold = LoggedTunable.from("Drive/Brake Mode Threshold", InchesPerSecond::of, 1);

	private static final LoggedTunable<PIDGains> drivePIDGains = LoggedTunable.from(
		"Drive/Module/Drive/PID",
		new PIDGains(
			0.1,
			0,
			0
		)
	);
	private static final LoggedTunable<PIDGains> azimuthPIDGains = LoggedTunable.from(
		"Drive/Module/Azimuth/PID",
		new PIDGains(
			5*2*Math.PI,
			0*2*Math.PI,
			0*2*Math.PI
		)
	);
	private final LoggedTunable<FFGains> azimuthFFGains;

	private final SimpleMotorFeedforward azimuthFF;
	private ExponentialProfile azimuthProfile;
	private ExponentialProfile.State azimuthGoalState = new ExponentialProfile.State();
	private ExponentialProfile.State azimuthSetpointState = new ExponentialProfile.State();

	// private final DeviceFaultAlerts driveMotorActiveFaultsAlert;
	// private final DeviceFaultAlerts driveMotorStickyFaultsAlert;
	// private final DeviceFaultAlerts azimuthMotorActiveFaultsAlert;
	// private final DeviceFaultAlerts azimuthMotorStickyFaultsAlert;
	// private final DeviceFaultClearer driveMotorStickyFaultClearer;
	// private final DeviceFaultClearer azimuthMotorStickyFaultClearer;

	private final Alert driveMotorDisconnectedAlert;
	private final Alert azimuthMotorDisconnectedAlert;
	private final Alert azimuthEncoderDisconnectedAlert;
	private final Alert driveMotorDisconnectedGlobalAlert;
	private final Alert azimuthMotorDisconnectedGlobalAlert;
	private final Alert azimuthEncoderDisconnectedGlobalAlert;

	public Module(ModuleIO io, ModuleConstants config) {
		this.io = io;
		this.config = config;

		this.io.configDrivePID(drivePIDGains.get());
		this.io.configAzimuthPID(azimuthPIDGains.get());

		for (int i = 0; i < this.modulePositionSampleBuffer.length; i++) {
			this.modulePositionSampleBuffer[i] = new SwerveModulePosition();
		}

		final var alertGroup = "Drive/Module " + this.config.name + "/Alerts";

		// this.driveMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Drive Motor has active faults: ", AlertType.kError));
		// this.driveMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Drive Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
		// this.azimuthMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Azimuth Motor has active faults: ", AlertType.kError));
		// this.azimuthMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Azimuth Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
		// this.driveMotorStickyFaultClearer = new DeviceFaultClearer("Drive/Module " + this.config.name + "/Drive Motor Sticky Faults");
		// this.azimuthMotorStickyFaultClearer = new DeviceFaultClearer("Drive/Module " + this.config.name + "/Azimuth Motor Sticky Faults");

		this.driveMotorDisconnectedAlert = new Alert(alertGroup, "Drive Motor Disconnected", AlertType.kError);
		this.azimuthMotorDisconnectedAlert = new Alert(alertGroup, "Azimuth Motor Disconnected", AlertType.kError);
		this.azimuthEncoderDisconnectedAlert = new Alert(alertGroup, "Azimuth Encoder Disconnected", AlertType.kError);
		this.driveMotorDisconnectedGlobalAlert = new Alert("Module " + this.config.name + " Drive Motor Disconnected!", AlertType.kError);
		this.azimuthMotorDisconnectedGlobalAlert = new Alert("Module " + this.config.name + " Azimuth Motor Disconnected!", AlertType.kError);
		this.azimuthEncoderDisconnectedGlobalAlert = new Alert("Module " + this.config.name + " Azimuth Encoder Disconnected!", AlertType.kError);

		this.azimuthFFGains = LoggedTunable.from(
			"Drive/Module/FF/" + this.config.name,
			this.config.azimuthFFGains
		);
		this.azimuthProfile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
			12.0,
			this.config.azimuthFFGains.kV(),
			this.config.azimuthFFGains.kA()
		));
		this.azimuthFF = this.config.azimuthFFGains.update(new SimpleMotorFeedforward(0.0, 0.0, 0.0));
	}

	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Update Inputs");
		Logger.processInputs("Inputs/Drive/Module " + this.config.name, this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Process Inputs");

		this.modulePositionSamples = new SwerveModulePosition[this.inputs.odometryDriveRads.length];
		for (int i = 0; i < this.modulePositionSamples.length; i++) {
			var angle = this.config.moduleTransform.getRotation().plus(
				Rotation2d.fromRadians(
					DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.odometryAzimuthRads[i])
				)
			);
			var distanceMeters = DriveConstants.wheel.radiansToMeters(DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.odometryDriveRads[i]));
			this.modulePositionSampleBuffer[i].distanceMeters = distanceMeters;
			this.modulePositionSampleBuffer[i].angle = angle;
		}
		System.arraycopy(this.modulePositionSampleBuffer, 0, this.modulePositionSamples, 0, this.modulePositionSamples.length);

		var angle = this.config.moduleTransform.getRotation().plus(
			Rotation2d.fromRadians(
				DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.azimuthEncoder.getPositionRads())
			)
		);
		this.modulePosition.angle = angle;
		this.moduleState.angle = angle;

		this.wheelAngularPositionRads = DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.driveMotor.encoder.getPositionRads());
		this.wheelAngularVelocityRadsPerSec = DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.driveMotor.encoder.getVelocityRadsPerSec());

		this.modulePosition.distanceMeters = DriveConstants.wheel.radiansToMeters(this.wheelAngularPositionRads);
		this.moduleState.speedMetersPerSecond = DriveConstants.wheel.radiansToMeters(this.wheelAngularVelocityRadsPerSec);

		if (drivePIDGains.hasChanged(this.hashCode())) {
			this.io.configDrivePID(drivePIDGains.get());
		}
		if (azimuthPIDGains.hasChanged(this.hashCode())) {
			this.io.configAzimuthPID(azimuthPIDGains.get());
		}
		if (azimuthFFGains.hasChanged(this.hashCode())) {
			azimuthFFGains.get().update(this.azimuthFF);
			this.azimuthProfile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(12.0, azimuthFFGains.get().kV(), azimuthFFGains.get().kA()));
		}

		// this.driveMotorActiveFaultsAlert.updateFrom(this.inputs.driveMotorFaults.activeFaults);
		// this.driveMotorStickyFaultsAlert.updateFrom(this.inputs.driveMotorFaults.stickyFaults);
		// this.azimuthMotorActiveFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.activeFaults);
		// this.azimuthMotorStickyFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.stickyFaults);
		// this.driveMotorStickyFaultClearer.clear(this.inputs.driveMotorFaults.stickyFaults, this.io::clearDriveStickyFaults, DeviceFaults.allMask);
		// this.azimuthMotorStickyFaultClearer.clear(this.inputs.azimuthMotorFaults.stickyFaults, this.io::clearAzimuthStickyFaults, DeviceFaults.allMask);

		this.driveMotorDisconnectedAlert.set(!this.inputs.driveMotorConnected);
		this.azimuthMotorDisconnectedAlert.set(!this.inputs.azimuthMotorConnected);
		this.azimuthEncoderDisconnectedAlert.set(!this.inputs.azimuthEncoderConnected);
		this.driveMotorDisconnectedGlobalAlert.set(!this.inputs.driveMotorConnected);
		this.azimuthMotorDisconnectedGlobalAlert.set(!this.inputs.azimuthMotorConnected);
		this.azimuthEncoderDisconnectedGlobalAlert.set(!this.inputs.azimuthEncoderConnected);

		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name);
	}

	/**
	 * Runs the module with the specified setpoint state. Must be called
	 * periodically.
	 */
	public void runSetpoint(SwerveModuleState setpoint) {
		setpoint.optimize(this.getAngle());

		var turnSetpoint = setpoint.angle;
		this.io.setAzimuthAngleRads(turnSetpoint.minus(this.config.moduleTransform.getRotation()).getRadians(), 0.0);

		setpoint.speedMetersPerSecond *= turnSetpoint.minus(this.getAngle()).getCos();

		var velocityRadPerSec = DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(DriveConstants.wheel.metersToRadians(setpoint.speedMetersPerSecond));

		var belowBrakeModeThreshold = Math.abs(setpoint.speedMetersPerSecond) < brakeModeThreshold.get().in(MetersPerSecond);

		this.io.setDriveVelocityRadPerSec(velocityRadPerSec, 0.0, setpoint.speedMetersPerSecond * 2.2, belowBrakeModeThreshold);
	}

	public void runSetpoint(double vXMetersPerSecond, double vYMetersPerSecond, double aXMetersPerSecondSqr, double aYMetersPerSecondSqr, double ffXVolts, double ffYVolts) {
		var currentModuleAngle = this.getAngle();
		var veloMagnitudeMPS = Math.hypot(vXMetersPerSecond, vYMetersPerSecond);
		var accelMagnitudeMPSS = Math.hypot(aXMetersPerSecondSqr, aYMetersPerSecondSqr);
		var feedforwardMagnitudeVolts = Math.hypot(ffXVolts, ffYVolts);
		double targetModuleAngleX;
		double targetModuleAngleY;
		if (veloMagnitudeMPS > 0.0) {
			targetModuleAngleX = vXMetersPerSecond / veloMagnitudeMPS;
			targetModuleAngleY = vYMetersPerSecond / veloMagnitudeMPS;
		} else if (feedforwardMagnitudeVolts > 0.0) {
			targetModuleAngleX = ffXVolts / feedforwardMagnitudeVolts;
			targetModuleAngleY = ffYVolts / feedforwardMagnitudeVolts;
		} else {
			targetModuleAngleX = currentModuleAngle.getCos();
			targetModuleAngleY = currentModuleAngle.getSin();
		}
		// Optimize target angle
		if (targetModuleAngleX * currentModuleAngle.getCos() + targetModuleAngleY * currentModuleAngle.getSin() < 0.0) {
			targetModuleAngleX *= -1.0;
			targetModuleAngleY *= -1.0;
		}

		var driveVeloMPS = vXMetersPerSecond * currentModuleAngle.getCos() + vYMetersPerSecond * currentModuleAngle.getSin();
		var driveAccelMPSS = aXMetersPerSecondSqr * currentModuleAngle.getCos() + aYMetersPerSecondSqr * currentModuleAngle.getSin();
		var driveFFVolts = ffXVolts * currentModuleAngle.getCos() + ffYVolts * currentModuleAngle.getSin();

		var driveVeloRadPerSec = DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(DriveConstants.wheel.metersToRadians(driveVeloMPS));
		var driveAccelRadPerSecSqr = DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(DriveConstants.wheel.metersToRadians(driveAccelMPSS));

		var belowBrakeModeThreshold = veloMagnitudeMPS < brakeModeThreshold.get().in(MetersPerSecond);

		var targetAzimuthAngleX = targetModuleAngleX * +this.config.moduleTransform.getRotation().getCos() + targetModuleAngleY * +this.config.moduleTransform.getRotation().getSin();
		var targetAzimuthAngleY = targetModuleAngleX * -this.config.moduleTransform.getRotation().getSin() + targetModuleAngleY * +this.config.moduleTransform.getRotation().getCos();

		this.io.setAzimuthAngleRads(Math.atan2(targetAzimuthAngleY, targetAzimuthAngleX), 0.0);
		this.io.setDriveVelocityRadPerSec(driveVeloRadPerSec, driveAccelRadPerSecSqr, driveFFVolts, belowBrakeModeThreshold);

		Logger.recordOutput("Drive/Module " + this.config.name + "/Target Angle", Rotation2d.fromRadians(Math.atan2(targetModuleAngleY, targetModuleAngleX)));
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive Velo MPS", driveVeloMPS, MetersPerSecond);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive Velo RadPerSec", driveVeloRadPerSec, RadiansPerSecond);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive Accel MPSS", driveAccelMPSS, MetersPerSecondPerSecond);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive Accel RadPerSecSqr", driveAccelRadPerSecSqr, RadiansPerSecondPerSecond);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive FF Volts", driveFFVolts, Volts);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive FFX Volts", ffXVolts, Volts);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive FFY Volts", ffYVolts, Volts);
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive FF angle", new Rotation2d(ffXVolts, ffYVolts));
		Logger.recordOutput("Drive/Module " + this.config.name + "/Drive Azimuth angle", new Rotation2d(targetAzimuthAngleX, targetAzimuthAngleY));

		this.driveVelo = new SwerveModuleState(
			veloMagnitudeMPS,
			new Rotation2d(vXMetersPerSecond, vYMetersPerSecond)
		);
		this.driveAccel = new SwerveModuleState(
			accelMagnitudeMPSS,
			new Rotation2d(aXMetersPerSecondSqr, aYMetersPerSecondSqr)
		);
		this.driveFF = new SwerveModuleState(
			feedforwardMagnitudeVolts,
			new Rotation2d(ffXVolts, ffYVolts)
		);
		this.driveScaledVelo = new SwerveModuleState(
			driveVeloMPS,
			new Rotation2d(vXMetersPerSecond, vYMetersPerSecond)
		);
		this.driveScaledAccel = new SwerveModuleState(
			driveAccelRadPerSecSqr,
			new Rotation2d(aXMetersPerSecondSqr, aYMetersPerSecondSqr)
		);
		this.driveScaledFF = new SwerveModuleState(
			driveFFVolts,
			new Rotation2d(ffXVolts, ffYVolts)
		);
	}

	public SwerveModuleState driveVelo = new SwerveModuleState();
	public SwerveModuleState driveAccel = new SwerveModuleState();
	public SwerveModuleState driveFF = new SwerveModuleState();
	public SwerveModuleState driveScaledVelo = new SwerveModuleState();
	public SwerveModuleState driveScaledAccel = new SwerveModuleState();
	public SwerveModuleState driveScaledFF = new SwerveModuleState();

	private void setAzimuthRobotHeadingRads(double azimuthRobotHeadingRads) {
		this.azimuthGoalState.position = azimuthRobotHeadingRads;
		this.azimuthGoalState.velocity = 0.0;
		var newSetpointState = this.azimuthProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.azimuthSetpointState, this.azimuthGoalState);
		var ffOut = this.azimuthFF.calculateWithVelocities(this.azimuthSetpointState.velocity, newSetpointState.velocity);
		this.io.setAzimuthAngleRads(azimuthRobotHeadingRads, azimuthRobotHeadingRads);

		// TODO Continuous azimuth wrapping for expo profile
	}

	/**
	 * Runs the module with the specified voltage
	 * Must be called periodically.
	 */
	public void runVolts(double volts, Rotation2d moduleAngle) {
		this.io.setAzimuthAngleRads(moduleAngle.minus(this.config.moduleTransform.getRotation()).getRadians(), 0.0);
		this.io.setDriveVolts(volts);
	}

	public void runDriveVolts(double volts) {
		this.io.setDriveVolts(volts);
	}
	public void runAzimuthVolts(double volts) {
		this.io.setAzimuthVolts(volts);
	}

	public void stopDrive(Optional<NeutralMode> neutralMode) {
		this.io.stopDrive(neutralMode);
	}
	public void stopTurn(Optional<NeutralMode> neutralMode) {
		this.io.stopAzimuth(neutralMode);
	}

	public SwerveModulePosition getModulePosition() {
		return this.modulePosition;
	}
	public SwerveModulePosition[] getModulePositionSamples() {
		return this.modulePositionSamples;
	}
	/** Returns the module state (turn angle and drive velocity). */
	public SwerveModuleState getModuleState() {
		return this.moduleState;
	}

	/** Returns the current turn angle of the module. */
	public Rotation2d getAngle() {
		return this.getModulePosition().angle;
	}

	/** Returns the current drive position of the module in radians. */
	public double getWheelAngularPositionRads() {
		return this.wheelAngularPositionRads;
	}
	/** Returns the drive velocity in radians/sec. */
	public double getWheelAngularVelocityRadsPerSec() {
		return this.wheelAngularVelocityRadsPerSec;
	}
	/** Returns the current drive position of the module in radians. */
	public double getWheelLinearPositionMeters() {
		return this.getModulePosition().distanceMeters;
	}
	/** Returns the drive velocity in radians/sec. */
	public double getWheelLinearVelocityMetersPerSec() {
		return this.getModuleState().speedMetersPerSecond;
	}

	public double getDriveAppliedVolts()        {return this.inputs.driveMotor.motor.getAppliedVolts();}
	public double getDriveStatorCurrentAmps()   {return this.inputs.driveMotor.motor.getStatorCurrentAmps();}
	public double getDriveSupplyCurrentAmps()   {return this.inputs.driveMotor.motor.getSupplyCurrentAmps();}
	public double getDriveTorqueCurrentAmps()   {return this.inputs.driveMotor.motor.getTorqueCurrentAmps();}
	public double getDriveTempCelsius()         {return this.inputs.driveMotor.motor.getDeviceTempCel();}

	public double getAzimuthAppliedVolts()      {return this.inputs.azimuthMotor.motor.getAppliedVolts();}
	public double getAzimuthStatorCurrentAmps() {return this.inputs.azimuthMotor.motor.getStatorCurrentAmps();}
	public double getAzimuthSupplyCurrentAmps() {return this.inputs.azimuthMotor.motor.getSupplyCurrentAmps();}
	public double getAzimuthTorqueCurrentAmps() {return this.inputs.azimuthMotor.motor.getTorqueCurrentAmps();}
	public double getAzimuthTempCelsius()       {return this.inputs.azimuthMotor.motor.getDeviceTempCel();}

	public double getAzimuthMotorCarriagePositionRads()       {return DriveConstants.azimuthMotorToCarriageRatio.applyUnsigned(this.inputs.azimuthMotor.encoder.getPositionRads());}
	public double getAzimuthMotorCarriageVelocityRadsPerSec() {return DriveConstants.azimuthMotorToCarriageRatio.applyUnsigned(this.inputs.azimuthMotor.encoder.getVelocityRadsPerSec());}
}
