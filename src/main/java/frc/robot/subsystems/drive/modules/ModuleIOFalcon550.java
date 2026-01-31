package frc.robot.subsystems.drive.modules;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.drive.odometry.OdometryThread;
import frc.robot.subsystems.drive.odometry.OdometryThread.DoubleBuffer;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class ModuleIOFalcon550 implements ModuleIO {
	protected final TalonFX driveMotor;
	protected final SparkMax azimuthMotor;
	protected final AbsoluteEncoder azimuthAbsoluteEncoder;
	protected final SparkClosedLoopController azimuthClosedLoopController;
	private final SparkMaxConfig azimuthConfig;

	private final BaseStatusSignal[] refreshSignals;
	private final BaseStatusSignal[] driveMotorConnectedSignals;
	private final EncodedMotorStatusSignalCache driveMotorStatusSignalCache;

	private final DoubleBuffer drivePositionBuffer;
	private final DoubleBuffer azimuthPositionBuffer;

	private final VoltageOut driveVolts = new VoltageOut(0);
	private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final CoastOut coastOutRequest = new CoastOut();
	private final StaticBrake staticBrakeRequest = new StaticBrake();

	public ModuleIOFalcon550(ModuleConstants config) {
		this.driveMotor = config.driveMotorID.talonFX();
		this.azimuthMotor = config.azimuthMotorID.sparkMax(MotorType.kBrushless);
		this.azimuthAbsoluteEncoder = this.azimuthMotor.getAbsoluteEncoder();
		this.azimuthClosedLoopController = this.azimuthMotor.getClosedLoopController();
		this.azimuthConfig = new SparkMaxConfig();

		var driveConfig = new TalonFXConfiguration();
		driveConfig.MotorOutput
			.withInverted(config.driveInverted)
			.withNeutralMode(NeutralModeValue.Coast)
		;
		driveConfig.ClosedLoopRamps
			.withVoltageClosedLoopRampPeriod(Seconds.of(0.075))
		;
		// driveConfig.OpenLoopRamps
		//     .withVoltageOpenLoopRampPeriod(Seconds.of(0.1875))
		// ;
		driveConfig.CurrentLimits
			// .withSupplyCurrentLimit(Amps.of(70))
			// .withSupplyCurrentLowerLimit(Amps.of(70))
			// .withSupplyCurrentLowerTime(Seconds.of(0))
			.withSupplyCurrentLimitEnable(true)
			// .withStatorCurrentLimit(Amps.of(80))
			// .withStatorCurrentLimitEnable(true)
		;

		this.driveMotor.getConfigurator().apply(driveConfig);

		this.azimuthConfig
			.idleMode(IdleMode.kCoast)
			.inverted(false)
			.smartCurrentLimit(40)
		;
		this.azimuthConfig.absoluteEncoder
			.zeroOffset(config.encoderZeroOffset.in(Rotations))
			.inverted(true)
		;
		this.azimuthConfig.signals
			.absoluteEncoderPositionPeriodMs((int) DriveConstants.odometryLoopFrequency.asPeriod().in(Milliseconds))
		;
		this.azimuthConfig.closedLoop
			.positionWrappingEnabled(true)
			.positionWrappingMinInput(0.0)
			.positionWrappingMaxInput(1.0)
			.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
			.allowedClosedLoopError(Units.degreesToRotations(1.0), ClosedLoopSlot.kSlot0)
		;

		this.azimuthMotor.configure(this.azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		this.driveMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.driveMotor);
		this.refreshSignals = new BaseStatusSignal[] {
			this.driveMotorStatusSignalCache.encoder().position(),
			this.driveMotorStatusSignalCache.encoder().velocity(),
			this.driveMotorStatusSignalCache.motor().appliedVoltage(),
			this.driveMotorStatusSignalCache.motor().statorCurrent(),
			this.driveMotorStatusSignalCache.motor().supplyCurrent(),
			this.driveMotorStatusSignalCache.motor().torqueCurrent(),
			this.driveMotorStatusSignalCache.motor().deviceTemperature(),
		};
		this.driveMotorConnectedSignals = new BaseStatusSignal[] {
			this.driveMotorStatusSignalCache.encoder().position(),
			this.driveMotorStatusSignalCache.encoder().velocity(),
			this.driveMotorStatusSignalCache.motor().appliedVoltage(),
			this.driveMotorStatusSignalCache.motor().statorCurrent(),
			this.driveMotorStatusSignalCache.motor().supplyCurrent(),
			this.driveMotorStatusSignalCache.motor().torqueCurrent(),
			this.driveMotorStatusSignalCache.motor().deviceTemperature(),
		};

		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.driveMotorStatusSignalCache.encoder().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.driveMotorStatusSignalCache.motor().getStatusSignals());
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.driveMotor));
		BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.driveMotor));
		BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.odometryLoopFrequency, this.driveMotorStatusSignalCache.encoder().position());
		this.driveMotor.optimizeBusUtilization();

		this.drivePositionBuffer = OdometryThread.getInstance().registerPhoenixDoubleSignal(this.driveMotorStatusSignalCache.encoder().position(), Units::rotationsToRadians);
		this.azimuthPositionBuffer = OdometryThread.getInstance().registerGenericDoubleSignal(this.azimuthAbsoluteEncoder::getPosition, Units::rotationsToRadians);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.refreshSignals);
		inputs.driveMotorConnected = BaseStatusSignal.isAllGood(this.driveMotorConnectedSignals);
		inputs.azimuthMotorConnected = true;
		inputs.azimuthEncoderConnected = true;
		inputs.driveMotor.updateFrom(this.driveMotorStatusSignalCache);
		inputs.azimuthMotor.updateFrom(this.azimuthMotor);
		inputs.azimuthEncoder.updateFrom(this.azimuthAbsoluteEncoder);

		inputs.odometryDriveRads = this.drivePositionBuffer.popAll();
		inputs.odometryAzimuthRads = this.azimuthPositionBuffer.popAll();
		// inputs.driveMotorFaults.updateFrom(this.driveMotor);
		// inputs.azimuthMotorFaults.updateFrom(this.azimuthMotor);
	}

	@Override
	public void setDriveVolts(double volts) {
		this.driveMotor.setControl(this.driveVolts.withOutput(volts));
	}

	@Override
	public void setDriveVelocityRadPerSec(double velocityRadPerSec, double accelerationRadPerSecSqr, double feedforwardVolts, boolean overrideWithBrakeMode) {
		this.driveMotor.setControl(this.driveVelocity
			.withVelocity(Units.radiansToRotations(velocityRadPerSec))
			.withAcceleration(Units.radiansToRotations(accelerationRadPerSecSqr))
			.withFeedForward(feedforwardVolts)
			.withOverrideBrakeDurNeutral(overrideWithBrakeMode)
		);
	}

	@Override
	public void setAzimuthVolts(double volts) {
		this.azimuthMotor.setVoltage(volts);
	}

	@Override
	public void setAzimuthAngleRads(double angleRads, double feedforwardVolts) {
		this.azimuthClosedLoopController.setSetpoint(
			Units.radiansToRotations(angleRads),
			ControlType.kPosition,
			ClosedLoopSlot.kSlot0,
			feedforwardVolts
		);
	}

	@Override
	public void stopDrive(Optional<NeutralMode> neutralMode) {
		var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
		this.driveMotor.setControl(controlRequest);
	}

	@Override
	public void stopAzimuth(Optional<NeutralMode> neutralMode) {
		//TODO Reimplement module turn brake mode
		// turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
		this.azimuthMotor.stopMotor();
	}

	@Override
	public void configDrivePID(PIDGains pidGains) {
		var config = new Slot0Configs();
		this.driveMotor.getConfigurator().refresh(config);
		pidGains.update(config);
		this.driveMotor.getConfigurator().apply(config);
	}

	@Override
	public void configAzimuthPID(PIDGains pidGains) {
		this.azimuthConfig.closedLoop
			.p(pidGains.kP())
			.i(pidGains.kI())
			.d(pidGains.kD())
		;
		this.azimuthMotor.configure(this.azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void clearDriveStickyFaults(long bitmask) {
		if (bitmask == DeviceFaults.noneMask) {return;}
		if (bitmask == DeviceFaults.allMask) {
			this.driveMotor.clearStickyFaults();
		} else {
			for (var faultType : FaultType.possibleTalonFXFaults) {
				if (faultType.isPartOf(bitmask)) {
					faultType.clearStickyFaultOn(this.driveMotor);
				}
			}
		}
	}

	@Override
	public void clearAzimuthStickyFaults(long bitmask) {
		if (bitmask == DeviceFaults.noneMask) {return;}
		this.azimuthMotor.clearFaults();
	}
}
