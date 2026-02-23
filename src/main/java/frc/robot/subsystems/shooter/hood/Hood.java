package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.angle.ArmMech;
import lombok.Getter;

public class Hood extends SubsystemBase {
	private final HoodIO io;
	private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

	private static final LoggedTunable<Angle> stowAngle = LoggedTunable.from("Subsystems/Shooter/Hood/Commands/Stow/Target Angle", Degrees::of, HoodConstants.minAngle.in(Degrees));
	private static final LoggedTunable<Angle> stowPulldownThreshold = LoggedTunable.from("Subsystems/Shooter/Hood/Commands/Stow/Pulldown Threshold", Degrees::of, HoodConstants.minAngle.plus(Degrees.of(1.0)).in(Degrees));
	private static final LoggedTunable<Voltage> stowPulldownVoltage = LoggedTunable.from("Subsystems/Shooter/Hood/Commands/Stow/Pulldown Voltage", Volts::of, -1.0);

	private static final LoggedTunable<Voltage> calibrationVoltage = LoggedTunable.from("Subsystems/Shooter/Hood/Commands/Calibration/Voltage", Volts::of, -2.0);

	private static final LoggedTunableNumber profilekV = LoggedTunable.from("Subsystems/Shooter/Hood/Mechanism/Profile/kV", 24.0);
	private static final LoggedTunableNumber profilekA = LoggedTunable.from("Subsystems/Shooter/Hood/Mechanism/Profile/kA", 24.0);
	private static final LoggedTunable<AngularVelocity> profileMaxVel = LoggedTunable.from("Subsystems/Shooter/Hood/Mechanism/Profile/Max Velocity", DegreesPerSecond::of, 0.0);

	private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
		"Subsystems/Shooter/Hood/Mechanism/FF",
		new FFConstants(
			0.0,
			0.0,
			2.4,
			0.0
		)
	);

	private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
		"Subsystems/Shooter/Hood/Mechanism/PID",
		new PIDConstants(
			1.5,
			0.0,
			0.0
		)
	);

	@Getter
	private double measuredAngleRads = 0.0;
	@Getter
	private double measuredVelocityRadsPerSec = 0.0;

	@Getter
	private double setpointAngleRads = 0.0;
	@Getter
	private double setpointVelocityRadsPerSec = 0.0;

	@Getter
	private boolean calibrated = false;

	public final ArmMech mech = new ArmMech(HoodConstants.hoodBase);

	private final Alert notCalibratedAlert = new Alert("Subsystems/Shooter/Hood/Alerts", "Not Calibrated", AlertType.kError);
	private final Alert notCalibratedGlobalAlert = new Alert("Hood Not Calibrated!", AlertType.kError);

	private final Alert motorDisconnectedAlert = new Alert("Subsystems/Shooter/Hood/Alerts", "Motor Disconnected", AlertType.kError);
	private final Alert motorDisconnectedGlobalAlert = new Alert("Hood Motor Disconnected!", AlertType.kError);

	public Hood(HoodIO io) {
		super("Shooter/Hood");

		System.out.println("[Init Hood] Instantiating Hood with " + io.getClass().getSimpleName());
		this.io = io;

		final var sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(1.0).per(Second),
				Volts.of(7.0),
				Seconds.of(10.0),
				(state) -> {
					Logger.recordOutput("SysID/Shooter-Hood/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(voltage) -> {
					this.io.setVolts(voltage.in(Volts));
				},
				(log) -> {
					Logger.recordOutput("SysID/Shooter-Hood/Voltage", this.inputs.motor.motor.getAppliedVolts());
					Logger.recordOutput("SysID/Shooter-Hood/Position Rads", this.getMeasuredAngleRads());
					Logger.recordOutput("SysID/Shooter-Hood/Velocity RadsPerSec", this.getMeasuredVelocityRadsPerSec());
				},
				this,
				"shooter-hood"
			)
		);
		SmartDashboard.putData("SysID/Shooter/Hood/Quasi Forward", sysidRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> this.getMeasuredAngleRads() >= HoodConstants.maxAngle.in(Radians)));
		SmartDashboard.putData("SysID/Shooter/Hood/Quasi Reverse", sysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> this.getMeasuredAngleRads() <= HoodConstants.minAngle.in(Radians)));
		SmartDashboard.putData("SysID/Shooter/Hood/Dynamic Forward", sysidRoutine.dynamic(SysIdRoutine.Direction.kForward).until(() -> this.getMeasuredAngleRads() >= HoodConstants.maxAngle.in(Radians)));
		SmartDashboard.putData("SysID/Shooter/Hood/Dynamic Reverse", sysidRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> this.getMeasuredAngleRads() <= HoodConstants.minAngle.in(Radians)));

		this.periodic();
	}

	@Override
	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Update Inputs");
		Logger.processInputs("Inputs/Shooter/Hood", this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Process Inputs");

		if (this.inputs.limitSwitch) {
			this.calibrated = true;
		}

		this.measuredAngleRads = HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
		this.measuredVelocityRadsPerSec = HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec());

		this.setpointAngleRads = HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motorProfilePositionRads);
		this.setpointVelocityRadsPerSec = HoodConstants.motorToMechanism.applyUnsigned(this.inputs.motorProfileVelocityRadsPerSec);

		Logger.recordOutput("Subsystems/Shooter/Hood/Angle/Measured", this.getMeasuredAngleRads(), Radians);
		Logger.recordOutput("Subsystems/Shooter/Hood/Velocity/Measured", this.getMeasuredVelocityRadsPerSec(), RadiansPerSecond);
		Logger.recordOutput("Subsystems/Shooter/Hood/Angle/Setpoint", this.getSetpointAngleRads(), Radians);
		Logger.recordOutput("Subsystems/Shooter/Hood/Velocity/Setpoint", this.getSetpointVelocityRadsPerSec(), RadiansPerSecond);

		this.mech.setRads(-this.getMeasuredAngleRads());

		var configChanged = false;
		if (
			Hood.profilekV.hasChanged(this.hashCode())
			| Hood.profilekA.hasChanged(this.hashCode())
			| Hood.profileMaxVel.hasChanged(this.hashCode())
		) {
			this.io.configProfile(
				Hood.profilekV.getAsDouble(),
				Hood.profilekA.getAsDouble(),
				Hood.profileMaxVel.get().in(RadiansPerSecond)
			);
			configChanged = true;
		}
		if (Hood.ffConsts.hasChanged(this.hashCode())) {
			this.io.configFF(Hood.ffConsts.get());
			configChanged = true;
		}
		if (Hood.pidConsts.hasChanged(this.hashCode())) {
			this.io.configPID(Hood.pidConsts.get());
			configChanged = true;
		}
		if (configChanged) {
			this.io.configSend();
		}

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

		this.notCalibratedAlert.set(!this.isCalibrated());
		this.notCalibratedGlobalAlert.set(!this.isCalibrated());

		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood");
	}

	public Command coast() {
		final var hood = this;
		return new Command() {
			{
				this.setName("Coast");
				this.addRequirements(hood);
			}

			@Override
			public void initialize() {
				hood.io.stop(NeutralMode.COAST);
			}

			@Override
			public void end(boolean interrupted) {
				hood.io.stop(NeutralMode.DEFAULT);
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}

	public Command calibrate() {
		final var hood = this;
		return new Command() {
			{
				this.setName("Calibrate");
				this.addRequirements(hood);
			}

			@Override
			public void execute() {
				hood.io.setVolts(Hood.calibrationVoltage.get().in(Volts));
			}

			@Override
			public void end(boolean interrupted) {
				hood.io.stop(NeutralMode.DEFAULT);
			}

			@Override
			public InterruptionBehavior getInterruptionBehavior() {
				return InterruptionBehavior.kCancelIncoming;
			}
		};
	}

	public Command stow() {
		final var hood = this;
		return new Command() {
			{
				this.setName("Stow");
				this.addRequirements(hood);
			}

			@Override
			public void execute() {
				if (hood.getMeasuredAngleRads() <= Hood.stowPulldownThreshold.get().in(Radians)) {
					hood.io.setVolts(Hood.stowPulldownVoltage.get().in(Volts));
				} else {
					hood.io.setPositionRads(Hood.stowAngle.get().in(Radians));
				}
			}

			@Override
			public void end(boolean interrupted) {
				hood.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command genAngleCommand(String name, DoubleSupplier goalAngleRadsSupplier) {
		final var hood = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(hood);
			}

			@Override
			public void execute() {
				var goalAngleRads = goalAngleRadsSupplier.getAsDouble();
				hood.io.setPositionRads(goalAngleRads);
				Logger.recordOutput("Subsystems/Shooter/Hood/Angle/Goal", goalAngleRads, Radians);
			}

			@Override
			public void end(boolean interrupted) {
				hood.io.stop(NeutralMode.DEFAULT);
			}
		};
	}
}
