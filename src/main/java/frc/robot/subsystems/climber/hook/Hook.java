package frc.robot.subsystems.climber.hook;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.linear.ElevatorMech;
import lombok.Getter;

public class Hook extends SubsystemBase {
	private final HookIO io;
	private final HookIOInputsAutoLogged inputs = new HookIOInputsAutoLogged();

	private static final LoggedTunable<Distance> retractLength = LoggedTunable.from("Climber/Hook/Lengths/Retract Length", Inches::of, HookConstants.minLength.in(Inches));
	private static final LoggedTunable<Distance> deployLength = LoggedTunable.from("Climber/Hook/Lengths/Deploy Length", Inches::of, HookConstants.maxLength.in(Inches));
	private static final LoggedTunable<Distance> climbLength = LoggedTunable.from("Climber/Hook/Lengths/Climb Length", Inches::of, 8.0);

	private static final LoggedTunable<Voltage> calibrationVoltage = LoggedTunable.from("Climber/Hook/Calibration Voltage", Volts::of, -2.0);

	private static final LoggedTunableNumber unloadedProfilekV = LoggedTunable.from("Climber/Hook/Unloaded Profile/kV", 1.0);
	private static final LoggedTunableNumber unloadedProfilekA = LoggedTunable.from("Climber/Hook/Unloaded Profile/kA", 1.0);
	private static final LoggedTunable<LinearVelocity> unloadedProfileMaxVel = LoggedTunable.from("Climber/Hook/Unloaded Profile/Max Velocity", InchesPerSecond::of, 0.0);

	private static final LoggedTunableNumber climbingProfilekV = LoggedTunable.from("Climber/Hook/Climbing Profile/kV", 1.0);
	private static final LoggedTunableNumber climbingProfilekA = LoggedTunable.from("Climber/Hook/Climbing Profile/kA", 1.0);
	private static final LoggedTunable<LinearVelocity> climbingProfileMaxVel = LoggedTunable.from("Climber/Hook/Climbing Profile/Max Velocity", InchesPerSecond::of, 0.0);

	private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
		"Climber/Hook/FF",
		new FFConstants(
			0.0,
			0.0,
			1.0,
			0.0
		)
	);
	private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
		"Climber/Hook/PID",
		new PIDConstants(
			1.0,
			0.0,
			0.0
		)
	);

	@Getter
	private double measuredLengthMeters = 0.0;
	@Getter
	private double measuredVelocityMetersPerSec = 0.0;

	@Getter
	private double setpointLengthMeters = 0.0;
	@Getter
	private double setpointVelocityMetersPerSec = 0.0;

	@Getter
	private boolean calibrated = false;

	public final ElevatorMech mech = new ElevatorMech(HookConstants.hookBase);

	private final Alert notCalibratedAlert = new Alert("Climber/Hook/Alerts", "Not Calibrated", AlertType.kError);
	private final Alert notCalibratedGlobalAlert = new Alert("Climber Hook Not Calibrated!", AlertType.kError);

	private final Alert motorDisconnectedAlert = new Alert("Climber/Hook/Alerts", "Motor Disconnected", AlertType.kError);
	private final Alert motorDisconnectedGlobalAlert = new Alert("Climber Hook Motor Disconnected!", AlertType.kError);

	public Hook(HookIO io) {
		super("Climber/Hook");

		System.out.println("[Init Hook] Instantiating Hook with " + io.getClass().getSimpleName());
		this.io = io;

		// this.io.configUnloadedProfile(
		// 	unloadedProfilekV.getAsDouble(),
		// 	unloadedProfilekA.getAsDouble(),
		// 	HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.metersToRadians(unloadedProfileMaxVel.get().in(MetersPerSecond)))
		// );
		// this.io.configClimbingProfile(
		// 	climbingProfilekV.getAsDouble(),
		// 	climbingProfilekA.getAsDouble(),
		// 	HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.metersToRadians(climbingProfileMaxVel.get().in(MetersPerSecond)))
		// );
		// this.io.configFF(ffConsts.get());
		// this.io.configPID(pidConsts.get());
		this.periodic();
	}


	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Hook/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Hook/Update Inputs");
		Logger.processInputs("Inputs/Climber/Hook", this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Hook/Process Inputs");

		if (this.inputs.limitSwitch) {
			this.calibrated = true;
		}
		Logger.recordOutput("Climber/Hook/Calibrated", this.isCalibrated());

		this.measuredLengthMeters = HookConstants.spool.radiansToMeters(HookConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads()));
		this.measuredVelocityMetersPerSec = HookConstants.spool.radiansToMeters(HookConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec()));

		this.setpointLengthMeters = HookConstants.spool.radiansToMeters(HookConstants.motorToMechanism.applyUnsigned(this.inputs.motorProfilePositionRads));
		this.setpointVelocityMetersPerSec = HookConstants.spool.radiansToMeters(HookConstants.motorToMechanism.applyUnsigned(this.inputs.motorProfileVelocityRadsPerSec));

		Logger.recordOutput("Climber/Hook/Length/Measured", this.getMeasuredLengthMeters(), Meters);
		Logger.recordOutput("Climber/Hook/Velocity/Measured", this.getMeasuredVelocityMetersPerSec(), MetersPerSecond);
		Logger.recordOutput("Climber/Hook/Length/Setpoint", this.getSetpointLengthMeters(), Meters);
		Logger.recordOutput("Climber/Hook/Velocity/Setpoint", this.getSetpointVelocityMetersPerSec(), MetersPerSecond);

		this.mech.setMeters(this.getMeasuredLengthMeters());

		if (LoggedTunable.hasChanged(this.hashCode(), unloadedProfilekV, unloadedProfilekA, unloadedProfileMaxVel)) {
			this.io.configUnloadedProfile(
				unloadedProfilekV.getAsDouble(),
				unloadedProfilekA.getAsDouble(),
				HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.metersToRadians(unloadedProfileMaxVel.get().in(MetersPerSecond)))
			);
		}
		if (LoggedTunable.hasChanged(this.hashCode(), climbingProfilekV, climbingProfilekA, climbingProfileMaxVel)) {
			this.io.configClimbingProfile(
				climbingProfilekV.getAsDouble(),
				climbingProfilekA.getAsDouble(),
				HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.metersToRadians(climbingProfileMaxVel.get().in(MetersPerSecond)))
			);
		}
		if (ffConsts.hasChanged(this.hashCode())) {
			this.io.configFF(ffConsts.get());
		}
		if (pidConsts.hasChanged(this.hashCode())) {
			this.io.configPID(pidConsts.get());
		}

		this.notCalibratedAlert.set(!this.isCalibrated());
		this.notCalibratedGlobalAlert.set(!this.isCalibrated());

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Hook/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Hook");
	}

	public Command coast() {
		final var hook = this;
		return new Command() {
			{
				this.setName("Coast");
				this.addRequirements(hook);
			}

			@Override
			public void initialize() {
				hook.io.stop(NeutralMode.COAST);
			}

			@Override
			public void end(boolean interrupted) {
				hook.io.stop(NeutralMode.DEFAULT);
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}

	public Command calibrate() {
		final var hook = this;
		return new Command() {
			{
				this.setName("Calibrate");
				this.addRequirements(hook);
			}

			@Override
			public void execute() {
				hook.io.setVolts(calibrationVoltage.get().in(Volts));
			}

			@Override
			public void end(boolean interrupted) {
				hook.io.stop(NeutralMode.DEFAULT);
			}

			@Override
			public boolean isFinished() {
				return hook.isCalibrated();
			}

			@Override
			public InterruptionBehavior getInterruptionBehavior() {
				return InterruptionBehavior.kCancelIncoming;
			}
		};
	}

	private Command genUnloadedCommand(String name, DoubleSupplier lengthMetersSupplier) {
		final var hook = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(hook);
			}

			@Override
			public void execute() {
				var goalLengthMeters = lengthMetersSupplier.getAsDouble();
				var goalAngleRads = HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.metersToRadians(goalLengthMeters));
				hook.io.setUnloadedPosition(goalAngleRads);
				Logger.recordOutput("Climber/Hook/Length/Goal", goalLengthMeters, Meters);
			}

			@Override
			public void end(boolean interrupted) {
				hook.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	private Command genClimbingCommand(String name, DoubleSupplier lengthMetersSupplier) {
		final var hook = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(hook);
			}

			@Override
			public void execute() {
				var goalLengthMeters = lengthMetersSupplier.getAsDouble();
				var goalAngleRads = HookConstants.motorToMechanism.inverse().applyUnsigned(HookConstants.spool.metersToRadians(goalLengthMeters));
				hook.io.setClimbingPosition(goalAngleRads);
				Logger.recordOutput("Climber/Hook/Length/Goal", goalLengthMeters, Meters);
			}

			@Override
			public void end(boolean interrupted) {
				hook.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command retract() {
		return this.genUnloadedCommand(
			"Retract",
			() -> retractLength.get().in(Meters)
		);
	}

	public Command deploy() {
		return this.genUnloadedCommand(
			"Deploy",
			() -> deployLength.get().in(Meters)
		);
	}

	public Command climb() {
		return this.genClimbingCommand(
			"Climb",
			() -> climbLength.get().in(Meters)
		);
	}
}
