package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
import frc.util.robotStructure.angle.ArmMech;
import lombok.Getter;

public class Hood extends SubsystemBase {
	private final HoodIO io;
	private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

	private static final LoggedTunable<Angle> idleAngle = LoggedTunable.from("Shooter/Hood/Idle Angle", Degrees::of, HoodConstants.minAngle.in(Degrees));
	private static final LoggedTunable<Voltage> calibrationVoltage = LoggedTunable.from("Shooter/Hood/Calibration Voltage", Volts::of, -2.0);

	private static final LoggedTunableNumber profilekV = LoggedTunable.from("Shooter/Hood/Profile/kV", 24.0);
	private static final LoggedTunableNumber profilekA = LoggedTunable.from("Shooter/Hood/Profile/kA", 24.0);
	private static final LoggedTunable<AngularVelocity> profileMaxVel = LoggedTunable.from("Shooter/Hood/Profile/Max Velocity", DegreesPerSecond::of, 0.0);

	private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
		"Shooter/Hood/FF",
		new FFConstants(
			0.0,
			0.0,
			2.4,
			0.0
		)
	);

	private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
		"Shooter/Hood/PID",
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

	private final Alert notCalibratedAlert = new Alert("Shooter/Hood/Alerts", "Not Calibrated", AlertType.kError);
	private final Alert notCalibratedGlobalAlert = new Alert("Hood Not Calibrated!", AlertType.kError);

	private final Alert motorDisconnectedAlert = new Alert("Shooter/Hood/Alerts", "Motor Disconnected", AlertType.kError);
	private final Alert motorDisconnectedGlobalAlert = new Alert("Hood Motor Disconnected!", AlertType.kError);

	public Hood(HoodIO io) {
		super("Shooter/Hood");

		System.out.println("[Init Hood] Instantiating Hood with " + io.getClass().getSimpleName());
		this.io = io;

		this.io.configPID(pidConsts.get());
		this.io.configFF(ffConsts.get());
		this.io.configProfile(profilekV.getAsDouble(), profilekA.getAsDouble(), profileMaxVel.get().in(RadiansPerSecond));
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

		Logger.recordOutput("Shooter/Hood/Angle/Measured", this.getMeasuredAngleRads(), Radians);
		Logger.recordOutput("Shooter/Hood/Velocity/Measured", this.getMeasuredVelocityRadsPerSec(), RadiansPerSecond);
		Logger.recordOutput("Shooter/Hood/Angle/Setpoint", this.getSetpointAngleRads(), Radians);
		Logger.recordOutput("Shooter/Hood/Velocity/Setpoint", this.getSetpointVelocityRadsPerSec(), RadiansPerSecond);

		this.mech.setRads(-this.getMeasuredAngleRads());

		if (pidConsts.hasChanged(hashCode())) {
			this.io.configPID(pidConsts.get());
		}

		if (ffConsts.hasChanged(hashCode())) {
			this.io.configFF(ffConsts.get());
		}

		if (profilekV.hasChanged(hashCode()) | profilekA.hasChanged(hashCode()) | profileMaxVel.hasChanged(hashCode())) {
			this.io.configProfile(profilekV.getAsDouble(), profilekA.getAsDouble(), profileMaxVel.get().in(RadiansPerSecond));
		}

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

		this.notCalibratedAlert.set(!this.isCalibrated());
		this.notCalibratedGlobalAlert.set(!this.isCalibrated());

		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Hood");
	}

	private void stop(Optional<NeutralMode> neutralMode) {
		this.io.stop(neutralMode);
	}

	private void setAngleGoalRads(double angleRads) {
		this.io.setPositionRads(
			HoodConstants.motorToMechanism.inverse().applyUnsigned(angleRads)
		);
		Logger.recordOutput("Shooter/Hood/Angle/Goal", angleRads);
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
				hood.stop(NeutralMode.COAST);
			}

			@Override
			public void end(boolean interrupted) {
				hood.stop(NeutralMode.DEFAULT);
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
				hood.io.setVolts(calibrationVoltage.get().in(Volts));
			}

			@Override
			public void end(boolean interrupted) {
				hood.stop(NeutralMode.DEFAULT);
			}

			@Override
			public InterruptionBehavior getInterruptionBehavior() {
				return InterruptionBehavior.kCancelIncoming;
			}
		};
	}

	public Command genAngleCommand(String name, DoubleSupplier angleRads) {
		final var hood = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(hood);
			}

			@Override
			public void execute() {
				hood.setAngleGoalRads(angleRads.getAsDouble());
			}

			@Override
			public void end(boolean interrupted) {
				hood.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command idle() {
		return this.genAngleCommand(
			"Idle",
			() -> idleAngle.get().in(Radians)
		);
	}
}
