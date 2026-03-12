package frc.robot.subsystems.shooter.flywheel;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelConfig;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import lombok.Getter;

public class Flywheel extends SubsystemBase {
	public final FlywheelConfig config;

	private final FlywheelIO io;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

	private static final LoggedTunable<LinearVelocity> spinupSurfaceVelo = LoggedTunable.from("Subsystems/Shooter/Flywheels/Commands/Spinup/Velocity", MetersPerSecond::of, +20.0);

	private static final LoggedTunable<LinearAcceleration> profileMaxAcceleration = LoggedTunable.from("Subsystems/Shooter/Flywheels/Mechanism/Profile/Max Acceleration", MetersPerSecondPerSecond::of, 10.0);
	private static final LoggedTunable<Velocity<LinearAccelerationUnit>> profileMaxJerk = LoggedTunable.from("Subsystems/Shooter/Flywheels/Mechanism/Profile/Max Jerk", MetersPerSecondPerSecond.per(Second)::of, 0.0);
	private static final LoggedTunable<FFConstants> ffGains = LoggedTunable.from(
		"Subsystems/Shooter/Flywheels/Mechanism/FF",
		new FFConstants(
			0.0,
			0.0,
			3.9,
			0.0
		)
	);
	private static final LoggedTunable<PIDConstants> pidGains = LoggedTunable.from(
		"Subsystems/Shooter/Flywheels/Mechanism/PID",
		new PIDConstants(
			5.0,
			0.0,
			0.0
		)
	);

	@Getter
	private double measuredSurfaceVeloMPS = 0.0;
	@Getter
	private double setpointSurfaceVeloMPS = 0.0;
	@Getter
	private double setpointSurfaceAccelMPSS = 0.0;

	public Flywheel(FlywheelConfig config, FlywheelIO io) {
		super("Shooter/Flywheels/" + config.name);

		this.config = config;

		System.out.println("[Init Flywheel] Instantiating Flywheel " + this.config.name + " with " + io.getClass().getSimpleName());
		this.io = io;

		final var sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(1.0).per(Second),
				Volts.of(7.0),
				Seconds.of(10.0),
				(state) -> {
					Logger.recordOutput("SysID/Shooter-Flywheel-" + config.name + "/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(voltage) -> {
					this.io.setVolts(voltage.in(Volts));
				},
				(log) -> {
					Logger.recordOutput("SysID/Shooter-Flywheel-" + config.name + "/Voltage", this.inputs.masterMotor.motor.getAppliedVolts());
					Logger.recordOutput("SysID/Shooter-Flywheel-" + config.name + "/Position Meters", FlywheelConstants.wheel.radiansToMeters(FlywheelConstants.motorToMechanism.applyUnsigned(this.inputs.masterMotor.encoder.getPositionRads())));
					Logger.recordOutput("SysID/Shooter-Flywheel-" + config.name + "/Velocity MetersPerSec", this.getMeasuredSurfaceVeloMPS());
				},
				this,
				"shooter-flywheel-" + config.name
			)
		);
		SmartDashboard.putData("SysID/Shooter/Flywheel/" + config.name + "/Quasi Forward", sysidRoutine.quasistatic(SysIdRoutine.Direction.kForward));
		SmartDashboard.putData("SysID/Shooter/Flywheel/" + config.name + "/Quasi Reverse", sysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
		SmartDashboard.putData("SysID/Shooter/Flywheel/" + config.name + "/Dynamic Forward", sysidRoutine.dynamic(SysIdRoutine.Direction.kForward));
		SmartDashboard.putData("SysID/Shooter/Flywheel/" + config.name + "/Dynamic Reverse", sysidRoutine.dynamic(SysIdRoutine.Direction.kReverse));

		if (!RobotConstants.tuningMode) {
			this.io.configProfile(
				FlywheelConstants.wheel.metersToRadians(Flywheel.profileMaxAcceleration.get().in(MetersPerSecondPerSecond)),
				FlywheelConstants.wheel.metersToRadians(Flywheel.profileMaxJerk.get().in(MetersPerSecondPerSecond.per(Second)))
			);
			this.io.configFF(Flywheel.ffGains.get().map((x) -> FlywheelConstants.wheel.radiansToMeters(x)));
			this.io.configPID(Flywheel.pidGains.get().map((x) -> FlywheelConstants.wheel.radiansToMeters(x)));
			this.io.configSend();
		}

		this.periodic();
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Shooter/Flywheels/" + this.config.name, this.inputs);

		this.measuredSurfaceVeloMPS = FlywheelConstants.wheel.radiansToMeters(FlywheelConstants.motorToMechanism.applyUnsigned(this.inputs.masterMotor.encoder.getVelocityRadsPerSec()));

		this.setpointSurfaceVeloMPS = FlywheelConstants.wheel.radiansToMeters(this.inputs.motorProfilePositionRads);
		this.setpointSurfaceAccelMPSS = FlywheelConstants.wheel.radiansToMeters(this.inputs.motorProfileVelocityRadsPerSec);

		Logger.recordOutput("Subsystems/Shooter/Flywheels/" + this.config.name + "/Surface Velocity/Measured", this.getMeasuredSurfaceVeloMPS(), MetersPerSecond);
		Logger.recordOutput("Subsystems/Shooter/Flywheels/" + this.config.name + "/Surface Velocity/Setpoint", this.getSetpointSurfaceVeloMPS(), MetersPerSecond);
		Logger.recordOutput("Subsystems/Shooter/Flywheels/" + this.config.name + "/Surface Acceleration/Setpoint", this.getSetpointSurfaceAccelMPSS(), MetersPerSecondPerSecond);

		var configChanged = false;
		if (
			Flywheel.profileMaxAcceleration.hasChanged(this.hashCode())
			| Flywheel.profileMaxJerk.hasChanged(this.hashCode())
		) {
			this.io.configProfile(
				FlywheelConstants.wheel.metersToRadians(Flywheel.profileMaxAcceleration.get().in(MetersPerSecondPerSecond)),
				FlywheelConstants.wheel.metersToRadians(Flywheel.profileMaxJerk.get().in(MetersPerSecondPerSecond.per(Second)))
			);
			configChanged = true;
		}
		if (Flywheel.ffGains.hasChanged(this.hashCode())) {
			this.io.configFF(Flywheel.ffGains.get().map((x) -> FlywheelConstants.wheel.radiansToMeters(x)));
			configChanged = true;
		}
		if (Flywheel.pidGains.hasChanged(this.hashCode())) {
			this.io.configPID(Flywheel.pidGains.get().map((x) -> FlywheelConstants.wheel.radiansToMeters(x)));
			configChanged = true;
		}
		if (configChanged) {
			this.io.configSend();
		}
	}

	public Command idle() {
		final var flywheel = this;
		return new Command() {
			{
				this.setName("Idle");
				this.addRequirements(flywheel);
			}

			@Override
			public void initialize() {
				flywheel.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command genSurfaceVeloCommand(String name, DoubleSupplier surfaceVeloSupplierMPS) {
		final var flywheel = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(flywheel);
			}

			@Override
			public void execute() {
				var goalSurfaceVeloMPS = surfaceVeloSupplierMPS.getAsDouble();
				var goalAngularVeloRadsPerSec = FlywheelConstants.wheel.metersToRadians(goalSurfaceVeloMPS);
				flywheel.io.setVelocityRadsPerSec(FlywheelConstants.motorToMechanism.inverse().applyUnsigned(goalAngularVeloRadsPerSec));
			}

			@Override
			public void end(boolean interrupted) {
				flywheel.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command spinup() {
		return this.genSurfaceVeloCommand(
			"Spinup",
			() -> Flywheel.spinupSurfaceVelo.get().in(MetersPerSecond)
		);
	}
}
