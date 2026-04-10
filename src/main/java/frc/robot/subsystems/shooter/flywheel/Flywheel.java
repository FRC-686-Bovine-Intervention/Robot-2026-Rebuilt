package frc.robot.subsystems.shooter.flywheel;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.RobotConstants;
import frc.util.FFGains;
import frc.util.NeutralMode;
import frc.util.PIDGains;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import lombok.Getter;

public class Flywheel extends SubsystemBase {
	private final FlywheelIO io;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

	private static final LoggedTunable<LinearVelocity> spinupSurfaceVelo = LoggedTunable.from("Subsystems/Shooter/Flywheel/Commands/Spinup/Velocity", MetersPerSecond::of, +20.0);

	private static final LoggedTunable<Current> bangBangCurrent = LoggedTunable.from("Subsystems/Shooter/Flywheel/Commands/Bang Bang/Current", Amps::of, 120.0);
	private static final LoggedTunableNumber bangBangThreshold = LoggedTunable.from("Subsystems/Shooter/Flywheel/Commands/Bang Bang/Threshold", 0.95);

	private static final LoggedTunable<LinearAcceleration> profileMaxAcceleration = LoggedTunable.from("Subsystems/Shooter/Flywheel/Mechanism/Profile/Max Acceleration", MetersPerSecondPerSecond::of, 7.0);
	private static final LoggedTunable<Velocity<LinearAccelerationUnit>> profileMaxJerk = LoggedTunable.from("Subsystems/Shooter/Flywheel/Mechanism/Profile/Max Jerk", MetersPerSecondPerSecond.per(Second)::of, 0.0);
	private static final LoggedTunable<FFGains> ffGains = LoggedTunable.from(
		"Subsystems/Shooter/Flywheel/Mechanism/FF",
		new FFGains(
			0.0,
			0.0,
			4.2,
			0.0
		)
	);
	private static final LoggedTunable<PIDGains> pidGains = LoggedTunable.from(
		"Subsystems/Shooter/Flywheel/Mechanism/PID",
		new PIDGains(
			15.0,
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

	public Flywheel(FlywheelIO io) {
		super("Shooter/Flywheel");

		System.out.println("[Init Flywheel] Instantiating Flywheel with " + io.getClass().getSimpleName());
		this.io = io;

		final var sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(1.0).per(Second),
				Volts.of(7.0),
				Seconds.of(10.0),
				(state) -> {
					Logger.recordOutput("SysID/Shooter-Flywheel/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(voltage) -> {
					this.io.setVolts(voltage.in(Volts));
				},
				(log) -> {
					Logger.recordOutput("SysID/Shooter-Flywheel/Voltage", this.inputs.leftBottomMotor.motor.getAppliedVolts());
					Logger.recordOutput("SysID/Shooter-Flywheel/Position Meters", FlywheelConstants.wheel.radiansToMeters(FlywheelConstants.motorToMechanism.applyUnsigned(this.inputs.leftBottomMotor.encoder.getPositionRads())));
					Logger.recordOutput("SysID/Shooter-Flywheel/Velocity MetersPerSec", this.getMeasuredSurfaceVeloMPS());
				},
				this,
				"shooter-flywheel-"
			)
		);
		SmartDashboard.putData("SysID/Shooter/Flywheel/Quasi Forward", sysidRoutine.quasistatic(SysIdRoutine.Direction.kForward));
		SmartDashboard.putData("SysID/Shooter/Flywheel/Quasi Reverse", sysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
		SmartDashboard.putData("SysID/Shooter/Flywheel/Dynamic Forward", sysidRoutine.dynamic(SysIdRoutine.Direction.kForward));
		SmartDashboard.putData("SysID/Shooter/Flywheel/Dynamic Reverse", sysidRoutine.dynamic(SysIdRoutine.Direction.kReverse));

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
		Logger.processInputs("Inputs/Shooter/Flywheel", this.inputs);

		this.measuredSurfaceVeloMPS = FlywheelConstants.wheel.radiansToMeters(FlywheelConstants.motorToMechanism.applyUnsigned(this.inputs.leftBottomMotor.encoder.getVelocityRadsPerSec()));

		this.setpointSurfaceVeloMPS = FlywheelConstants.wheel.radiansToMeters(this.inputs.profilePositionRads);
		this.setpointSurfaceAccelMPSS = FlywheelConstants.wheel.radiansToMeters(this.inputs.profileVelocityRadsPerSec);

		Logger.recordOutput("Subsystems/Shooter/Flywheel/Surface Velocity/Measured", this.getMeasuredSurfaceVeloMPS(), MetersPerSecond);
		Logger.recordOutput("Subsystems/Shooter/Flywheel/Surface Velocity/Setpoint", this.getSetpointSurfaceVeloMPS(), MetersPerSecond);
		Logger.recordOutput("Subsystems/Shooter/Flywheel/Surface Acceleration/Setpoint", this.getSetpointSurfaceAccelMPSS(), MetersPerSecondPerSecond);

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
				if (flywheel.getMeasuredSurfaceVeloMPS() > goalSurfaceVeloMPS * Flywheel.bangBangThreshold.getAsDouble()) {
					flywheel.io.setVelocityRadsPerSec(goalAngularVeloRadsPerSec);
					Logger.recordOutput("Subsystems/Shooter/Flywheel/Bang Bang Enabled", false);
				} else {
					flywheel.io.setCurrent(bangBangCurrent.get().in(Amps));
					Logger.recordOutput("Subsystems/Shooter/Flywheel/Bang Bang Enabled", true);
				}
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
