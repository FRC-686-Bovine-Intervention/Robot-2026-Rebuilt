package frc.robot.subsystems.shooter.flywheel;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

	private static final LoggedTunable<LinearVelocity> spinupSurfaceVelo = LoggedTunable.from("Shooter/Flyhweels/Spinup Velocity", MetersPerSecond::of, +20.0);

	private static final LoggedTunable<PIDConstants> pidGains = LoggedTunable.from("Shooter/Flywheels/PID", new PIDConstants(
		0,
		0,
		0
	));
	private static final LoggedTunable<FFConstants> ffGains = LoggedTunable.from("Shooter/Flywheels/FF", new FFConstants(
		0,
		0,
		0,
		0
	));
	private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.from("Shooter/Flywheels/Profile",
		new TrapezoidProfile.Constraints(
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

		this.io.configProfile(
			profileConsts.get().maxVelocity,
			profileConsts.get().maxAcceleration
		);
		this.io.configFF(ffGains.get());
		this.io.configPID(pidGains.get());
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Shooter/Flywheels/" + this.config.name, this.inputs);

		this.measuredSurfaceVeloMPS = FlywheelConstants.driverFlywheelWheel.radiansToMeters(FlywheelConstants.driverMotorToFlywheelRatio.applyUnsigned(this.inputs.masterMotor.encoder.getVelocityRadsPerSec()));

		this.setpointSurfaceVeloMPS = FlywheelConstants.driverFlywheelWheel.radiansToMeters(FlywheelConstants.driverMotorToFlywheelRatio.applyUnsigned(this.inputs.motorProfilePositionRads));
		this.setpointSurfaceAccelMPSS = FlywheelConstants.driverFlywheelWheel.radiansToMeters(FlywheelConstants.driverMotorToFlywheelRatio.applyUnsigned(this.inputs.motorProfileVelocityRadsPerSec));

		Logger.recordOutput("Shooter/Flywheels/" + this.config.name + "/Surface Velocity/Measured", this.getMeasuredSurfaceVeloMPS(), MetersPerSecond);
		Logger.recordOutput("Shooter/Flywheels/" + this.config.name + "/Surface Velocity/Setpoint", this.getSetpointSurfaceVeloMPS(), MetersPerSecond);
		Logger.recordOutput("Shooter/Flywheels/" + this.config.name + "/Surface Acceleration/Setpoint", this.getSetpointSurfaceAccelMPSS(), MetersPerSecondPerSecond);

		if (profileConsts.hasChanged(this.hashCode())) {
			this.io.configProfile(
				profileConsts.get().maxVelocity,
				profileConsts.get().maxAcceleration
			);
		}

		if (ffGains.hasChanged(this.hashCode())) {
			this.io.configFF(ffGains.get());
		}

		if (pidGains.hasChanged(this.hashCode())) {
			this.io.configPID(pidGains.get());
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
				var goalAngularVeloRadsPerSec = FlywheelConstants.driverFlywheelWheel.metersToRadians(goalSurfaceVeloMPS);
				flywheel.io.setVelocityRadsPerSec(
					FlywheelConstants.driverMotorToFlywheelRatio.inverse().applyUnsigned(goalAngularVeloRadsPerSec),
					0.0,
					0.0
				);
			}

			@Override
			public void end(boolean interrupted) {
				flywheel.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command spinup() {
		return this.genSurfaceVeloCommand("Spinup", () -> spinupSurfaceVelo.get().in(MetersPerSecond));
	}
}
