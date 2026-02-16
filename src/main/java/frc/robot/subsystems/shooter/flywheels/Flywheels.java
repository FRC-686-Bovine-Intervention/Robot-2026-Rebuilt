package frc.robot.subsystems.shooter.flywheels;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Flywheels extends SubsystemBase {
	private final FlywheelsIO io;
	private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

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
	private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.from("Shooter/Flywheels/Profile", new TrapezoidProfile.Constraints(
		0,
		0
	));

	private TrapezoidProfile motionProfile = new TrapezoidProfile(profileConsts.get());
	private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
	private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ffGains.get().kS(), ffGains.get().kV(), ffGains.get().kA());

	private double surfaceVeloMPS = 0.0;

	public Flywheels(FlywheelsIO io) {
		super("Shooter/Flywheels");

		System.out.println("[Init Flywheel] Instantiating Flywheel with " + io.getClass().getSimpleName());
		this.io = io;

		this.io.configPID(pidGains.get());
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Shooter/Flywheels", this.inputs);

		if (pidGains.hasChanged(this.hashCode())) {
			this.io.configPID(pidGains.get());
		}

		if (ffGains.hasChanged(this.hashCode())) {
			ffGains.get().update(this.ff);
		}
		if (profileConsts.hasChanged(this.hashCode())) {
			this.motionProfile = new TrapezoidProfile(profileConsts.get());
		}
	}

	public Command runAtSurfaceVelo(DoubleSupplier surfaceVeloSupplierMPS) {
		final var flywheels = this;
		return new Command() {
			{
				this.setName("Run At Velo");
				this.addRequirements(flywheels);
			}

			@Override
			public void initialize() {
				flywheels.setpointState.position = flywheels.surfaceVeloMPS;
				flywheels.setpointState.velocity = 0.0;
			}

			@Override
			public void execute() {
				var goalSurfaceVeloMPS = surfaceVeloSupplierMPS.getAsDouble();
				var goalState = new TrapezoidProfile.State(goalSurfaceVeloMPS, 0.0);
				var newDriverSetpoint = flywheels.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, flywheels.setpointState, goalState);
				var driverFFOut = flywheels.ff.calculateWithVelocities(flywheels.setpointState.position, newDriverSetpoint.position);
				flywheels.io.setVelocityRadsPerSec(
					FlywheelConstants.driverMotorToFlywheelRatio.inverse().applyUnsigned(FlywheelConstants.driverFlywheelWheel.metersToRadians(newDriverSetpoint.position)),
					FlywheelConstants.driverMotorToFlywheelRatio.inverse().applyUnsigned(FlywheelConstants.driverFlywheelWheel.metersToRadians(newDriverSetpoint.velocity)),
					driverFFOut
				);
			}

			@Override
			public void end(boolean interrupted) {
				flywheels.io.stop(NeutralMode.DEFAULT);
			}
		};
	}
}
