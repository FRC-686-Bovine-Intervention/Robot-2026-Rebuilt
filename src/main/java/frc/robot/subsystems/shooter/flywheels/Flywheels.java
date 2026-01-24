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

	private static final LoggedTunable<PIDConstants> driverPIDConsts = LoggedTunable.from("Shooter/Flywheels/Driver/PID", new PIDConstants(
		0,
		0,
		0
	));
	private static final LoggedTunable<FFConstants> driverFFConsts = LoggedTunable.from("Shooter/Flywheels/Driver/FF", new FFConstants(
		0,
		0,
		0,
		0
	));
	private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.from("Shooter/Flywheels/Profile", new TrapezoidProfile.Constraints(
		0,
		0
	));

	private TrapezoidProfile driverMotionProfile = new TrapezoidProfile(profileConsts.get());
	private TrapezoidProfile.State driverSetpointState = new TrapezoidProfile.State();
	private SimpleMotorFeedforward driverFF = new SimpleMotorFeedforward(driverFFConsts.get().kS(), driverFFConsts.get().kV(), driverFFConsts.get().kA());

	private double driverSurfaceVeloMPS = 0.0;

	public Flywheels(FlywheelsIO io) {
		super("Shooter/Flywheels");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Shooter/Flywheels", this.inputs);

		if (driverPIDConsts.hasChanged(this.hashCode())) {
			this.io.configDriverPID(driverPIDConsts.get());
		}
		if (driverFFConsts.hasChanged(this.hashCode())) {
			driverFFConsts.get().update(this.driverFF);
		}
		if (profileConsts.hasChanged(this.hashCode())) {
			this.driverMotionProfile = new TrapezoidProfile(profileConsts.get());
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
				flywheels.driverSetpointState.position = flywheels.driverSurfaceVeloMPS;
				flywheels.driverSetpointState.velocity = 0.0;
			}

			@Override
			public void execute() {
				var goalSurfaceVeloMPS = surfaceVeloSupplierMPS.getAsDouble();
				var goalState = new TrapezoidProfile.State(goalSurfaceVeloMPS, 0.0);
				var newDriverSetpoint = flywheels.driverMotionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, flywheels.driverSetpointState, goalState);
				var driverFFOut = flywheels.driverFF.calculateWithVelocities(flywheels.driverSetpointState.position, newDriverSetpoint.position);
				flywheels.io.setDriverVelocityRadsPerSec(
					FlywheelConstants.driverMotorToFlywheelRatio.inverse().applyUnsigned(FlywheelConstants.driverFlywheelWheel.metersToRadians(newDriverSetpoint.position)),
					FlywheelConstants.driverMotorToFlywheelRatio.inverse().applyUnsigned(FlywheelConstants.driverFlywheelWheel.metersToRadians(newDriverSetpoint.velocity)),
					driverFFOut
				);
			}

			@Override
			public void end(boolean interrupted) {
				flywheels.io.stopDriver(NeutralMode.DEFAULT);
			}
		};
	}
}
