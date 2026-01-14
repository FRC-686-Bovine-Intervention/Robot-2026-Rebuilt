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
	private static final LoggedTunable<PIDConstants> kickerPIDConsts = LoggedTunable.from("Shooter/Flywheels/Kicker/PID", new PIDConstants(
		0,
		0,
		0
	));
	private static final LoggedTunable<FFConstants> kickerFFConsts = LoggedTunable.from("Shooter/Flywheels/Kicker/FF", new FFConstants(
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
	private TrapezoidProfile kickerMotionProfile = new TrapezoidProfile(profileConsts.get());
	private TrapezoidProfile.State driverSetpointState = new TrapezoidProfile.State();
	private TrapezoidProfile.State kickerSetpointState = new TrapezoidProfile.State();
	private SimpleMotorFeedforward driverFF = new SimpleMotorFeedforward(driverFFConsts.get().kS(), driverFFConsts.get().kV(), driverFFConsts.get().kA());
	private SimpleMotorFeedforward kickerFF = new SimpleMotorFeedforward(kickerFFConsts.get().kS(), kickerFFConsts.get().kV(), kickerFFConsts.get().kA());

	private double driverSurfaceVeloMPS = 0.0;
	private double kickerSurfaceVeloMPS = 0.0;

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
		if (kickerPIDConsts.hasChanged(this.hashCode())) {
			this.io.configKickerPID(kickerPIDConsts.get());
		}
		if (driverFFConsts.hasChanged(this.hashCode())) {
			driverFFConsts.get().update(this.driverFF);
		}
		if (kickerFFConsts.hasChanged(this.hashCode())) {
			kickerFFConsts.get().update(this.kickerFF);
		}
		if (profileConsts.hasChanged(this.hashCode())) {
			this.driverMotionProfile = new TrapezoidProfile(profileConsts.get());
			this.kickerMotionProfile = new TrapezoidProfile(profileConsts.get());
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
				flywheels.kickerSetpointState.position = flywheels.kickerSurfaceVeloMPS;
				flywheels.kickerSetpointState.velocity = 0.0;
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
				var newKickerSetpoint = flywheels.kickerMotionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, flywheels.kickerSetpointState, goalState);
				var kickerFFOut = flywheels.kickerFF.calculateWithVelocities(flywheels.kickerSetpointState.position, newKickerSetpoint.position);
				flywheels.io.setKickerVelocityRadsPerSec(
					FlywheelConstants.kickerMotorToFlywheelRatio.inverse().applyUnsigned(FlywheelConstants.kickerWheel.metersToRadians(newKickerSetpoint.position)),
					FlywheelConstants.kickerMotorToFlywheelRatio.inverse().applyUnsigned(FlywheelConstants.kickerWheel.metersToRadians(newKickerSetpoint.velocity)),
					kickerFFOut
				);
			}

			@Override
			public void end(boolean interrupted) {
				flywheels.io.stopDriver(NeutralMode.DEFAULT);
				flywheels.io.stopKicker(NeutralMode.DEFAULT);
			}
		};
	}
}
