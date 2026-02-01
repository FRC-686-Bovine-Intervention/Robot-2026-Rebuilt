package frc.robot.subsystems.shooter.flywheels;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public class Flywheels extends SubsystemBase {
	private final FlywheelsIO io;
	private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

	private static final LoggedTunable<PIDConstants> driverPIDConsts = LoggedTunable.from("Shooter/Flywheels/Driver/PID", new PIDConstants(
		0.5,
		0,
		0
	));
	private static final LoggedTunable<FFConstants> driverFFConsts = LoggedTunable.from("Shooter/Flywheels/Driver/FF", new FFConstants(
		0,
		0,
		0.175,
		0
	));
	private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.from("Shooter/Flywheels/Profile", new TrapezoidProfile.Constraints(
		0,
		0
	));
	private static final LoggedTunableNumber motorPulleyTeethCount = LoggedTunable.from("Shooter/Flywheels/Gear Ratio/Motor Pulley Teeth Count", 33);
	private static final LoggedTunableNumber flywheelPullyTeethCount = LoggedTunable.from("Shooter/Flywheels/Gear Ratio/Flywheel Pulley Teeth Count", 16);
	private static final LoggedTunable<Distance> flywheelRadius = LoggedTunable.from("Shooter/Flywheels/Gear Ratio/Flywheel Radius Inches", Inches::of, 2.0);

	private GearRatio gearRatio = new GearRatio().sprocket(motorPulleyTeethCount.getAsDouble()).sprocket(flywheelPullyTeethCount.getAsDouble());
	private LinearRelation flywheel = LinearRelation.wheelRadius(flywheelRadius.get());

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

		this.driverSurfaceVeloMPS = this.flywheel.radiansToMeters(this.gearRatio.applyUnsigned(this.inputs.masterMotor.encoder.getVelocityRadsPerSec()));

		Logger.recordOutput("Shooter/Flywheels/Measured Velo MPS", this.driverSurfaceVeloMPS, MetersPerSecond);

		if (driverPIDConsts.hasChanged(this.hashCode())) {
			this.io.configPID(driverPIDConsts.get());
		}
		if (driverFFConsts.hasChanged(this.hashCode())) {
			driverFFConsts.get().update(this.driverFF);
		}
		if (profileConsts.hasChanged(this.hashCode())) {
			this.driverMotionProfile = new TrapezoidProfile(profileConsts.get());
		}
		if (LoggedTunable.hasChanged(this.hashCode(), motorPulleyTeethCount, flywheelPullyTeethCount)) {
			this.gearRatio = new GearRatio().sprocket(motorPulleyTeethCount.getAsDouble()).sprocket(flywheelPullyTeethCount.getAsDouble());
		}
		if (flywheelRadius.hasChanged(this.hashCode())) {
			this.flywheel = LinearRelation.wheelRadius(flywheelRadius.get());
		}
	}

	public Command stop(Optional<NeutralMode> neutralMode) {
		final var flywheels = this;
		return new Command() {
			{
				this.setName("Stop");
				this.addRequirements(flywheels);
			}

			@Override
			public void initialize() {
				flywheels.io.stop(neutralMode);
			}

			@Override
			public void end(boolean interrupted) {
				flywheels.io.stop(NeutralMode.DEFAULT);
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
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
				flywheels.io.setVelocityRadsPerSec(
					flywheels.gearRatio.inverse().applyUnsigned(flywheels.flywheel.metersToRadians(newDriverSetpoint.position)),
					flywheels.gearRatio.inverse().applyUnsigned(flywheels.flywheel.metersToRadians(newDriverSetpoint.velocity)),
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
