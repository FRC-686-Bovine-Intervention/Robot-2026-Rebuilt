package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ExtensionSystem;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.FourBarLinkage;
import frc.util.robotStructure.angle.ArmMech;
import lombok.Getter;

public class IntakeSlam extends SubsystemBase {
	private final IntakeSlamIO io;
	private final IntakeSlamIOInputsAutoLogged inputs = new IntakeSlamIOInputsAutoLogged();

	private static final LoggedTunable<Angle> retractAngle = LoggedTunable.from("Intake/Slam/Retract/Angle", Degrees::of, IntakeSlamConstants.maxAngle.in(Degrees));
	private static final LoggedTunable<Angle> deployAngle = LoggedTunable.from("Intake/Slam/Deploy/Angle", Degrees::of, IntakeSlamConstants.minAngle.in(Degrees));
	private static final LoggedTunable<Angle> deployFlopAngle = LoggedTunable.from("Intake/Slam/Deploy/Flop Angle", Degrees::of, IntakeSlamConstants.maxAngle.minus(Degrees.of(10.0)).in(Degrees));

	private static final LoggedTunableNumber profilekV = LoggedTunable.from("Intake/Slam/Profile/kV", 24.0);
	private static final LoggedTunableNumber profilekA = LoggedTunable.from("Intake/Slam/Profile/kA", 24.0);
	private static final LoggedTunable<AngularVelocity> profileMaxVel = LoggedTunable.from("Intake/Slam/Profile/Max Velocity", DegreesPerSecond::of, 0.0);

	private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
		"Intake/Slam/FF",
		new FFConstants(
			0.0,
			0.0,
			2.4,
			0.0
		)
	);

	private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
		"Intake/Slam/PID",
		new PIDConstants(
			1.5,
			0.0,
			0.0
		)
	);

	private final Alert motorDisconnectedAlert = new Alert("Intake/Slam/Alerts", "Motor Disconnected", AlertType.kError);
	private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Slam Motor Disconnected!", AlertType.kError);

	private final FourBarLinkage mechLinkage = new FourBarLinkage(
		IntakeSlamConstants.mechDriverAxle2d,
		IntakeSlamConstants.mechFollowerAxle2d,
		IntakeSlamConstants.mechDriverLength,
		IntakeSlamConstants.mechFollowerLength,
		IntakeSlamConstants.mechCouplerLength,
		false
	);
	public final ArmMech driverMech = new ArmMech(IntakeSlamConstants.mechDriverBase3d);
	public final ArmMech followerMech = new ArmMech(IntakeSlamConstants.mechFollowerBase3d);
	public final ArmMech couplerMech = new ArmMech(IntakeSlamConstants.mechCouplerBase3d);

	@Getter
	private double measuredAngleRads = 0.0;
	@Getter
	private double measuredVelocityRadsPerSec = 0.0;

	@Getter
	private double setpointAngleRads = 0.0;
	@Getter
	private double setpointVelocityRadsPerSec = 0.0;

	public IntakeSlam(IntakeSlamIO io) {
		super("Intake/Slam");

		System.out.println("[Init IntakeSlam] Instantiating IntakeSlam with " + io.getClass().getSimpleName());
		this.io = io;

		final var sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(1.0).per(Second),
				Volts.of(7.0),
				Seconds.of(10.0),
				(state) -> {
					Logger.recordOutput("SysID/Intake-Slam/State", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(voltage) -> {
					this.io.setVolts(voltage.in(Volts));
				},
				(log) -> {
					Logger.recordOutput("SysID/Intake-Slam/Voltage", this.inputs.motor.motor.getAppliedVolts());
					Logger.recordOutput("SysID/Intake-Slam/Position Rads", this.getMeasuredAngleRads());
					Logger.recordOutput("SysID/Intake-Slam/Velocity RadsPerSec", this.getMeasuredVelocityRadsPerSec());
				},
				this,
				"intake-slam"
			)
		);
		SmartDashboard.putData("SysID/Intake/Slam/Quasi Forward", sysidRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> this.getMeasuredAngleRads() >= IntakeSlamConstants.maxAngle.in(Radians)));
		SmartDashboard.putData("SysID/Intake/Slam/Quasi Reverse", sysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> this.getMeasuredAngleRads() <= IntakeSlamConstants.minAngle.in(Radians)));
		SmartDashboard.putData("SysID/Intake/Slam/Dynamic Forward", sysidRoutine.dynamic(SysIdRoutine.Direction.kForward).until(() -> this.getMeasuredAngleRads() >= IntakeSlamConstants.maxAngle.in(Radians)));
		SmartDashboard.putData("SysID/Intake/Slam/Dynamic Reverse", sysidRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> this.getMeasuredAngleRads() <= IntakeSlamConstants.minAngle.in(Radians)));

		this.periodic();
	}

	@Override
	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Update Inputs");
		Logger.processInputs("Inputs/Intake/Slam", this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Process Inputs");

		this.measuredAngleRads = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads() + IntakeSlamConstants.encoderZeroOffset.in(Radians));
		this.measuredVelocityRadsPerSec = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

		this.setpointAngleRads = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads() + IntakeSlamConstants.encoderZeroOffset.in(Radians));
		this.setpointVelocityRadsPerSec = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

		Logger.recordOutput("Intake/Slam/Angle/Measured", this.getMeasuredAngleRads(), Radians);
		Logger.recordOutput("Intake/Slam/Velocity/Measured", this.getMeasuredVelocityRadsPerSec(), RadiansPerSecond);
		Logger.recordOutput("Intake/Slam/Angle/Setpoint", this.getSetpointAngleRads(), Radians);
		Logger.recordOutput("Intake/Slam/Velocity/Setpoint", this.getSetpointVelocityRadsPerSec(), RadiansPerSecond);

		this.mechLinkage.setDriverAngleRads(this.getMeasuredAngleRads());
		this.driverMech.setRads(this.mechLinkage.getHorizonBaseDriverCouplerAngleRads());
		this.followerMech.setRads(this.mechLinkage.getHorizonBaseFollowerCouplerAngleRads());
		this.couplerMech.setRads(this.mechLinkage.getDriverRelativeCouplerAngleRads());

		var configChanged = false;
		if (
			IntakeSlam.profilekV.hasChanged(this.hashCode())
			| IntakeSlam.profilekA.hasChanged(this.hashCode())
			| IntakeSlam.profileMaxVel.hasChanged(this.hashCode())
		) {
			this.io.configProfile(
				IntakeSlam.profilekV.getAsDouble(),
				IntakeSlam.profilekA.getAsDouble(),
				IntakeSlam.profileMaxVel.get().in(RadiansPerSecond)
			);
			configChanged = true;
		}
		if (IntakeSlam.ffConsts.hasChanged(this.hashCode())) {
			this.io.configFF(IntakeSlam.ffConsts.get());
			configChanged = true;
		}
		if (IntakeSlam.pidConsts.hasChanged(this.hashCode())) {
			this.io.configPID(IntakeSlam.pidConsts.get());
			configChanged = true;
		}
		if (configChanged) {
			this.io.configSend();
		}

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam");
	}

	private void setAngleGoalRads(double angleRads) {
		this.io.setPositionRads(angleRads - IntakeSlamConstants.encoderZeroOffset.in(Radians));
		Logger.recordOutput("Intake/Slam/Angle/Goal", angleRads - IntakeSlamConstants.encoderZeroOffset.in(Radians));
	}

	public Command coast() {
		final var slam = this;
		return new Command() {
			{
				this.setName("Coast");
				this.addRequirements(slam);
			}

			@Override
			public void initialize() {
				slam.io.stop(NeutralMode.COAST);
			}

			@Override
			public void end(boolean interrupted) {
				slam.io.stop(NeutralMode.DEFAULT);
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}

	public Command retract() {
		final var slam = this;
		return new Command() {
			{
				this.setName("Retract");
				this.addRequirements(slam);
			}

			@Override
			public void execute() {
				slam.setAngleGoalRads(retractAngle.get().in(Radians));
			}

			@Override
			public void end(boolean interrupted) {
				slam.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command deploy(ExtensionSystem extension) {
		final var slam = this;
		return new Command() {
			{
				this.setName("Deploy");
				this.addRequirements(slam, extension);
			}

			@Override
			public void execute() {
				if (slam.getMeasuredAngleRads() > deployFlopAngle.get().in(Radians)) {
					slam.setAngleGoalRads(deployAngle.get().in(Radians));
				} else {
					slam.io.stop(NeutralMode.COAST);
				}
			}

			@Override
			public void end(boolean interrupted) {
				slam.io.stop(NeutralMode.COAST);
			}
		};
	}
}
