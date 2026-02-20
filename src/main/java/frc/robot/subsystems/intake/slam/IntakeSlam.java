package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ExtensionSystem;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.FourBarLinkage;
import frc.util.robotStructure.angle.ArmMech;
import lombok.Getter;

public class IntakeSlam extends SubsystemBase {
	private final IntakeSlamIO io;
	private final IntakeSlamIOInputsAutoLogged inputs = new IntakeSlamIOInputsAutoLogged();

	private static final LoggedTunable<Angle> retractAngle = LoggedTunable.from("Intake/Slam/Retract/Angle", Degrees::of, IntakeSlamConstants.maxAngle.in(Degrees));
	private static final LoggedTunable<Angle> deployAngle = LoggedTunable.from("Intake/Slam/Deploy/Angle", Degrees::of, IntakeSlamConstants.minAngle.in(Degrees));
	private static final LoggedTunable<Angle> deployFlopAngle = LoggedTunable.from("Intake/Slam/Deploy/Flop Angle", Degrees::of, IntakeSlamConstants.maxAngle.minus(Degrees.of(10)).in(Degrees));

	private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
		"Intake/Slam/PID",
		new PIDConstants(
			0.0,
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
	private double angleRads = 0.0;
	@Getter
	private double velocityRadsPerSec = 0.0;

	public IntakeSlam(IntakeSlamIO io) {
		super("Intake/Slam");

		System.out.println("[Init IntakeSlam] Instantiating IntakeSlam with " + io.getClass().getSimpleName());
		this.io = io;

		this.io.configPID(pidConsts.get());
	}

	@Override
	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Before");
		this.io.updateInputs(this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Update Inputs");
		Logger.processInputs("Inputs/Intake/Slam", this.inputs);
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Process Inputs");

		this.angleRads = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads() + IntakeSlamConstants.encoderZeroOffset.in(Radians));
		this.velocityRadsPerSec = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

		Logger.recordOutput("Intake/Slam/Angle/Measured", this.getAngleRads(), Radians);
		Logger.recordOutput("Intake/Slam/Velocity/Measured", this.getVelocityRadsPerSec(), RadiansPerSecond);

		this.mechLinkage.setDriverAngleRads(this.getAngleRads());
		this.driverMech.setRads(this.mechLinkage.getHorizonBaseDriverCouplerAngleRads());
		this.followerMech.setRads(this.mechLinkage.getHorizonBaseFollowerCouplerAngleRads());
		this.couplerMech.setRads(this.mechLinkage.getDriverRelativeCouplerAngleRads());

		if (pidConsts.hasChanged(this.hashCode())) {
			this.io.configPID(pidConsts.get());
		}

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam");
	}

	private void setAngleGoalRads(double angleRads) {
		this.io.setPositionRads(
			// IntakeSlamConstants.motorToMechanism.inverse().applyUnsigned(angleRads - this.motorOffsetRads),
			angleRads - IntakeSlamConstants.encoderZeroOffset.in(Radians),
			0.0,
			0.0
		);
		Logger.recordOutput("Intake/Slam/Angle/Goal", angleRads - IntakeSlamConstants.encoderZeroOffset.in(Radians));
		Logger.recordOutput("Intake/Slam/Velocity/Goal", 0.0);
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
				if (slam.getAngleRads() > deployFlopAngle.get().in(Radians)) {
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
