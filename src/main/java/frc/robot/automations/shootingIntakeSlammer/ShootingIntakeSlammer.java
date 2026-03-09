package frc.robot.automations.shootingIntakeSlammer;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.EdgeDetector;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class ShootingIntakeSlammer implements Runnable {
	private final HopperTrackerIO io;
	private final HopperTrackerIOInputsAutoLogged inputs = new HopperTrackerIOInputsAutoLogged();

	private final IntakeSlam slam;
	private final ExtensionSystem extensionSystem;
	private final Shooter shooter;

	private final Command command;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	private static final LoggedTunableNumber slamBallThreshold = LoggedTunable.from("ShootingIntakeSlammer/Slam Ball Threshold", 5.0);
	private static final LoggedTunable<Time> slamPeriod = LoggedTunable.from("ShootingIntakeSlammer/Slam Period", Seconds::of, 1);

	public ShootingIntakeSlammer(HopperTrackerIO io, IntakeSlam slam, ExtensionSystem extensionSystem, Shooter shooter) {
		this.io = io;
		this.slam = slam;
		this.extensionSystem = extensionSystem;
		this.shooter = shooter;

		this.command = this.slam.deploy(extensionSystem).withDeadline(Commands.waitTime(slamPeriod.get())).andThen(this.slam.stow().withDeadline(Commands.waitTime(slamPeriod.get()))).repeatedly();
	}

	@Override
	public void run() {
		if (!this.command.isScheduled()) {
			this.io.setIntakeDeployed(this.slam.isDeployed());
		}
		this.io.updateInputs(this.inputs);

		Logger.recordOutput("DEBUG/ShootingIntakeSlammer", inputs.ballCount);

		this.edgeDetector.update(
			this.inputs.ballCount <= slamBallThreshold.getAsDouble()
			&& this.shooter.aimingSystem.getCurrentCommand() != null
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
