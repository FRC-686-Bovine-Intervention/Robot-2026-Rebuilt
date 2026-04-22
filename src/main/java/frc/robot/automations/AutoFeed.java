package frc.robot.automations;


import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HubShifts;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.EdgeDetector;
import frc.util.Environment;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class AutoFeed implements Runnable {
	private final Shooter shooter;

	private final Command intakeDeployCommand;

	private final Command autoFeedCommand;
	private final Command autoAgitateCommand;
	private final Command autoCompressCommand;

	private final BooleanSupplier disableTrigger;
	private final BooleanSupplier enablePassingTrigger;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	private final Debouncer shooterDebouncer = new Debouncer(0.25, DebounceType.kRising);

	private static final LoggedTunable<Time> inactiveFeedTime = LoggedTunable.from("Automations/Auto Feed/Inactive Feed Time", Seconds::of, 2.0);

	public AutoFeed(Shooter shooter, Rollers rollers, Intake intake, ExtensionSystem extensionSystem, Command intakeDeployCommand, BooleanSupplier disableTrigger, BooleanSupplier enablePassingTrigger) {
		this.shooter = shooter;

		this.disableTrigger = disableTrigger;
		this.enablePassingTrigger = enablePassingTrigger;

		this.autoFeedCommand = Commands.parallel(
			rollers.feeder.feed(),
			rollers.indexer.index()
		)
		.withName("Auto Feed");

		this.autoAgitateCommand = intake.slam.hopperAgitate(extensionSystem).withName("Auto Agitate");
		this.autoCompressCommand = intake.rollers.compress().withName("Auto Compress");

		this.intakeDeployCommand = intakeDeployCommand;
	}

	@Override
	public void run() {
		final var isEnabled = this.shooter.aimingSystem.isAutoFeedEnabled();
		final var currentHubShift = HubShifts.getCurrentShift();
		final var isHubShift =
			!Environment.isCompetition()
			|| (
				(
					currentHubShift.isHubActive().getOurs()
					&& currentHubShift.getSecsLeftInShift() > this.shooter.aimingSystem.shootingCalc.getTOFSeconds() - AutoFeed.inactiveFeedTime.get().in(Seconds)
				)
				|| (
					!currentHubShift.isHubActive().getOurs()
					&& currentHubShift.getSecsSinceShiftStarted() < AutoFeed.inactiveFeedTime.get().in(Seconds) - this.shooter.aimingSystem.shootingCalc.getTOFSeconds()
				)
				|| (
					currentHubShift.next().isHubActive().getOurs()
					&& currentHubShift.next().getSecsSinceShiftStarted() >= -this.shooter.aimingSystem.shootingCalc.getTOFSeconds()
				)
			)
		;

		final boolean shooterWithinTolerance;
		if (this.enablePassingTrigger.getAsBoolean()) {
			shooterWithinTolerance = this.shooter.withinPassingTolerance();
		} else {
			shooterWithinTolerance = this.shooter.withinShootingTolerance();
		}

		final var shooterDebounced = this.shooterDebouncer.calculate(shooterWithinTolerance);

		this.edgeDetector.update(
			isEnabled
			&& isHubShift
			&& shooterDebounced
			&& !this.disableTrigger.getAsBoolean()
		);

		if (this.edgeDetector.risingEdge() && !this.autoFeedCommand.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.autoFeedCommand);
			CommandScheduler.getInstance().schedule(this.autoAgitateCommand);
			CommandScheduler.getInstance().schedule(this.autoCompressCommand);
		} else if (this.edgeDetector.fallingEdge() && this.autoFeedCommand.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.autoFeedCommand);
			CommandScheduler.getInstance().schedule(this.intakeDeployCommand);
			CommandScheduler.getInstance().cancel(this.autoCompressCommand);
		}
	}
}
