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
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.tuning.LoggedTunable;
import frc.util.EdgeDetector;
import frc.util.Environment;

public class AutoFeed implements Runnable {
	private final Shooter shooter;
	private final Rollers rollers;

	private final Command command;

	private final BooleanSupplier disableTrigger;
	private final BooleanSupplier enablePassingTrigger;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	private final Debouncer shooterDebouncer = new Debouncer(0.25, DebounceType.kRising);

	private static final LoggedTunable<Time> inactiveFeedTime = LoggedTunable.from("Automations/Auto Feed/Inactive Feed Time", Seconds::of, 2.0);

	public AutoFeed(Shooter shooter, Rollers rollers, BooleanSupplier disableTrigger, BooleanSupplier enablePassingTrigger) {
		this.shooter = shooter;
		this.rollers = rollers;

		this.disableTrigger = disableTrigger;
		this.enablePassingTrigger = enablePassingTrigger;

		this.command = Commands.parallel(
			this.rollers.feeder.feed(),
			this.rollers.indexer.index()
		)
		.withName("Auto Feed");
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
					&& currentHubShift.next().getSecsSinceShiftStarted() < AutoFeed.inactiveFeedTime.get().in(Seconds) - this.shooter.aimingSystem.shootingCalc.getTOFSeconds()
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

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
