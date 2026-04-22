package frc.robot.automations;


import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HubShifts;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.EdgeDetector;
import frc.util.Environment;
import frc.util.loggerUtil.tunables.LoggedTunable;

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
		final var isShooting = this.shooter.flywheel.isShooting() && this.shooter.hood.isShooting();
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

		final var enablePassing = this.enablePassingTrigger.getAsBoolean();

		final boolean shooterWithinTolerance;
		if (enablePassing) {
			shooterWithinTolerance = this.shooter.withinPassingTolerance();
		} else {
			shooterWithinTolerance = this.shooter.withinShootingTolerance();
		}

		final var shooterDebounced = this.shooterDebouncer.calculate(shooterWithinTolerance);

		final var disableButton = this.disableTrigger.getAsBoolean();

		var hubTagSeen = false;
		for (final var tagID : FieldConstants.hubTagIDs.getOurs()) {
			if (!RobotState.getInstance().isTagStale(tagID)) {
				hubTagSeen = true;
				break;
			}
		}

		if (enablePassing) {
			this.edgeDetector.update(
				isShooting
				&& isEnabled
				&& shooterDebounced
				&& !disableButton
			);
		} else {
			this.edgeDetector.update(
				isShooting
				&& isEnabled
				&& isHubShift
				&& shooterDebounced
				&& !disableButton
				&& hubTagSeen
			);
		}


		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}

		Leds.getInstance().shooterReadyAnimation.setFlag(this.edgeDetector.getValue() && !enablePassing && isShooting);
		Leds.getInstance().shooterWaitingForShiftAnimation.setFlag(!isHubShift && !enablePassing && isShooting);
		Leds.getInstance().shooterWaitingForTagsAnimation.setFlag(!hubTagSeen && !enablePassing && isShooting);
		Leds.getInstance().shooterOutOfToleranceAnimation.setFlag(!shooterWithinTolerance && !enablePassing && isShooting);
		Leds.getInstance().shooterDisabledAnimation.setFlag(this.disableTrigger.getAsBoolean() && !enablePassing && isShooting);

		Logger.recordOutput("Automations/Auto Feed/Is Shooting", isShooting);
		Logger.recordOutput("Automations/Auto Feed/Within Tolerance", shooterWithinTolerance);
		Logger.recordOutput("Automations/Auto Feed/Passing Enabled", enablePassing);
		Logger.recordOutput("Automations/Auto Feed/Feeding", this.edgeDetector.getValue());
		Logger.recordOutput("Automations/Auto Feed/Shooting Conditions/Enabled", isEnabled);
		Logger.recordOutput("Automations/Auto Feed/Shooting Conditions/Hub Active", isHubShift);
		Logger.recordOutput("Automations/Auto Feed/Shooting Conditions/Within Tolerance Debounced", shooterDebounced);
		Logger.recordOutput("Automations/Auto Feed/Shooting Conditions/Disable Button", disableButton);
		Logger.recordOutput("Automations/Auto Feed/Shooting Conditions/Hub Tag Seen", hubTagSeen);
		Logger.recordOutput("Automations/Auto Feed/Passing Conditions/Enabled", isEnabled);
		Logger.recordOutput("Automations/Auto Feed/Passing Conditions/Within Tolerance Debounced", shooterDebounced);
		Logger.recordOutput("Automations/Auto Feed/Passing Conditions/Disable Button", disableButton);
	}
}
