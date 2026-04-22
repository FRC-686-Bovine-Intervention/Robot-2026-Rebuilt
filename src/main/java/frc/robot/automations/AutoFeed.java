package frc.robot.automations;


import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HubShifts;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
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
		final var isShooting = this.shooter.flywheel.isShooting() && this.shooter.hood.isShooting();
		final var isEnabled = this.shooter.aimingSystem.isAutoFeedEnabled();
		final var ignoringTags = this.shooter.aimingSystem.isAutoFeedIgnoreTags();
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
			shooterWithinTolerance = this.shooter.withinShootingTolerance(ignoringTags);
		}

		final var shooterDebounced = this.shooterDebouncer.calculate(shooterWithinTolerance);

		final var disableButton = this.disableTrigger.getAsBoolean();

		var hubTagSeen = false;
		if (!ignoringTags) {
			for (final var tagID : FieldConstants.hubTagIDs.getOurs()) {
				if (!RobotState.getInstance().isTagStale(tagID)) {
					hubTagSeen = true;
					break;
				}
			}
		}

		if (enablePassing) {
			this.edgeDetector.update(
				isShooting
				&& isEnabled
				&& shooterDebounced
				&& !disableButton
				&& DriverStation.isTeleopEnabled()
			);
		} else {
			this.edgeDetector.update(
				isShooting
				&& isEnabled
				&& isHubShift
				&& shooterDebounced
				&& !disableButton
				&& (hubTagSeen || ignoringTags)
				&& DriverStation.isTeleopEnabled()
			);
		}


		if (this.edgeDetector.risingEdge() && !this.autoFeedCommand.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.autoFeedCommand);
			CommandScheduler.getInstance().schedule(this.autoAgitateCommand);
			CommandScheduler.getInstance().schedule(this.autoCompressCommand);
		} else if (this.edgeDetector.fallingEdge() && this.autoFeedCommand.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.autoFeedCommand);
			CommandScheduler.getInstance().schedule(this.intakeDeployCommand);
			CommandScheduler.getInstance().cancel(this.autoCompressCommand);
		}

		Leds.getInstance().shooterReadyAnimation.setFlag(this.edgeDetector.getValue() && !enablePassing && isShooting && DriverStation.isTeleopEnabled());
		Leds.getInstance().shooterWaitingForShiftAnimation.setFlag(!isHubShift && !enablePassing && isShooting && DriverStation.isTeleopEnabled());
		Leds.getInstance().shooterWaitingForTagsAnimation.setFlag(!(hubTagSeen || ignoringTags) && !enablePassing && isShooting && DriverStation.isTeleopEnabled());
		Leds.getInstance().shooterOutOfToleranceAnimation.setFlag(!shooterWithinTolerance && !enablePassing && isShooting && DriverStation.isTeleopEnabled());
		Leds.getInstance().shooterDisabledAnimation.setFlag(this.disableTrigger.getAsBoolean() && !enablePassing && isShooting && DriverStation.isTeleopEnabled());

		Logger.recordOutput("Automations/Auto Feed/Is Shooting", isShooting);
		Logger.recordOutput("Automations/Auto Feed/Ignoring Tags", ignoringTags);
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
