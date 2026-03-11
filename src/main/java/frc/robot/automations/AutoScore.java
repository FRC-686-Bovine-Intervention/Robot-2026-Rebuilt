package frc.robot.automations;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.HubShifts;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.EdgeDetector;

public class AutoScore implements Runnable {
	private final Drive drive;
	private final Shooter shooter;
	private final Rollers rollers;
	private final Command command;

	private final Trigger disableTrigger;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public AutoScore(Drive drive, Shooter shooter, Rollers rollers, Trigger disableTrigger) {
		this.drive = drive;
		this.shooter = shooter;
		this.rollers = rollers;
		this.disableTrigger = disableTrigger;

		this.command = Commands.parallel(
			this.rollers.agitator.index(),
			this.rollers.feeder.feed(),
			this.rollers.indexer.index()
		)
		.withName("Auto Score");
	}

	@Override
	public void run() {
		var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
		var currentHubShift = HubShifts.getCurrentShift();
		this.shooter.aimingSystem.shootingCalc.calculate(robotPose, this.drive.getFieldMeasuredSpeeds(), FieldConstants.hubAimPoint.getOurs());
		boolean isHubShift = (currentHubShift.isHubActive().getOurs() && currentHubShift.getSecsLeftInShift() > this.shooter.aimingSystem.shootingCalc.getTOFSeconds()) || (currentHubShift.next().isHubActive().getOurs() && currentHubShift.next().getSecsSinceShiftStarted() >= -this.shooter.aimingSystem.shootingCalc.getTOFSeconds());

		this.edgeDetector.update(
			FieldConstants.allianceZone.getOurs().withinBounds(robotPose.getTranslation())
			&& this.shooter.withinTolerance()
			&& isHubShift
			&& !disableTrigger.getAsBoolean()
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
