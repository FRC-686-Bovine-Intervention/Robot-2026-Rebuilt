package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HubShifts;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.EdgeDetector;

public class AutomaticScoring implements Runnable {
	private final Drive drive;
	private final Shooter shooter;
	private final Rollers rollers;
	private final Command command;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public AutomaticScoring(Drive drive, Shooter shooter, Rollers rollers) {
		this.drive = drive;
		this.shooter = shooter;
		this.rollers = rollers;
		this.command = Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					RobotState.getInstance()::getEstimatedGlobalPose,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimLeftFlywheelAtHub(),
				this.shooter.aimRightFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHub(this.drive.rotationalSubsystem),
				this.rollers.indexer.index(),
				this.rollers.agitator.index(),
				this.rollers.feeder.feed()
			)
			.withName("Auto Score");
	}

	@Override
	public void run() {
		var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
		var currentHubShift = HubShifts.getCurrentShift();
		this.shooter.aimingSystem.shootingCalc.calculate(robotPose.getTranslation(), this.drive.getFieldMeasuredSpeeds(), FieldConstants.hubAimPoint.getOurs());
		boolean isHubShift = (currentHubShift.isHubActive().getOurs() && currentHubShift.getSecsLeftInShift() > this.shooter.aimingSystem.shootingCalc.getTOFSeconds()) || (currentHubShift.next().isHubActive().getOurs() && currentHubShift.getSecsSinceShiftStarted() <= this.shooter.aimingSystem.shootingCalc.getTOFSeconds());

		this.edgeDetector.update(
			FieldConstants.allianceZone.getOurs().withinBounds(robotPose.getTranslation())
			&& this.drive.rotationalSubsystem.getCurrentCommand() == null
			&& isHubShift
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
