package frc.robot.automations;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
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
import frc.util.loggerUtil.tunables.LoggedTunable;

public class AutomaticScoring implements Runnable {
	private final Drive drive;
	private final Shooter shooter;
	private final Rollers rollers;
	private final Command shootCommand;
	private final Command spinUpCommand;

	private final EdgeDetector shootingEdgeDetector = new EdgeDetector(false);
	private final EdgeDetector spinUpEdgeDetector = new EdgeDetector(false);


	private static final LoggedTunable<Time> spinUpSeconds = LoggedTunable.from("Automations/Auto Score/Spin Up Time", Seconds::of, 1);

	public AutomaticScoring(Drive drive, Shooter shooter, Rollers rollers) {
		this.drive = drive;
		this.shooter = shooter;
		this.rollers = rollers;
		this.spinUpCommand = Commands.parallel(
			this.shooter.aimingSystem.aimAtHub(
					RobotState.getInstance()::getEstimatedGlobalPose,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimLeftFlywheelAtHub(),
				this.shooter.aimRightFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHub(this.drive.rotationalSubsystem)
		).withName("Auto Spin Up");

		this.shootCommand = Commands.parallel(
				spinUpCommand,
				this.rollers.indexer.index(),
				this.rollers.agitator.index(),
				this.rollers.feeder.feed()
		).withName("Auto Score");
	}

	@Override
	public void run() {
		var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
		var currentHubShift = HubShifts.getCurrentShift();
		this.shooter.aimingSystem.shootingCalc.calculate(robotPose, this.drive.getFieldMeasuredSpeeds(), FieldConstants.hubAimPoint.getOurs());
		boolean willBeHubShift = (currentHubShift.next().isHubActive().getOurs() && currentHubShift.next().getSecsSinceShiftStarted() >= -spinUpSeconds.get().in(Seconds) - this.shooter.aimingSystem.shootingCalc.getTOFSeconds());
		boolean isHubShift = (currentHubShift.isHubActive().getOurs() && currentHubShift.getSecsLeftInShift() > this.shooter.aimingSystem.shootingCalc.getTOFSeconds()) || (currentHubShift.next().isHubActive().getOurs() && currentHubShift.next().getSecsSinceShiftStarted() >= -this.shooter.aimingSystem.shootingCalc.getTOFSeconds());

		this.spinUpEdgeDetector.update(
			FieldConstants.allianceZone.getOurs().withinBounds(robotPose.getTranslation())
			&& this.drive.rotationalSubsystem.getCurrentCommand() == null
			&& willBeHubShift
		);
		this.shootingEdgeDetector.update(
			FieldConstants.allianceZone.getOurs().withinBounds(robotPose.getTranslation())
			&& this.drive.rotationalSubsystem.getCurrentCommand() == null
			&& isHubShift
		);

		if (this.spinUpEdgeDetector.risingEdge() && !this.spinUpCommand.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.spinUpCommand);
		} else if (this.spinUpEdgeDetector.fallingEdge() && this.spinUpCommand.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.spinUpCommand);
		}

		if (this.shootingEdgeDetector.risingEdge() && !this.shootCommand.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.shootCommand);
		} else if (this.shootingEdgeDetector.fallingEdge() && this.shootCommand.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.shootCommand);
		}
	}
}
