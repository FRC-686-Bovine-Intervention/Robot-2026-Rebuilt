package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HubShifts;
import frc.robot.HubShifts.Shift;
import frc.util.EdgeDetector;
import frc.util.controllers.XboxController;

public class HubShiftNotifications implements Runnable {
	private final Command command;

	public HubShiftNotifications(XboxController controller) {
		final var leftRumble = controller.leftRumble;
		final var rightRumble = controller.rightRumble;
		this.command = new Command() {
			{
				this.setName("Hub Shift Notification");
				this.addRequirements(leftRumble, rightRumble);
			}

			private Shift initShift;

			@Override
			public void initialize() {
				this.initShift = HubShifts.getCurrentShift();
			}

			@Override
			public void execute() {
				var wrappedTime = this.initShift.getSecsSinceShiftStarted() % 1.0;
				if (initShift.isHubActive().getOurs()) {
					if (initShift.next().isHubActive().getOurs()) {
						leftRumble.setRumble((wrappedTime < 0.3) ? (0.8) : (0.0));
					} else {
						if (this.initShift.getSecsLeftInShift() > 0.0) {
							rightRumble.setRumble((wrappedTime < 0.5) ? (0.8) : (0.0));
						} else {
							leftRumble.setRumble((wrappedTime < 0.3) ? (0.8) : (0.0));
						}
					}
				} else {
					if (initShift.next().isHubActive().getOurs()) {
						if (this.initShift.getSecsLeftInShift() > 0.0) {
							leftRumble.setRumble((wrappedTime < 0.3) ? (0.8) : (0.0));
						} else {
							rightRumble.setRumble((wrappedTime < 0.5) ? (0.8) : (0.0));
						}
					} else {
						rightRumble.setRumble((wrappedTime < 0.5) ? (0.8) : (0.0));
					}
				}
			}

			@Override
			public void end(boolean interrupted) {
				leftRumble.setRumble(0.0);
				rightRumble.setRumble(0.0);
			}

			@Override
			public boolean isFinished() {
				return this.initShift.getSecsLeftInShift() < -1.0;
			}
		};
	}

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	@Override
	public void run() {
		this.edgeDetector.update(HubShifts.getCurrentShift() != Shift.Auto && HubShifts.getCurrentShift().getSecsLeftInShift() < 3.0);

		if (this.edgeDetector.risingEdge()) {
			CommandScheduler.getInstance().schedule(this.command);
		}
	}
}
