package frc.util.mechanismUtil;

import edu.wpi.first.wpilibj.Alert;

public class Fault {
	private final Alert activeAlert;
	private final Alert stickyAlert;
	private boolean active = false;
	private boolean stickyActive = false;

	public Fault(Alert activeAlert, Alert stickyAlert) {
		this.activeAlert = activeAlert;
		this.stickyAlert = stickyAlert;
	}

	public void setActive(boolean active) {
		this.active = active;
		if (this.active) {
			this.stickyActive = true;
		}
		activeAlert.set(this.active);
		stickyAlert.set(this.stickyActive);
	}

	public void clearSticky() {
		stickyActive = active;
	}

	public boolean isActive() {
		return active;
	}
	public boolean isStickyActive() {
		return stickyActive;
	}
}
