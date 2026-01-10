package frc.util.faults;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Alert;
import frc.util.faults.DeviceFaults.FaultType;

public class DeviceFaultAlerts {
	private final Alert alert;
	private final String prefix;
	private final long bitmask;

	public DeviceFaultAlerts(Alert alert, long bitmask) {
		this.alert = alert;
		this.prefix = this.alert.getText();
		this.bitmask = bitmask;
	}
	public DeviceFaultAlerts(Alert alert, FaultType... ignoreList) {
		this.alert = alert;
		this.prefix = this.alert.getText();
		var bitmask = DeviceFaults.allMask;
		for (var ignoreFault : ignoreList) {
			bitmask ^= ignoreFault.getBitmask();
		}
		this.bitmask = bitmask;
	}

	public void updateFrom(DeviceFaults faults) {
		this.alert.set(faults.anyOf(this.bitmask));
		if (this.alert.get()) {
			this.alert.setText(this.prefix + Arrays.toString(faults.getActiveFaults(this.bitmask)));
		}
	}
}
