package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.util.flipping.AllianceFlipped;
import lombok.Getter;

public class HubShifts {

	public static enum Shift {
		Auto,
		Transition,
		Shift1 {
			@Override
			public AllianceFlipped<Boolean> isHubActive() {
				var autoWinner = HubShifts.getAutoWinner();
				if (autoWinner.isEmpty()) {
					return BLUE_ACTIVE;
				}
				return switch (autoWinner.get()) {
					case Blue -> RED_ACTIVE;
					case Red -> BLUE_ACTIVE;
				};
			}
		},
		Shift2 {
			@Override
			public AllianceFlipped<Boolean> isHubActive() {
				var autoWinner = HubShifts.getAutoWinner();
				if (autoWinner.isEmpty()) {
					return RED_ACTIVE;
				}
				return switch (autoWinner.get()) {
					case Blue -> BLUE_ACTIVE;
					case Red -> RED_ACTIVE;
				};
			}
		},
		Shift3 {
			@Override
			public AllianceFlipped<Boolean> isHubActive() {
				var autoWinner = HubShifts.getAutoWinner();
				if (autoWinner.isEmpty()) {
					return BLUE_ACTIVE;
				}
				return switch (autoWinner.get()) {
					case Blue -> RED_ACTIVE;
					case Red -> BLUE_ACTIVE;
				};
			}
		},
		Shift4 {
			@Override
			public AllianceFlipped<Boolean> isHubActive() {
				var autoWinner = HubShifts.getAutoWinner();
				if (autoWinner.isEmpty()) {
					return RED_ACTIVE;
				}
				return switch (autoWinner.get()) {
					case Blue -> BLUE_ACTIVE;
					case Red -> RED_ACTIVE;
				};
			}
		},
		Endgame,
		Disabled {
			@Override
			public AllianceFlipped<Boolean> isHubActive() {
				return NONE_ACTIVE;
			}

			@Override
			public double getSecsLeftInShift() {
				return 0.0;
			}

			@Override
			public double getSecsSinceShiftStarted() {
				return 0.0;
			}
		},
		;

		public AllianceFlipped<Boolean> isHubActive() {
			return BOTH_ACTIVE;
		}

		public Shift next() {
			return switch (this) {
				case Auto -> Transition;
				case Transition -> Shift1;
				case Shift1 -> Shift2;
				case Shift2 -> Shift3;
				case Shift3 -> Shift4;
				case Shift4 -> Endgame;
				case Endgame -> Disabled;
				case Disabled -> Disabled;
			};
		}

		public double getSecsSinceShiftStarted() {
			final var shiftStartSecs = switch (this) {
				case Auto -> 0.0;
				case Transition -> 0.0;
				case Shift1 -> 10.0;
				case Shift2 -> 35.0;
				case Shift3 -> 60.0;
				case Shift4 -> 85.0;
				case Endgame -> 110.0;
				case Disabled -> 0.0;
			};
			final var enableTimestampSecs = switch (this) {
				case Auto -> HubShifts.autoEnableTimestampSecs;
				default -> HubShifts.teleopEnableTimestampSecs;
			};
			return (Timer.getTimestamp() - enableTimestampSecs) - shiftStartSecs;
		}
		public double getSecsLeftInShift() {
			final var shiftEndSecs = switch (this) {
				case Auto -> 20.0;
				case Transition -> 10.0;
				case Shift1 -> 35.0;
				case Shift2 -> 60.0;
				case Shift3 -> 85.0;
				case Shift4 -> 110.0;
				case Endgame -> 140.0;
				case Disabled -> 0.0;
			};
			final var enableTimestampSecs = switch (this) {
				case Auto -> HubShifts.autoEnableTimestampSecs;
				default -> HubShifts.teleopEnableTimestampSecs;
			};
			return shiftEndSecs - (Timer.getTimestamp() - enableTimestampSecs);
		}
		public double getShiftLength() {
			return switch (this) {
				case Auto -> 20.0;
				case Transition -> 10.0;
				case Shift1 -> 25.0;
				case Shift2 -> 25.0;
				case Shift3 -> 25.0;
				case Shift4 -> 25.0;
				case Endgame -> 30.0;
				case Disabled -> Double.POSITIVE_INFINITY;
			};
		}
	}

	@Getter
	private static Shift currentShift = Shift.Disabled;

	private static double autoEnableTimestampSecs;
	private static boolean prevAutoEnable = false;
	private static double teleopEnableTimestampSecs;
	private static boolean prevTeleopEnable = false;

	public static void periodic() {
		final var autoEnable = DriverStation.isAutonomousEnabled();
		final var teleopEnable = DriverStation.isTeleopEnabled();

		if (autoEnable && !HubShifts.prevAutoEnable) {
			HubShifts.autoEnableTimestampSecs = Timer.getTimestamp();
		}
		if (teleopEnable && !HubShifts.prevTeleopEnable) {
			HubShifts.teleopEnableTimestampSecs = Timer.getTimestamp();
		}

		HubShifts.prevAutoEnable = autoEnable;
		HubShifts.prevTeleopEnable = teleopEnable;

		if (autoEnable) {
			HubShifts.currentShift = Shift.Auto;
		} else if (teleopEnable) {
			final var timeSinceTeleEnable = Timer.getTimestamp() - HubShifts.teleopEnableTimestampSecs;
			if (timeSinceTeleEnable < 10.0) {
				HubShifts.currentShift = Shift.Transition;
			} else if (timeSinceTeleEnable < 35.0) {
				HubShifts.currentShift = Shift.Shift1;
			} else if (timeSinceTeleEnable < 60.0) {
				HubShifts.currentShift = Shift.Shift2;
			} else if (timeSinceTeleEnable < 85.0) {
				HubShifts.currentShift = Shift.Shift3;
			} else if (timeSinceTeleEnable < 110.0) {
				HubShifts.currentShift = Shift.Shift4;
			} else {
				HubShifts.currentShift = Shift.Endgame;
			}
		} else {
			HubShifts.currentShift = Shift.Disabled;
		}

		Logger.recordOutput("Hub Shifts/Current Shift", HubShifts.getCurrentShift());
		Logger.recordOutput("Hub Shifts/Hub Active", HubShifts.getCurrentShift().isHubActive().getOurs());
		Logger.recordOutput("Hub Shifts/Seconds left in Shift", HubShifts.getCurrentShift().getSecsLeftInShift(), Seconds);
		Logger.recordOutput("Hub Shifts/Seconds since Shift Started", HubShifts.getCurrentShift().getSecsSinceShiftStarted(), Seconds);
	}

	private static final Optional<Alliance> BLUE = Optional.of(Alliance.Blue);
	private static final Optional<Alliance> RED = Optional.of(Alliance.Red);
	private static final Optional<Alliance> NONE = Optional.empty();

	public static Optional<Alliance> getAutoWinner() {
		var message = DriverStation.getGameSpecificMessage();
		if (message.isEmpty()) {
			return NONE;
		}
		return switch (message.charAt(0)) {
			case 'B' -> BLUE;
			case 'R' -> RED;
			default -> NONE;
		};
	}

	private static final AllianceFlipped<Boolean> BOTH_ACTIVE = new AllianceFlipped<>(true, true);
	private static final AllianceFlipped<Boolean> BLUE_ACTIVE = new AllianceFlipped<>(true, false);
	private static final AllianceFlipped<Boolean> RED_ACTIVE = new AllianceFlipped<>(false, true);
	private static final AllianceFlipped<Boolean> NONE_ACTIVE = new AllianceFlipped<>(false, false);
}
