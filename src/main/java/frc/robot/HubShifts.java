package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.util.EdgeDetector;
import frc.util.flipping.AllianceFlipped;
import frc.util.loggerUtil.tunables.LoggedTunable;
import lombok.Getter;

public class HubShifts {
	private static final Optional<Alliance> BLUE = Optional.of(Alliance.Blue);
	private static final Optional<Alliance> RED = Optional.of(Alliance.Red);
	private static final Optional<Alliance> NONE = Optional.empty();

	private static final AllianceFlipped<Boolean> BOTH_ACTIVE = new AllianceFlipped<>(true, true);
	private static final AllianceFlipped<Boolean> BLUE_ACTIVE = new AllianceFlipped<>(true, false);
	private static final AllianceFlipped<Boolean> RED_ACTIVE = new AllianceFlipped<>(false, true);
	private static final AllianceFlipped<Boolean> NONE_ACTIVE = new AllianceFlipped<>(false, false);

	public static enum Shift {
		Auto,
		Transition,
		Shift1 {
			@Override
			public AllianceFlipped<Boolean> isHubActive() {
				var autoWinner = HubShifts.getAutoWinner();
				if (autoWinner.isEmpty()) {
					return BOTH_ACTIVE;
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
					return BOTH_ACTIVE;
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
					return BOTH_ACTIVE;
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
					return BOTH_ACTIVE;
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
				return Double.POSITIVE_INFINITY;
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
			var shiftStartSecs = switch (this) {
				case Auto -> 0.0;
				case Transition -> 0.0;
				case Shift1 -> 10.0;
				case Shift2 -> 35.0;
				case Shift3 -> 60.0;
				case Shift4 -> 85.0;
				case Endgame -> 110.0;
				case Disabled -> 0.0;
			};
			return (Timer.getTimestamp() - teleopEnableTime) - shiftStartSecs;
		}

		public double getSecsLeftInShift() {
			var shiftEndSecs = switch (this) {
				case Auto -> 0.0;
				case Transition -> 10.0;
				case Shift1 -> 35.0;
				case Shift2 -> 60.0;
				case Shift3 -> 85.0;
				case Shift4 -> 110.0;
				case Endgame -> 140.0;
				case Disabled -> 0.0;
			};
			return shiftEndSecs - (Timer.getTimestamp() - teleopEnableTime);
		}
	}

	@Getter
	private static Shift currentShift = Shift.Disabled;

	private static double teleopEnableTime;
	private static final EdgeDetector teleopEnableEdgeDetector = new EdgeDetector(false);

	private static final LoggedTunable<Time> matchTimeErrorSyncThreshold = LoggedTunable.from("Hub Shifts/Match Time Sync Threshold", Seconds::of, 2.0);
	private static int prevMatchTime = 0;

	public static void periodic() {
		teleopEnableEdgeDetector.update(DriverStation.isTeleopEnabled());

		if (teleopEnableEdgeDetector.risingEdge()) {
			teleopEnableTime = Timer.getTimestamp();
		}

		if (DriverStation.isAutonomousEnabled()) {
			currentShift = Shift.Auto;
		} else if (teleopEnableEdgeDetector.getValue()) {
			DriverStation.getMatchTime();
			var timeSinceTeleEnable = Timer.getTimestamp() - teleopEnableTime;
			if (timeSinceTeleEnable < 10.0) {
				currentShift = Shift.Transition;
			} else if (timeSinceTeleEnable < 35.0) {
				currentShift = Shift.Shift1;
			} else if (timeSinceTeleEnable < 60.0) {
				currentShift = Shift.Shift2;
			} else if (timeSinceTeleEnable < 85.0) {
				currentShift = Shift.Shift3;
			} else if (timeSinceTeleEnable < 110.0) {
				currentShift = Shift.Shift4;
			} else {
				currentShift = Shift.Endgame;
			}
		} else {
			currentShift = Shift.Disabled;
		}
	}

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
}
