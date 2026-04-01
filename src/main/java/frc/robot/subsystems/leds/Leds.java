package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.LoggedTracer;
import frc.util.VirtualSubsystem;
import frc.util.led.animation.AllianceColorAnimation;
import frc.util.led.animation.BarAnimation;
import frc.util.led.animation.FillAnimation;
import frc.util.led.animation.FlashingAnimation;
import frc.util.led.animation.StatusLightAnimation;
import frc.util.led.animation.WaveAnimation;
import frc.util.led.functions.InterpolationFunction;
import frc.util.led.functions.WaveFunction;
import frc.util.led.strips.hardware.AddressableStrip;
import frc.util.led.strips.hardware.HardwareStrip;

public class Leds extends VirtualSubsystem {
	private static Leds instance;
	public static Leds getInstance() {if(instance == null) {instance = new Leds();} return instance;}

	private final HardwareStrip hardwareStrip;

	private final Notifier loadingNotifier;

	public Leds() {
		System.out.println("[Init Leds] Instantiating Leds");

		this.hardwareStrip = new AddressableStrip(HardwareDevices.ledPort, 61);

		final var rightTowerStrip = this.hardwareStrip.substrip(0, 16);
		final var leftTowerStrip = this.hardwareStrip.substrip(16, 32).reverse();
		final var intakeStrip = this.hardwareStrip.substrip(32, 61);

		final var sideStrips = leftTowerStrip.parallel(rightTowerStrip);

		this.bootingAnimation = new FlashingAnimation(this.hardwareStrip, (t) -> WaveFunction.Sinusoidal.applyAsDouble(System.currentTimeMillis() / 1000.0), InterpolationFunction.linear.gradient(Color.kBlack, Color.kDimGray));
		this.autonomousBackgroundAnimation = new FillAnimation(sideStrips, Color.kBlack);
		this.autonomousDelayingAnimation = new BarAnimation(sideStrips, (p) -> Color.kGreen);
		this.autonomousRunningAnimation = new WaveAnimation(sideStrips, (time, pos) -> WaveFunction.Sawtooth.applyAsDouble((time * 4) - (pos * 4)), InterpolationFunction.step.gradient(new Color(0,0,0.2), new Color(0.1,0.1,0)));
		this.autonomousFinishedAnimation = new BarAnimation(sideStrips, (p) -> Color.kGreen);
		this.autonomousOverrunAnimation = new FlashingAnimation(sideStrips, WaveFunction.Sawtooth.frequency(4.0), InterpolationFunction.step.gradient(Color.kBlack, Color.kRed));
		this.estopped = new FillAnimation(sideStrips, Color.kRed);
		this.tipped = new FlashingAnimation(sideStrips, WaveFunction.Sawtooth.frequency(2.0), InterpolationFunction.step.gradient(Color.kBlack, Color.kWhite));
		this.allianceColorAnimation = new AllianceColorAnimation(sideStrips, Color.kFirstBlue, Color.kRed);
		this.driverStationConnection = new StatusLightAnimation(sideStrips.substrip(0, 2), Color.kOrange, Color.kGreen);
		this.fmsConnection = new FillAnimation(sideStrips.substrip(0, 2), Color.kBlue);
		this.hopperCamConnection = new StatusLightAnimation(sideStrips.substrip(2, 3), Color.kOrange, Color.kGreen);
		this.hubZoomCamConnection = new StatusLightAnimation(sideStrips.substrip(3, 4), Color.kOrange, Color.kGreen);
		this.leftBroadCamConnection = new StatusLightAnimation(sideStrips.substrip(4, 5), Color.kOrange, Color.kGreen);
		this.rightBroadCamConnection = new StatusLightAnimation(sideStrips.substrip(5, 6), Color.kOrange, Color.kGreen);
		this.backBroadCamConnection = new StatusLightAnimation(sideStrips.substrip(6, 7), Color.kOrange, Color.kGreen);
		this.intakeCamConnection = new StatusLightAnimation(sideStrips.substrip(7, 8), Color.kOrange, Color.kGreen);

		this.hoodNotCalibratedAnimation = new FlashingAnimation(sideStrips.substrip(13, 16), WaveFunction.Sawtooth.frequency(2.0), InterpolationFunction.step.gradient(Color.kBlack, Color.kRed));
		this.hoodCalibratedAnimation = new FillAnimation(sideStrips.substrip(13, 16), Color.kGreen);

		this.hubShiftAnimation = new HubShiftAnimation(leftTowerStrip.parallel(rightTowerStrip), Color.kGreen, Color.kRed, new Color(0.0, 0.2, 0.0), Color.kRed);

		this.loadingNotifier = new Notifier(() -> {
			synchronized(this) {
				this.bootingAnimation.apply();
				this.hardwareStrip.refresh();
			}
		});
		System.out.println("[Init Leds] Starting Loading Notifier");
		this.loadingNotifier.startPeriodic(RobotConstants.rioUpdatePeriodSecs);
	}

	public final FlashingAnimation bootingAnimation;
	public final StatusLightAnimation driverStationConnection;
	public final FillAnimation fmsConnection;
	public final StatusLightAnimation hopperCamConnection;
	public final StatusLightAnimation hubZoomCamConnection;
	public final StatusLightAnimation leftBroadCamConnection;
	public final StatusLightAnimation rightBroadCamConnection;
	public final StatusLightAnimation backBroadCamConnection;
	public final StatusLightAnimation intakeCamConnection;
	public final FillAnimation estopped;
	public final FillAnimation autonomousBackgroundAnimation;
	public final BarAnimation autonomousDelayingAnimation;
	public final WaveAnimation autonomousRunningAnimation;
	public final BarAnimation autonomousFinishedAnimation;
	public final FlashingAnimation autonomousOverrunAnimation;
	public final FlashingAnimation tipped;
	public final AllianceColorAnimation allianceColorAnimation;

	public final HubShiftAnimation hubShiftAnimation;

	public final FlashingAnimation hoodNotCalibratedAnimation;
	public final FillAnimation hoodCalibratedAnimation;

	private int skippedFrames = 0;
	private static final int frameSkipAmount = 15;

	@Override
	public void periodic() {
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Leds/Before");
		this.driverStationConnection.setStatus(DriverStation.isDSAttached());
		this.fmsConnection.setFlag(DriverStation.isFMSAttached());
		this.estopped.setFlag(DriverStation.isEStopped());
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Leds/Periodic");
		LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Leds");
	}

	@Override
	public synchronized void postCommandPeriodic() {
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Leds/Before");
		if (skippedFrames < frameSkipAmount) {
			skippedFrames += 1;
			return;
		}
		this.loadingNotifier.stop();

		this.allianceColorAnimation.apply();

		if (DriverStation.isDisabled()) {
			this.driverStationConnection.apply();
			this.fmsConnection.applyIfFlagged();
			this.hopperCamConnection.apply();
			this.hubZoomCamConnection.apply();
			this.leftBroadCamConnection.apply();
			this.rightBroadCamConnection.apply();
			this.backBroadCamConnection.apply();
			this.intakeCamConnection.apply();

			this.hoodCalibratedAnimation.applyIfFlagged();
		} else {
			this.hubShiftAnimation.apply();
		}

		this.autonomousBackgroundAnimation.applyIfFlagged();
		this.autonomousDelayingAnimation.applyIfFlagged();
		this.autonomousRunningAnimation.applyIfFlagged();
		this.autonomousFinishedAnimation.applyIfFlagged();
		this.autonomousOverrunAnimation.applyIfFlagged();

		this.hoodNotCalibratedAnimation.applyIfFlagged();

		this.tipped.applyIfFlagged();
		this.estopped.applyIfFlagged();

		this.hardwareStrip.refresh();
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Leds/Periodic");
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Leds");
	}
}
