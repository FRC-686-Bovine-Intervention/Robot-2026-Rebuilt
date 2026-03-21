// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.routines.DoubleSwipe;
import frc.robot.auto.routines.Preloads;
import frc.robot.automations.AutoFeed;
import frc.robot.automations.HubShiftNotifications;
import frc.robot.automations.IntakeDeployHysteresis;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.commonDevices.CommonCANdi;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.drive.commands.WheelRadiusCalibration;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.modules.ModuleIO;
import frc.robot.subsystems.drive.modules.ModuleIOFalcon550;
import frc.robot.subsystems.drive.modules.ModuleIOSim;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIO;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIOOdometryThread;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOSparkMax;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.intake.slam.IntakeSlamIO;
import frc.robot.subsystems.intake.slam.IntakeSlamIOSim;
import frc.robot.subsystems.intake.slam.IntakeSlamIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.RollerSensorsIO;
import frc.robot.subsystems.rollers.RollerSensorsIOCANdi;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.agitator.Agitator;
import frc.robot.subsystems.rollers.agitator.AgitatorIO;
import frc.robot.subsystems.rollers.agitator.AgitatorIOTalonFX;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.feeder.FeederIO;
import frc.robot.subsystems.rollers.feeder.FeederIOTalonFX;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.indexer.IndexerIO;
import frc.robot.subsystems.rollers.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.aiming.passing.InterpolationPassingCalc;
import frc.robot.subsystems.shooter.aiming.shooting.InterpolationShootingCalc;
import frc.robot.subsystems.shooter.aiming.shooting.PhysicsShootingCalc;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFXS;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.ApriltagPipeline;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO;
import frc.robot.subsystems.vision.cameras.CameraIOPhoton;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.EdgeDetector;
import frc.util.PIDGains;
import frc.util.Perspective;
import frc.util.controllers.XboxController;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.Mechanism3d;

public class RobotContainer {
	// Subsystems
	public final Drive drive;
	public final Shooter shooter;
	public final Intake intake;
	public final Rollers rollers;
	// public final Climber climber;
	public final ExtensionSystem extensionSystem;

	// Vision
	public final ApriltagVision apriltagVision;
	public final ObjectVision objectVision;

	public final Camera hubZoomCamera;
	public final Camera leftBroadCamera;
	public final Camera rightBroadCamera;
	public final Camera backBroadCamera;
	public final Camera intakeCamera;

	// Auto
	public final AutoManager autoManager;

	// Event Loops
	public final EventLoop automationsLoop = new EventLoop();

	// Controllers
	private final XboxController driveController = new XboxController(0, "Drive Controller");
	private final Joystick secondDriverJoystick = new Joystick(1);
	private final Trigger secondDriverOverride = new Trigger(() -> secondDriverJoystick.button(3, CommandScheduler.getInstance().getDefaultButtonLoop()).or(secondDriverJoystick.button(4, CommandScheduler.getInstance().getDefaultButtonLoop())).getAsBoolean());

	@SuppressWarnings("unused")
	private final CommandJoystick simJoystick = new CommandJoystick(5);

	@SuppressWarnings("resource")
	public RobotContainer() {
		System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());

		// Initialize subsystems with appropriate IO
		switch (RobotType.getMode()) {
			case REAL -> {
				var commonCANdi = new CommonCANdi();

				this.drive = new Drive(
					new OdometryTimestampIOOdometryThread(),
					new GyroIOPigeon2(),
					Arrays.stream(DriveConstants.moduleConstants)
						.map(ModuleIOFalcon550::new)
						.toArray(ModuleIO[]::new)
				);
				this.shooter = new Shooter(
					new Flywheel(new FlywheelIOTalonFX()),
					new Hood(new HoodIOTalonFXS(commonCANdi)),
					new AimingSystem(
						new InterpolationShootingCalc(),
						new InterpolationPassingCalc()
					)
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIOSparkMax()),
					new IntakeSlam(new IntakeSlamIOTalonFX())
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIOTalonFX()),
					new Agitator(new AgitatorIOTalonFX()),
					new Feeder(new FeederIOTalonFX()),
					new RollerSensorsIOCANdi(commonCANdi)
				);
				// this.climber = new Climber(
				// 	new Hook(new HookIO() {})
				// );

				this.hubZoomCamera = new Camera(
					new CameraIOPhoton("Left Top"),
					"Hub Zoom",
					VisionConstants.topLeftMount,
					(connected) -> {Leds.getInstance().hubZoomCamConnection.setStatus(connected);}
				);
				this.leftBroadCamera = new Camera(
					new CameraIOPhoton("Left Bottom"),
					"Left Broad",
					VisionConstants.bottomLeftMount,
					(connected) -> {Leds.getInstance().leftBroadCamConnection.setStatus(connected);}
				);
				this.rightBroadCamera = new Camera(
					new CameraIOPhoton("Right Top"),
					"Right Broad",
					VisionConstants.topRightMount,
					(connected) -> {Leds.getInstance().rightBroadCamConnection.setStatus(connected);}
				);
				this.backBroadCamera = new Camera(
					new CameraIOPhoton("Right Bottom"),
					"Back Broad",
					VisionConstants.bottomRightMount,
					(connected) -> {Leds.getInstance().backBroadCamConnection.setStatus(connected);}
				);
				this.intakeCamera = new Camera(
					new CameraIO() {},
					"Intake",
					VisionConstants.intakeMount,
					(connected) -> {Leds.getInstance().intakeCamConnection.setStatus(connected);}
				);

				commonCANdi.configSend();
			}
			case SIM -> {
				var commonCANdi = new CommonCANdi();

				this.drive = new Drive(
					new OdometryTimestampIOSim(),
					new GyroIO() {},
					Arrays.stream(DriveConstants.moduleConstants)
						.map(ModuleIOSim::new)
						.toArray(ModuleIO[]::new)
				);
				this.shooter = new Shooter(
					new Flywheel(new FlywheelIO() {}),
					new Hood(new HoodIOSim(commonCANdi)),
					new AimingSystem(
						new PhysicsShootingCalc(),
						new InterpolationPassingCalc()
					)
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIO() {}),
					new IntakeSlam(new IntakeSlamIOSim())
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIO() {}),
					new Agitator(new AgitatorIO() {}),
					new Feeder(new FeederIO() {}),
					new RollerSensorsIOCANdi(commonCANdi)
				);
				// this.climber = new Climber(
				// 	new Hook(new HookIOSim())
				// );

				this.hubZoomCamera = new Camera(
					new CameraIO() {},
					"Hub Zoom",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().hubZoomCamConnection.setStatus(connected);}
				);
				this.leftBroadCamera = new Camera(
					new CameraIO() {},
					"Left Broad",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().leftBroadCamConnection.setStatus(connected);}
				);
				this.rightBroadCamera = new Camera(
					new CameraIO() {},
					"Right Broad",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().rightBroadCamConnection.setStatus(connected);}
				);
				this.backBroadCamera = new Camera(
					new CameraIO() {},
					"Back Broad",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().backBroadCamConnection.setStatus(connected);}
				);
				this.intakeCamera = new Camera(
					new CameraIO() {},
					"Intake",
					VisionConstants.intakeMount,
					(connected) -> {Leds.getInstance().intakeCamConnection.setStatus(connected);}
				);

				commonCANdi.configSend();
			}
			default -> {
				this.drive = new Drive(
					new OdometryTimestampIO() {},
					new GyroIO() {},
					new ModuleIO(){},
					new ModuleIO(){},
					new ModuleIO(){},
					new ModuleIO(){}
				);
				this.shooter = new Shooter(
					new Flywheel(new FlywheelIO() {}),
					new Hood(new HoodIO() {}),
					new AimingSystem(
						new PhysicsShootingCalc(),
						new InterpolationPassingCalc()
					)
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIO() {}),
					new IntakeSlam(new IntakeSlamIO() {})
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIO() {}),
					new Agitator(new AgitatorIO() {}),
					new Feeder(new FeederIO() {}),
					new RollerSensorsIO() {}
				);
				// this.climber = new Climber(
				// 	new Hook(new HookIO() {})
				// );

				this.hubZoomCamera = new Camera(
					new CameraIO() {},
					"Hub Zoom",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().hubZoomCamConnection.setStatus(connected);}
				);
				this.leftBroadCamera = new Camera(
					new CameraIO() {},
					"Left Broad",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().leftBroadCamConnection.setStatus(connected);}
				);
				this.rightBroadCamera = new Camera(
					new CameraIO() {},
					"Right Broad",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().rightBroadCamConnection.setStatus(connected);}
				);
				this.backBroadCamera = new Camera(
					new CameraIO() {},
					"Back Broad",
					Transform3d.kZero,
					(connected) -> {Leds.getInstance().backBroadCamConnection.setStatus(connected);}
				);
				this.intakeCamera = new Camera(
					new CameraIO() {},
					"Intake",
					VisionConstants.intakeMount,
					(connected) -> {Leds.getInstance().intakeCamConnection.setStatus(connected);}
				);
			}
		}

		this.extensionSystem = new ExtensionSystem();

		// Initialize vision systems with camera pipelines
		this.apriltagVision = new ApriltagVision(
			new ApriltagPipeline(this.hubZoomCamera, 0, 1.0)
			// new ApriltagPipeline(this.leftBroadCamera, 0, 2.0),
			// new ApriltagPipeline(this.rightBroadCamera, 0, 2.0),
			// new ApriltagPipeline(this.backBroadCamera, 0, 2.0)
		);
		this.objectVision = new ObjectVision(

		);

		// Setup robot structure
		this.drive.structureRoot
			.addChild(this.intake.slam.driverMech
				.addChild(this.intake.slam.couplerMech)
			)
			.addChild(this.intake.slam.followerMech)
			.addChild(this.shooter.hood.mech)
			// .addChild(this.climber.hook.mech)
			.addChild(this.hubZoomCamera.mount)
			.addChild(this.leftBroadCamera.mount)
			.addChild(this.rightBroadCamera.mount)
			.addChild(this.backBroadCamera.mount)
		;

		// Register Mechanism3ds
		Mechanism3d.registerMechs(
			this.shooter.hood.mech,
			this.intake.slam.followerMech,
			this.intake.slam.driverMech,
			this.intake.slam.couplerMech
			// this.climber.hook.mech
		);

		System.out.println("[Init RobotContainer] Configuring Commands");
		this.configureCommands();

		System.out.println("[Init RobotContainer] Configuring Notifications");

		System.out.println("[Init RobotContainer] Configuring Autonomous Modes");

		final var autoSelector = new AutoSelector("Auto Selector");
		// Add autonomous routines to autonomous selector
		autoSelector.addDefaultRoutine(new DoubleSwipe(this));
		autoSelector.addRoutine(new Preloads(this));

		this.autoManager = new AutoManager(autoSelector);

		System.out.println("[Init RobotContainer] Configuring System Check");
		SmartDashboard.putData("System Check/Drive/Spin",
			new Command() {
				private final Drive.Rotational rotationalSubsystem = drive.rotationalSubsystem;
				private final Timer timer = new Timer();
				{
					addRequirements(this.rotationalSubsystem);
					setName("TEST Spin");
				}
				public void initialize() {
					this.timer.restart();
				}
				public void execute() {
					this.rotationalSubsystem.driveVelocity(Math.sin(this.timer.get()) * 3);
				}
				public void end(boolean interrupted) {
					this.timer.stop();
					this.rotationalSubsystem.stop();
				}
			}
		);
		SmartDashboard.putData("System Check/Drive/Circle",
			new Command() {
				private final Drive.Translational translationSubsystem = drive.translationSubsystem;
				private final Timer timer = new Timer();
				{
					addRequirements(this.translationSubsystem);
					setName("TEST Circle");
				}
				public void initialize() {
					this.timer.restart();
				}
				public void execute() {
					this.translationSubsystem.driveVelocity(
						new ChassisSpeeds(
							Math.cos(this.timer.get()) * 0.01,
							Math.sin(this.timer.get()) * 0.01,
							0
						)
					);
				}
				public void end(boolean interrupted) {
					this.timer.stop();
					this.translationSubsystem.stop();
				}
			}
		);

		SmartDashboard.putData("Wheel Calibration", Commands.defer(
			() ->
				new WheelRadiusCalibration(
					drive,
					WheelRadiusCalibration.VOLTAGE_RAMP_RATE.get(),
					WheelRadiusCalibration.MAX_VOLTAGE.get()
				)
				.withName("Wheel Calibration"),
				Set.of(drive.translationSubsystem, drive.rotationalSubsystem)
			)
		);

		if (RobotConstants.tuningMode) {
			new Alert("Tuning mode active", AlertType.kInfo).set(true);
		}
	}

	private void configureCommands() {
		// Construct Commands
		final var translationJoystick = this.driveController.leftStick
			.smoothRadialDeadband(0.05)
			.radialSensitivity(0.5)
		;
		final Supplier<ChassisSpeeds> desiredTranslationalRobotVelo = () -> {
			var joyX = +translationJoystick.y().getAsDouble();
			var joyY = -translationJoystick.x().getAsDouble();

			var perspectiveForward = Perspective.getCurrent().getForwardDirection();
			var fieldX = joyX * perspectiveForward.getCos() - joyY * perspectiveForward.getSin();
			var fieldY = joyX * perspectiveForward.getSin() + joyY * perspectiveForward.getCos();

			var robotRot = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
			var robotX = fieldX * robotRot.getCos() - fieldY * -robotRot.getSin();
			var robotY = fieldX * -robotRot.getSin() + fieldY * robotRot.getCos();

			var driveX = robotX * DriveConstants.maxDriveSpeed.in(MetersPerSecond);
			var driveY = robotY * DriveConstants.maxDriveSpeed.in(MetersPerSecond);

			return new ChassisSpeeds(driveX, driveY, 0.0);
		};
		final var rotateAxis = this.driveController.leftTrigger
			.add(this.driveController.rightTrigger.invert())
			.smoothDeadband(0.01)
			.sensitivity(0.75)
		;
		final var driveTranslationCommand = new Command() {
			{
				this.setName("Driver Controlled");
				this.addRequirements(drive.translationSubsystem);
			}

			@Override
			public void execute() {
				var joyX = +translationJoystick.y().getAsDouble();
				var joyY = -translationJoystick.x().getAsDouble();

				var perspectiveForward = Perspective.getCurrent().getForwardDirection();
				var fieldX = joyX * perspectiveForward.getCos() - joyY * perspectiveForward.getSin();
				var fieldY = joyX * perspectiveForward.getSin() + joyY * perspectiveForward.getCos();

				var robotRot = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
				var robotX = fieldX * robotRot.getCos() - fieldY * -robotRot.getSin();
				var robotY = fieldX * -robotRot.getSin() + fieldY * robotRot.getCos();

				var driveX = robotX * DriveConstants.maxDriveSpeed.in(MetersPerSecond);
				var driveY = robotY * DriveConstants.maxDriveSpeed.in(MetersPerSecond);

				drive.translationSubsystem.driveVelocity(driveX, driveY);
			}

			@Override
			public void end(boolean interrupted) {
				drive.translationSubsystem.stop();
			}
		};
		final var driveRotateCommand = new Command() {
			{
				this.setName("Drive Controlled");
				this.addRequirements(drive.rotationalSubsystem);
			}

			@Override
			public void execute() {
				var omega = rotateAxis.getAsDouble() * DriveConstants.maxTurnRate.in(RadiansPerSecond) * 0.5;

				drive.rotationalSubsystem.driveVelocity(omega);
			}

			@Override
			public void end(boolean interrupted) {
				drive.rotationalSubsystem.stop();
			}
		};

		final var driveTankCommand = new Command() {
			private static final LoggedTunableNumber offsetThreshold = LoggedTunable.from("Controls/Tank Drive/Offset/Threshold", 0.25);
			private static final LoggedTunable<Angle> offsetAngle = LoggedTunable.from("Controls/Tank Drive/Offset/Angle", Degrees::of, 20.0);

			private static final LoggedTunable<LinearVelocity> linearThreshold = LoggedTunable.from("Controls/Tank Drive/Velo Threshold", MetersPerSecond::of, 0.5);
			private static final LoggedTunable<AngularVelocity> maxOmega = LoggedTunable.from("Controls/Tank Drive/Max Omega", RotationsPerSecond::of, 0.5);

			private static final LoggedTunable<PIDGains> pidGains = LoggedTunable.from(
				"Controls/Tank Drive/Azimuth PID",
				new PIDGains(
					3.5,
					0.0,
					0.0
				)
			);
			private final PIDController pid = new PIDController(pidGains.get().kP(), pidGains.get().kI(), pidGains.get().kD());

			{
				this.setName("Tank");
				this.addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);

				pid.enableContinuousInput(-Math.PI, Math.PI);
			}

			private double targetHeadingRads = 0.0;

			@Override
			public void initialize() {
				if (pidGains.hasChanged(this.hashCode())) {
					pidGains.get().update(this.pid);
				}

				this.targetHeadingRads = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians();
			}

			@Override
			public void execute() {
				var joyX = +translationJoystick.y().getAsDouble();
				var joyY = -translationJoystick.x().getAsDouble();

				var perspectiveForward = Perspective.getCurrent().getForwardDirection();
				var fieldX = joyX * perspectiveForward.getCos() - joyY * perspectiveForward.getSin();
				var fieldY = joyX * perspectiveForward.getSin() + joyY * perspectiveForward.getCos();

				var driveX = fieldX * DriveConstants.maxDriveSpeed.in(MetersPerSecond);
				var driveY = fieldY * DriveConstants.maxDriveSpeed.in(MetersPerSecond);

				var robotRot = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
				double robotX;
				double robotY;

				if (Math.hypot(driveX, driveY) > linearThreshold.get().in(MetersPerSecond)) {
					var rawTargetHeadingRads = Math.atan2(fieldY, fieldX);
					var offsetRads = 0.0;
					if (rotateAxis.getAsDouble() >= offsetThreshold.getAsDouble()) {
						offsetRads = +offsetAngle.get().in(Radians);
					} else if (rotateAxis.getAsDouble() <= -offsetThreshold.getAsDouble()) {
						offsetRads = -offsetAngle.get().in(Radians);
					}
					var robotHeadingOffsetRads = MathUtil.angleModulus(robotRot.getRadians() - offsetRads);
					this.targetHeadingRads = MathUtil.angleModulus(rawTargetHeadingRads + offsetRads);
					var offsetX = Math.max(driveX * +Math.cos(robotHeadingOffsetRads) - driveY * -Math.sin(robotHeadingOffsetRads), 0.0);
					robotX = offsetX * +Math.cos(offsetRads);
					robotY = offsetX * -Math.sin(offsetRads);
				} else {
					robotX = driveX * +robotRot.getCos() - driveY * -robotRot.getSin();
					robotY = driveX * -robotRot.getSin() + driveY * +robotRot.getCos();
				}

				var pidOut = this.pid.calculate(robotRot.getRadians(), this.targetHeadingRads);

				var omega = MathUtil.clamp(pidOut, -maxOmega.get().in(RadiansPerSecond), +maxOmega.get().in(RadiansPerSecond));

				drive.translationSubsystem.driveVelocity(robotX, robotY);
				drive.rotationalSubsystem.driveVelocity(omega);
			}

			@Override
			public void end(boolean interrupted) {
				drive.translationSubsystem.stop();
				drive.rotationalSubsystem.stop();
			}
		};

		final var intakeRollersIdleCommand = this.intake.rollers.idle();
		final var intakeRollersIntakeCommand = this.intake.rollers.intake();
		final var intakeStowCommand = this.intake.slam.stow();
		final var intakeDeployCommand = this.intake.slam.deploy(this.extensionSystem);

		final var rollersIndexerIdleCommand = this.rollers.indexer.idle();
		final var rollersFeederIdleCommand = this.rollers.feeder.idle();
		final var rollersAgitatorIdleCommand = this.rollers.agitator.idle();
		final var rollersFeedCommand =
			Commands.parallel(
				this.rollers.indexer.index(),
				this.rollers.feeder.feed(),
				this.rollers.agitator.index()
			)
			.withName("Feed")
		;

		final var flywheelIdleCommand = this.shooter.flywheel.idle();
		final var hoodStowCommand = this.shooter.hood.stow();

		final var aimAtHubCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					RobotState.getInstance()::getEstimatedGlobalPose,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHubWithXLock(this.drive, desiredTranslationalRobotVelo)
			)
			.withName("Aim at Hub")
		;
		final var aimAtHubFromHubFrontCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					FieldConstants.hubIntakeFrontRobotPose::getOurs,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHubWithXLock(this.drive, desiredTranslationalRobotVelo)
			)
			.withName("Aim at Hub from Hub Front")
		;
		final var aimAtHubFromLeftTrenchCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					FieldConstants.leftTrenchPresetShotPose::getOurs,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHubWithXLock(this.drive, desiredTranslationalRobotVelo)
			)
			.withName("Aim at Hub from Left Trench")
		;
		final var aimAtHubFromRightTrenchCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					FieldConstants.rightTrenchPresetShotPose::getOurs,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHubWithXLock(this.drive, desiredTranslationalRobotVelo)
			)
			.withName("Aim at Hub from Right Trench")
		;
		final var aimAtHubFromTowerCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					FieldConstants.towerPresetShotPose::getOurs,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHubWithXLock(this.drive, desiredTranslationalRobotVelo)
			)
			.withName("Aim at Hub from Tower")
		;
		final var aimToPassCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimToPass(
					RobotState.getInstance()::getEstimatedGlobalPose,
					this.drive::getFieldMeasuredSpeeds,
					() -> {
						if (RobotState.getInstance().getEstimatedGlobalPose().getY() > FieldConstants.fieldWidth.div(2.0).in(Meters)) {
							return FieldConstants.topPassPoint.getOurs();
						} else {
							return FieldConstants.botPassPoint.getOurs();
						}
					}
				).repeatedly(),
				this.shooter.aimFlywheelToPass(),
				this.shooter.aimHoodToPass(),
				this.shooter.aimDriveToPass(this.drive.rotationalSubsystem)
			)
			.withName("Aim to Pass")
		;

		// final var climberHookStowCommand = this.climber.hook.stow();
		// final var climberHookDeployCommand = this.climber.hook.deploy();
		// final var climberHookAutoDeployCommand = this.climber.hook.deploy().withName("Auto Deploy");
		// final var climberHookClimbCommand = this.climber.hook.climb();

		// Set default commands
		this.intake.rollers.setDefaultCommand(intakeRollersIdleCommand);
		this.intake.slam.setDefaultCommand(intakeStowCommand);

		this.rollers.indexer.setDefaultCommand(rollersIndexerIdleCommand);
		this.rollers.feeder.setDefaultCommand(rollersFeederIdleCommand);
		this.rollers.agitator.setDefaultCommand(rollersAgitatorIdleCommand);

		this.shooter.hood.setDefaultCommand(hoodStowCommand);
		this.shooter.flywheel.setDefaultCommand(flywheelIdleCommand);

		// this.climber.hook.setDefaultCommand(climberHookStowCommand);

		this.hubZoomCamera.setDefaultCommand(this.hubZoomCamera.setPipelineIndex(0));
		this.leftBroadCamera.setDefaultCommand(this.leftBroadCamera.setPipelineIndex(0));
		this.rightBroadCamera.setDefaultCommand(this.rightBroadCamera.setPipelineIndex(0));
		this.backBroadCamera.setDefaultCommand(this.backBroadCamera.setPipelineIndex(0));
		this.intakeCamera.setDefaultCommand(this.intakeCamera.setPipelineIndex(0));

		// Bind automations
		// this.automationsLoop.bind(new BumpMitigation(this.drive));
		// this.automationsLoop.bind(new TrenchMitigation(this.drive, this.intake.slam, this.extensionSystem, this.shooter.hood, intakeDeployCommand));
		this.automationsLoop.bind(new IntakeDeployHysteresis(this.intake.slam, intakeDeployCommand));
		// this.automationsLoop.bind(new HookAutoDeployHysteresis(this.climber.hook, climberHookAutoDeployCommand));
		// this.automationsLoop.bind(new AutoSpinUp(this.drive, this.shooter, intakeRollersIntakeCommand));
		// this.automationsLoop.bind(new AutoDriveAim(this.drive, this.shooter, intakeRollersIntakeCommand));
		this.automationsLoop.bind(new AutoFeed(this.drive, this.shooter, this.rollers, this.intake.slam, this.extensionSystem, this.driveController.y().or(secondDriverOverride)));
		this.automationsLoop.bind(new HubShiftNotifications(this.driveController));
		new Trigger(this.automationsLoop, () -> !this.shooter.hood.isCalibrated() && DriverStation.isEnabled()).whileTrue(this.shooter.hood.calibrate());
		// new Trigger(this.automationsLoop, () -> !this.climber.hook.isCalibrated() && DriverStation.isEnabled()).whileTrue(this.climber.hook.calibrate());

		// Bind buttons
		this.driveController.leftBumper().whileTrue(driveTankCommand);
		new Trigger(() -> 
			translationJoystick.magnitude() > 0.0
			&& !driveTankCommand.isScheduled()
			&& !aimAtHubCommand.isScheduled()
			&& !aimAtHubFromHubFrontCommand.isScheduled()
			&& !aimAtHubFromLeftTrenchCommand.isScheduled()
			&& !aimAtHubFromRightTrenchCommand.isScheduled()
			&& !aimAtHubFromTowerCommand.isScheduled()
		).whileTrue(driveTranslationCommand);
		new Trigger(() -> 
			Math.abs(rotateAxis.getAsDouble()) > 0.0
			&& !driveTankCommand.isScheduled()
			&& !aimAtHubCommand.isScheduled()
			&& !aimAtHubFromHubFrontCommand.isScheduled()
			&& !aimAtHubFromLeftTrenchCommand.isScheduled()
			&& !aimAtHubFromRightTrenchCommand.isScheduled()
			&& !aimAtHubFromTowerCommand.isScheduled()
		).whileTrue(driveRotateCommand);

		// Setup position reset command
		this.driveController.leftStickButton().and(this.driveController.rightStickButton()).onTrue(Commands.runOnce(() -> RobotState.getInstance().resetPose(FieldConstants.hubIntakeFrontRobotPose.getOurs())).ignoringDisable(true));
		this.driveController.rightStickButton().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetPose(new Pose2d(FieldConstants.hubIntakeFrontRobotPose.getOurs().getTranslation(), RobotState.getInstance().getEstimatedGlobalPose().getRotation()))).ignoringDisable(true));
		/*
		 * (A)
		 *  | Press: Deploy intake (if not deployed) and roll in
		 *  | Double Press: Retract intake
		 */
		final var intakeDoublePressThreshold = LoggedTunable.from("Controls/Intake/Double Press Threshold", Seconds::of, 0.25);
		final var intakeDoublePressTimer = new Timer();
		CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
			if (this.driveController.hid.getAButtonPressed()) {
				CommandScheduler.getInstance().schedule(intakeRollersIntakeCommand);
				CommandScheduler.getInstance().schedule(intakeDeployCommand);
			}
			if (this.driveController.hid.getAButtonReleased()) {
				CommandScheduler.getInstance().cancel(intakeRollersIntakeCommand);
				if (intakeDoublePressTimer.isRunning()) {
					if (!intakeDoublePressTimer.hasElapsed(intakeDoublePressThreshold.get().in(Seconds))) {
						CommandScheduler.getInstance().schedule(intakeStowCommand);
					}
				} else {
					intakeDoublePressTimer.start();
				}
			}
			if (intakeDoublePressTimer.hasElapsed(intakeDoublePressThreshold.get().in(Seconds))) {
				intakeDoublePressTimer.stop();
				intakeDoublePressTimer.reset();
			}
		});

		final var aimAutoTrigger = new EdgeDetector(false);
		final var aimHubTrigger = new EdgeDetector(false);
		final var aimLeftTrenchTrigger = new EdgeDetector(false);
		final var aimRightTrenchTrigger = new EdgeDetector(false);
		final var aimTowerTrigger = new EdgeDetector(false);

		CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
			var rightBumper = this.driveController.hid.getRightBumperButton();
			var rightStickRads = this.driveController.rightStick.radsFromPosYCCW();
			var rightStickMag = this.driveController.rightStick.magnitude() > 0.75;

			aimAutoTrigger.update(rightBumper);
			aimHubTrigger.update(rightBumper && rightStickMag && rightStickRads < Math.PI / 4.0 && rightStickRads > -Math.PI / 4.0);
			aimLeftTrenchTrigger.update(rightBumper && rightStickMag && rightStickRads < -Math.PI / 4.0 && rightStickRads > 3.0 * -Math.PI / 4.0);
			aimRightTrenchTrigger.update(rightBumper && rightStickMag && rightStickRads < 3.0 * Math.PI / 4.0 && rightStickRads > Math.PI / 4.0);
			aimTowerTrigger.update(rightBumper && rightStickMag && (rightStickRads > 3.0 * Math.PI / 4.0 || rightStickRads < 3.0 * -Math.PI / 4.0));

			if (aimAutoTrigger.risingEdge()) {
				if (FieldConstants.allianceZone.getOurs().withinBounds(RobotState.getInstance().getEstimatedGlobalPose().getTranslation())) {
					CommandScheduler.getInstance().schedule(aimAtHubCommand);
				} else {
					CommandScheduler.getInstance().schedule(aimToPassCommand);
				}
			}
			if (aimHubTrigger.risingEdge()) {
				CommandScheduler.getInstance().schedule(aimAtHubFromHubFrontCommand);
			}
			if (aimLeftTrenchTrigger.risingEdge()) {
				CommandScheduler.getInstance().schedule(aimAtHubFromLeftTrenchCommand);
			}
			if (aimRightTrenchTrigger.risingEdge()) {
				CommandScheduler.getInstance().schedule(aimAtHubFromRightTrenchCommand);
			}
			if (aimTowerTrigger.risingEdge()) {
				CommandScheduler.getInstance().schedule(aimAtHubFromTowerCommand);
			}
			if (aimAutoTrigger.fallingEdge()) {
				CommandScheduler.getInstance().cancel(aimAtHubCommand);
				CommandScheduler.getInstance().cancel(aimAtHubFromHubFrontCommand);
				CommandScheduler.getInstance().cancel(aimAtHubFromLeftTrenchCommand);
				CommandScheduler.getInstance().cancel(aimAtHubFromRightTrenchCommand);
				CommandScheduler.getInstance().cancel(aimAtHubFromTowerCommand);
				CommandScheduler.getInstance().cancel(aimToPassCommand);
			}
		});

		this.driveController.x().whileTrue(this.rollers.feed().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("Force Feed"));

		var traj = AutoCommons.loadBlueChoreoTrajectory("TestPath");

		this.driveController.povUp().whileTrue(Commands.defer(() -> new FollowTrajectoryCommand(this.drive, traj.getOurs(), false), Set.of(this.drive.translationSubsystem, this.drive.rotationalSubsystem)));
	}
}
