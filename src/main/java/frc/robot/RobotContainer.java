// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.Set;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.hook.Hook;
import frc.robot.subsystems.climber.hook.HookIO;
import frc.robot.subsystems.climber.hook.HookIOSim;
import frc.robot.subsystems.climber.hook.HookIOTalonFX;
import frc.robot.subsystems.commonDevices.CommonCANdi;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import frc.robot.subsystems.intake.rollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.intake.slam.IntakeSlamIO;
import frc.robot.subsystems.intake.slam.IntakeSlamIOSim;
import frc.robot.subsystems.intake.slam.IntakeSlamIOTalonFX;
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
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.Perspective;
import frc.util.controllers.Joystick;
import frc.util.controllers.XboxController;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.Mechanism3d;

public class RobotContainer {
	// Subsystems
	public final Drive drive;
	public final Shooter shooter;
	public final Intake intake;
	public final Rollers rollers;
	public final Climber climber;
	public final ExtensionSystem extensionSystem;

	// Vision
	public final ApriltagVision apriltagVision;
	public final ObjectVision objectVision;

	// Auto
	public final AutoManager autoManager;

	// Event Loops
	public final EventLoop automationsLoop = new EventLoop();

	// Controllers
	private final XboxController driveController = new XboxController(0);
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
					new Flywheel(FlywheelConstants.leftFlywheelConfig, new FlywheelIOTalonFX(FlywheelConstants.leftFlywheelConfig)),
					new Flywheel(FlywheelConstants.rightFlywheelConfig, new FlywheelIOTalonFX(FlywheelConstants.rightFlywheelConfig)),
					new Hood(new HoodIO() {}),
					new AimingSystem(
						new InterpolationShootingCalc(),
						new InterpolationPassingCalc()
					)
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIOTalonFX()),
					new IntakeSlam(new IntakeSlamIOTalonFX())
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIOTalonFX()),
					new Agitator(new AgitatorIOTalonFX()),
					new Feeder(new FeederIOTalonFX()),
					new RollerSensorsIOCANdi(commonCANdi)
				);
				this.climber = new Climber(
					new Hook(new HookIOTalonFX())
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
					new Flywheel(FlywheelConstants.leftFlywheelConfig, new FlywheelIO() {}),
					new Flywheel(FlywheelConstants.rightFlywheelConfig, new FlywheelIO() {}),
					new Hood(new HoodIO() {}),
					new AimingSystem(
						new InterpolationShootingCalc(),
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
				this.climber = new Climber(
					new Hook(new HookIOSim())
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
					new Flywheel(FlywheelConstants.leftFlywheelConfig, new FlywheelIO() {}),
					new Flywheel(FlywheelConstants.rightFlywheelConfig, new FlywheelIO() {}),
					new Hood(new HoodIO() {}),
					new AimingSystem(
						new InterpolationShootingCalc(),
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
				this.climber = new Climber(
					new Hook(new HookIO() {})
				);
			}
		}

		this.extensionSystem = new ExtensionSystem();

		// Initialize vision systems with camera pipelines
		this.apriltagVision = new ApriltagVision(

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
			.addChild(this.climber.hook.mech)
		;

		// Register Mechanism3ds
		Mechanism3d.registerMechs(
			this.intake.slam.driverMech,
			this.intake.slam.followerMech,
			this.intake.slam.couplerMech,
			this.shooter.hood.mech,
			this.climber.hook.mech
		);

		System.out.println("[Init RobotContainer] Configuring Commands");
		this.configureCommands();

		System.out.println("[Init RobotContainer] Configuring Notifications");

		System.out.println("[Init RobotContainer] Configuring Autonomous Modes");

		var autoSelector = new AutoSelector("Auto Selector");
		// Add autonomous routines to autonomous selector

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
		// Setup joystick driving as default command for drivetrain
		this.drive.translationSubsystem.setDefaultCommand(new Command() {
			{
				this.setName("Driver Controlled");
				this.addRequirements(drive.translationSubsystem);
			}
			private final Joystick driveJoystick = driveController.leftStick.smoothRadialDeadband(0.05).radialSensitivity(0.5);
			@Override
			public void execute() {
				var joyX = +driveJoystick.y().getAsDouble();
				var joyY = -driveJoystick.x().getAsDouble();

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
		});
		this.drive.rotationalSubsystem.setDefaultCommand(new Command() {
			{
				this.setName("Drive Controlled");
				this.addRequirements(drive.rotationalSubsystem);
			}
			private final Joystick.Axis axis = driveController.leftTrigger.add(driveController.rightTrigger.invert()).smoothDeadband(0.05).sensitivity(0.5);
			@Override
			public void execute() {
				var omega = this.axis.getAsDouble() * DriveConstants.maxTurnRate.in(RadiansPerSecond);

				drive.rotationalSubsystem.driveVelocity(omega);
			}
			@Override
			public void end(boolean interrupted) {
				drive.rotationalSubsystem.stop();
			}
		});

		final var intakeRollersIdleCommand = this.intake.rollers.idle();
		final var intakeRollersIntakeCommand = this.intake.rollers.intake();
		final var intakeRetractCommand = this.intake.slam.retract();
		final var intakeDeployCommand = this.intake.slam.deploy(this.extensionSystem);

		this.intake.rollers.setDefaultCommand(intakeRollersIdleCommand);
		this.intake.slam.setDefaultCommand(intakeRetractCommand);

		final var leftFlywheelIdleCommand = this.shooter.leftFlywheel.idle();
		final var rightFlywheelIdleCommand = this.shooter.rightFlywheel.idle();
		final var hoodIdleCommand = this.shooter.hood.idle();
		final var aimAtHubCommand = Commands.parallel(
			this.shooter.aimingSystem.aimAtHub(
				RobotState.getInstance()::getEstimatedGlobalPose,
				this.drive::getFieldMeasuredSpeeds,
				() -> Translation3d.kZero
			).repeatedly(),
			this.shooter.aimLeftFlywheelAtHub(),
			this.shooter.aimRightFlywheelAtHub(),
			this.shooter.aimHoodAtHub(),
			this.shooter.aimDriveAtHub(this.drive.rotationalSubsystem)
		).withName("Aim at Hub");
		final var aimToPassCommand = Commands.parallel(
			this.shooter.aimingSystem.aimToPass(
				RobotState.getInstance()::getEstimatedGlobalPose,
				this.drive::getFieldMeasuredSpeeds,
				() -> Translation3d.kZero
			).repeatedly(),
			this.shooter.aimLeftFlywheelToPass(),
			this.shooter.aimRightFlywheelToPass(),
			this.shooter.aimHoodToPass(),
			this.shooter.aimDriveToPass(this.drive.rotationalSubsystem)
		).withName("Aim to Pass");

		this.shooter.leftFlywheel.setDefaultCommand(leftFlywheelIdleCommand);
		this.shooter.rightFlywheel.setDefaultCommand(rightFlywheelIdleCommand);
		this.shooter.hood.setDefaultCommand(hoodIdleCommand);

		final var climberRetractCommand = this.climber.hook.retract();
		final var climberDeployCommand = this.climber.hook.deploy();
		final var climberClimbCommand = this.climber.hook.climb();

		this.climber.hook.setDefaultCommand(climberRetractCommand);

		// Auto calibrate hood if not calibrated
		// new Trigger(this.automationsLoop, () -> !this.shooter.hood.isCalibrated() && DriverStation.isEnabled()).whileTrue(this.shooter.hood.calibrate());

		// Auto calibrate hook if not calibrated
		new Trigger(this.automationsLoop, () -> /* !this.climber.hook.isCalibrated() &&  */DriverStation.isEnabled()).whileTrue(this.climber.hook.calibrate());

		// Setup position reset command
		// this.driveController.leftStickButton().and(this.driveController.rightStickButton()).onTrue(Commands.runOnce(() -> RobotState.getInstance().resetPose(
		// 	new Pose2d(
		// 		14.45,
		// 		5,
		// 		Rotation2d.kZero
		// 	)
		// )));

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
						CommandScheduler.getInstance().schedule(intakeRetractCommand);
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

		CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
			if (this.driveController.hid.getRightBumperButtonPressed()) {
				CommandScheduler.getInstance().schedule(aimAtHubCommand);
			}
			if (this.driveController.hid.getRightBumperButtonReleased()) {
				CommandScheduler.getInstance().cancel(aimAtHubCommand);
			}
		});
	}
}
