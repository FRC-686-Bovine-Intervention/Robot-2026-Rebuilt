// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.automations.BumpMitigation;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.hook.Hook;
import frc.robot.subsystems.climber.hook.HookIO;
import frc.robot.subsystems.climber.hook.HookIOSim;
import frc.robot.subsystems.commonDevices.CommonCANdi;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.commands.WheelRadiusCalibration;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.modules.ModuleIO;
import frc.robot.subsystems.drive.modules.ModuleIOSim;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIO;
import frc.robot.subsystems.drive.odometry.OdometryTimestampIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.intake.slam.IntakeSlamIO;
import frc.robot.subsystems.intake.slam.IntakeSlamIOSim;
import frc.robot.subsystems.rollers.RollerSensorsIO;
import frc.robot.subsystems.rollers.RollerSensorsIOCANdi;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.agitator.Agitator;
import frc.robot.subsystems.rollers.agitator.AgitatorIO;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.feeder.FeederIO;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.indexer.IndexerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.aiming.passing.InterpolationPassingCalc;
import frc.robot.subsystems.shooter.aiming.shooting.InterpolationShootingCalc;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIOPhoton;
import frc.robot.subsystems.vision.object.ObjectPipeline;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.Perspective;
import frc.util.controllers.XboxController;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.Mechanism3d;
import frc.util.misc.Cluster;

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
					new Hood(new HoodIOSim(commonCANdi)),
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
			new ObjectPipeline(new Camera(new CameraIOPhoton("Intake"), "Intake", new Transform3d(
				Inches.of(0),
				Inches.of(0),
				Inches.of(7.5),
				Rotation3d.kZero
			), null), 0)
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
			this.shooter.hood.mech,
			this.intake.slam.followerMech,
			this.intake.slam.driverMech,
			this.intake.slam.couplerMech,
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
		// Construct Commands
		final var translationJoystick = this.driveController.leftStick
			.smoothRadialDeadband(0.05)
			.radialSensitivity(0.5)
		;
		final var rotateAxis = this.driveController.leftTrigger
			.add(this.driveController.rightTrigger.invert())
			.smoothDeadband(0.05)
			.sensitivity(0.5)
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
				var omega = rotateAxis.getAsDouble() * DriveConstants.maxTurnRate.in(RadiansPerSecond);

				drive.rotationalSubsystem.driveVelocity(omega);
			}

			@Override
			public void end(boolean interrupted) {
				drive.rotationalSubsystem.stop();
			}
		};

		final var intakeRollersIdleCommand = this.intake.rollers.idle();
		final var intakeRollersIntakeCommand = this.intake.rollers.intake();
		final var intakeStowCommand = this.intake.slam.stow();
		final var intakeDeployCommand = this.intake.slam.deploy(this.extensionSystem);

		final var rollersIndexerIdleCommand = this.rollers.indexer.idle();
		final var rollersFeederIdleCommand = this.rollers.feeder.idle();
		final var rollersAgitatorIdleCommand = this.rollers.agitiator.idle();
		final var rollersFeedCommand =
			Commands.parallel(
				this.rollers.indexer.index(),
				this.rollers.feeder.feed(),
				this.rollers.agitiator.index()
			)
			.withName("Feed")
		;

		final var leftFlywheelIdleCommand = this.shooter.leftFlywheel.idle();
		final var rightFlywheelIdleCommand = this.shooter.rightFlywheel.idle();
		final var hoodStowCommand = this.shooter.hood.stow();

		final var aimAtHubCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimAtHub(
					RobotState.getInstance()::getEstimatedGlobalPose,
					this.drive::getFieldMeasuredSpeeds,
					FieldConstants.hubAimPoint::getOurs
				).repeatedly(),
				this.shooter.aimLeftFlywheelAtHub(),
				this.shooter.aimRightFlywheelAtHub(),
				this.shooter.aimHoodAtHub(),
				this.shooter.aimDriveAtHub(this.drive.rotationalSubsystem)
			)
			.withName("Aim at Hub")
		;
		final var aimToPassCommand =
			Commands.parallel(
				this.shooter.aimingSystem.aimToPass(
					RobotState.getInstance()::getEstimatedGlobalPose,
					this.drive::getFieldMeasuredSpeeds,
					() -> Translation3d.kZero
				).repeatedly(),
				this.shooter.aimLeftFlywheelToPass(),
				this.shooter.aimRightFlywheelToPass(),
				this.shooter.aimHoodToPass(),
				this.shooter.aimDriveToPass(this.drive.rotationalSubsystem)
			)
			.withName("Aim to Pass")
		;

		final var climberStowCommand = this.climber.hook.stow();
		final var climberDeployCommand = this.climber.hook.deploy();
		final var climberClimbCommand = this.climber.hook.climb();

		// Set default commands
		this.intake.rollers.setDefaultCommand(intakeRollersIdleCommand);
		this.intake.slam.setDefaultCommand(intakeStowCommand);
		this.rollers.indexer.setDefaultCommand(rollersIndexerIdleCommand);
		this.rollers.feeder.setDefaultCommand(rollersFeederIdleCommand);
		this.rollers.agitiator.setDefaultCommand(rollersAgitatorIdleCommand);
		this.shooter.hood.setDefaultCommand(hoodStowCommand);
		this.shooter.leftFlywheel.setDefaultCommand(leftFlywheelIdleCommand);
		this.shooter.rightFlywheel.setDefaultCommand(rightFlywheelIdleCommand);
		this.climber.hook.setDefaultCommand(climberStowCommand);

		// Bind automations
		this.automationsLoop.bind(new BumpMitigation(this.drive));
		new Trigger(this.automationsLoop, () -> !this.shooter.hood.isCalibrated() && DriverStation.isEnabled()).whileTrue(this.shooter.hood.calibrate());
		new Trigger(this.automationsLoop, () -> !this.climber.hook.isCalibrated() && DriverStation.isEnabled()).whileTrue(this.climber.hook.calibrate());

		// Bind buttons
		new Trigger(() -> translationJoystick.magnitude() > 0.0).whileTrue(driveTranslationCommand);
		new Trigger(() -> Math.abs(rotateAxis.getAsDouble()) > 0.0).whileTrue(driveRotateCommand);


		// Setup position reset command
		this.driveController.leftStickButton().and(this.driveController.rightStickButton()).onTrue(Commands.runOnce(() -> RobotState.getInstance().resetPose(FieldConstants.hubFrontRobotPose.getOurs())));
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

		// Setup position reset command
		// this.driveController.leftStickButton().and(this.driveController.rightStickButton()).onTrue(Commands.runOnce(() -> RobotState.getInstance().resetPose(
		// 	new Pose2d(
		// 		14.45,
		// 		5,
		// 		Rotation2d.kZero
		// 	)
		// )));

		this.driveController.rightBumper().onTrue(new Command() {
			{
				setName("Auto Intake");
				addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);
			}

			Cluster chosenCluster;
			Translation2d start;
			boolean hasReachedStart = false;

			@Override
			public void initialize() {
				var fuel = objectVision.getTrackedObjectsOfType(0);
				List<Translation2d> fuelPoints = new ArrayList<>();
				// for (var ball : fuel) {
				// 	fuelPoints.add(ball.fieldPos);
				// }
				fuelPoints = List.of(
					new Translation2d(1.0, 1.0),
					new Translation2d(1.1, 1.1),
					new Translation2d(1.0,1.2),
					new Translation2d(2.0, 2.0),
					new Translation2d(2.5, 2.5),
					new Translation2d(3.0, 3.0)
				);

				Logger.recordOutput("DEBUG/AutoIntake/FuelPoints", fuelPoints.stream().map((point) -> new Pose2d(point, Rotation2d.kZero)).toArray(Pose2d[]::new));

				var clusters = Cluster.formClusters(fuelPoints, 0.3);
				Logger.recordOutput("DEBUG/AutoIntake/Clusters", clusters.stream().map((cluster) -> new Pose2d(cluster.getWeightedCenter(), Rotation2d.kZero)).toArray(Pose2d[]::new));
				// Logger.recordOutput("DEBUG/AutoIntake/ClusterSizes", clusters.stream().map((c) -> c.getMemberCount()).toArray(int[]::new));

				var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
				double chosenClusterScore = 0;
				for (var cluster : clusters) {
					var center = cluster.getWeightedCenter();
					double dist = robotPose.getTranslation().getDistance(center);
					double density = cluster.getDensity();
					Logger.recordOutput("DEBUG/AutoIntake/Density/" + clusters.indexOf(cluster), density);
					double count = cluster.getMemberCount();
					Logger.recordOutput("DEBUG/AutoIntake/ClusterCount/" + clusters.indexOf(cluster), count);
					ChassisSpeeds robotRelative = drive.getRobotMeasuredSpeeds();

					Translation2d clusterToRobot = new Pose2d(center, Rotation2d.kZero).relativeTo(robotPose).getTranslation();
					clusterToRobot = clusterToRobot.getNorm() != 0 ? clusterToRobot.div(clusterToRobot.getNorm()) : Translation2d.kZero;

					Translation2d robotVelocity = new Translation2d(robotRelative.vxMetersPerSecond, robotRelative.vyMetersPerSecond);
					robotVelocity = robotVelocity.getNorm() != 0 ? robotVelocity.div(robotVelocity.getNorm()) : Translation2d.kZero;

					double angleScore = Math.acos(clusterToRobot.dot(robotVelocity));
					Logger.recordOutput("DEBUG/AutoIntake/AngleScore/" + clusters.indexOf(cluster), angleScore);

					double score = (density * count)/(dist * angleScore); 	//Will end up changing
					Logger.recordOutput("DEBUG/AutoIntake/ClusterScore/" + clusters.indexOf(cluster), score);

					if (score > chosenClusterScore) {
						Logger.recordOutput("DEBUG/AutoIntake/ChosenClusterChanged", true);
						chosenCluster = cluster;
						chosenClusterScore = score;
					}
				}

				Logger.recordOutput("DEBUG/AutoIntake/ChosenClusterIndex", clusters.indexOf(chosenCluster));
				Logger.recordOutput("DEBUG/AutoIntake/ChosenClusterClosest", chosenCluster == null ? null : chosenCluster.getClosest(robotPose.getTranslation()));
				Logger.recordOutput("DEBUG/AutoIntake/LineOfBestFit", chosenCluster == null ? null : Arrays.stream(chosenCluster.getLineOfBestFit()).map((point) -> new Pose2d(point, Rotation2d.kZero)).toArray(Pose2d[]::new));
				start = chosenCluster == null ? null : chosenCluster.getClosest(robotPose.getTranslation());
			}

			@Override
			public void execute() {
				if (start != null) {
					var pose = RobotState.getInstance().getEstimatedGlobalPose();
				if (!hasReachedStart) {
					var startRobotRelative = new Pose2d(start, Rotation2d.kZero).relativeTo(pose).getTranslation();
					var angle = new Rotation2d(Math.atan2(startRobotRelative.getY(), startRobotRelative.getX()));
					drive.translationSubsystem.simplePIDTo(() -> startRobotRelative);
					drive.rotationalSubsystem.pidControlledHeading(() -> angle);
				} else {
					chosenCluster.resetReduction();
					chosenCluster.reduceToAllAheadOf(pose);
					Translation2d[] line = chosenCluster.getLineOfBestFit();
					Translation2d closestLinePoint;
					Translation2d farthestLinePoint;
					if (line[0].getDistance(pose.getTranslation()) < line[1].getDistance(pose.getTranslation())) {
						closestLinePoint = line[0];
						farthestLinePoint = line[1];
					} else {
						closestLinePoint = line[1];
						farthestLinePoint = line[0];
					}
					closestLinePoint = new Pose2d(closestLinePoint, Rotation2d.kZero).relativeTo(pose).getTranslation();
					farthestLinePoint = new Pose2d(farthestLinePoint, Rotation2d.kZero).relativeTo(pose).getTranslation();
					var velocity = farthestLinePoint.minus(closestLinePoint);
					velocity = velocity.div(velocity.getNorm()).times(3);
					var angle = new Rotation2d(Math.atan2(velocity.getY(), velocity.getX()));
					drive.translationSubsystem.driveVelocity(velocity.getX(), velocity.getY());
					drive.rotationalSubsystem.pidControlledHeading(() -> angle);
				}
				}
			}
		});

		// CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
		// 	if (this.driveController.hid.getRightBumperButtonPressed()) {
		// 		CommandScheduler.getInstance().schedule(aimAtHubCommand);
		// 	}
		// 	if (this.driveController.hid.getRightBumperButtonReleased()) {
		// 		CommandScheduler.getInstance().cancel(aimAtHubCommand);
		// 	}
		// });
	}
}
