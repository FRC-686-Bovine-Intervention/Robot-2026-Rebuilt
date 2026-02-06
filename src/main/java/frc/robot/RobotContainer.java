// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climber.Climber;
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
import frc.robot.subsystems.intake.rollers.RollersIO;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.indexer.IndexerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.Perspective;
import frc.util.controllers.Joystick;
import frc.util.controllers.XboxController;
import frc.util.robotStructure.Mechanism3d;
import frc.util.misc.Cluster;

public class RobotContainer {
	// Subsystems
	public final Drive drive;
	public final Shooter shooter;
	public final Intake intake;
	public final Rollers rollers;
	public final Climber climber;

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
					new OdometryTimestampIOOdometryThread(),
					new GyroIOPigeon2(),
					Arrays.stream(DriveConstants.moduleConstants)
						.map(ModuleIOFalcon550::new)
						.toArray(ModuleIO[]::new)
				);
				this.shooter = new Shooter(
					new Flywheels(new FlywheelsIO() {}),
					new Hood(new HoodIO() {})
				);
				this.intake = new Intake(
					new frc.robot.subsystems.intake.rollers.Rollers(new RollersIO() {})
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIO() {})
				);
				this.climber = new Climber(

				);
			}
			case SIM -> {
				this.drive = new Drive(
					new OdometryTimestampIOSim(),
					new GyroIO() {},
					Arrays.stream(DriveConstants.moduleConstants)
						.map(ModuleIOSim::new)
						.toArray(ModuleIO[]::new)
				);
				this.shooter = new Shooter(
					new Flywheels(new FlywheelsIO() {}),
					new Hood(new HoodIO() {})
				);
				this.intake = new Intake(
					new frc.robot.subsystems.intake.rollers.Rollers(new RollersIO() {})
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIO() {})
				);
				this.climber = new Climber(

				);
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
					new Flywheels(new FlywheelsIO() {}),
					new Hood(new HoodIO() {})
				);
				this.intake = new Intake(
					new frc.robot.subsystems.intake.rollers.Rollers(new RollersIO() {})
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIO() {})
				);
				this.climber = new Climber(

				);
			}
		}

		// Initialize vision systems with camera pipelines
		this.apriltagVision = new ApriltagVision(

		);
		this.objectVision = new ObjectVision(

		);

		// Setup robot structure
		// this.drive.structureRoot

		// ;

		// Register Mechanism3ds
		Mechanism3d.registerMechs(

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
				for (var ball : fuel) {
					fuelPoints.add(ball.fieldPos);
				}
				var clusters = Cluster.formClusters(fuelPoints, 0.3);
				var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
				double chosenClusterScore = 0;
				for (var cluster : clusters) {
					var center = cluster.getWeightedCenter();
					double dist = robotPose.getTranslation().getDistance(center);
					double density = cluster.getDensity();
					double count = cluster.getMemberCount();
					double score = (density * count)/dist; 	//Will end up changing

					if (score > chosenClusterScore) {
						chosenCluster = cluster;
						chosenClusterScore = score;
					}
				}

				start = chosenCluster.getClosest(robotPose.getTranslation());
			}

			@Override
			public void execute() {
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
		});
	}
}
