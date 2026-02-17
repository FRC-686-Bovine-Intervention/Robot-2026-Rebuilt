// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.ExtensionSystem;
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
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.intake.slam.IntakeSlamIO;
import frc.robot.subsystems.intake.slam.IntakeSlamIOSim;
import frc.robot.subsystems.intake.slam.IntakeSlamIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.indexer.IndexerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.Perspective;
import frc.util.controllers.Joystick;
import frc.util.controllers.XboxController;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.Mechanism3d;

public class RobotContainer {
	// Subsystems
	public final Drive drive;
	public final Shooter shooter;
	public final Intake intake;
	public final Rollers rollers;
	public final Climber climber;
	public final ExtensionSystem extension;

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
					new Flywheels(new FlywheelsIOTalonFX(HardwareDevices.leftFlywheelMotorMasterID, HardwareDevices.leftFlywheelMotorSlaveID)),
					new Flywheels(new FlywheelsIOTalonFX(HardwareDevices.centerFlywheelMotorMasterID, HardwareDevices.centerFlywheelMotorSlaveID)),
					new Flywheels(new FlywheelsIOTalonFX(HardwareDevices.rightFlywheelMotorMasterID, HardwareDevices.rightFlywheelMotorSlaveID)),
					new Hood(new HoodIO() {})
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIO() {}),
					new IntakeSlam(new IntakeSlamIOTalonFX())
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
					new Flywheels(new FlywheelsIO() {}),
					new Flywheels(new FlywheelsIO() {}),
					new Hood(new HoodIO() {})
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIO() {}),
					new IntakeSlam(new IntakeSlamIOSim())
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
					new Flywheels(new FlywheelsIO() {}),
					new Flywheels(new FlywheelsIO() {}),
					new Hood(new HoodIO() {})
				);
				this.intake = new Intake(
					new IntakeRollers(new IntakeRollersIO() {}),
					new IntakeSlam(new IntakeSlamIO() {})
				);
				this.rollers = new Rollers(
					new Indexer(new IndexerIO() {})
				);
				this.climber = new Climber(

				);
			}
		}

		this.extension = new ExtensionSystem();

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
		Joystick.Axis turnAxis = driveController.leftTrigger.add(driveController.rightTrigger.invert()).smoothDeadband(0.05).sensitivity(0.5);
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
			@Override
			public void execute() {
				var omega = turnAxis.getAsDouble() * DriveConstants.maxTurnRate.in(RadiansPerSecond);

				drive.rotationalSubsystem.driveVelocity(omega);
			}
			@Override
			public void end(boolean interrupted) {
				drive.rotationalSubsystem.stop();
			}
		});
		this.intake.rollers.setDefaultCommand(this.intake.rollers.idle());
		this.intake.slam.setDefaultCommand(this.intake.slam.retract());

		this.automationsLoop.bind(() -> {
			var robotPose = RobotState.getInstance().getEstimatedGlobalPose();

			var flCorner = robotPose.getTranslation().plus(RobotConstants.flBumperCorner.rotateBy(robotPose.getRotation()));
			var frCorner = robotPose.getTranslation().plus(RobotConstants.frBumperCorner.rotateBy(robotPose.getRotation()));
			var blCorner = robotPose.getTranslation().plus(RobotConstants.blBumperCorner.rotateBy(robotPose.getRotation()));
			var brCorner = robotPose.getTranslation().plus(RobotConstants.brBumperCorner.rotateBy(robotPose.getRotation()));

			Logger.recordOutput("CORNER DETECT/alliance zone/FL", FieldConstants.allianceZone.getOurs().withinBounds(flCorner));
			Logger.recordOutput("CORNER DETECT/alliance zone/FR", FieldConstants.allianceZone.getOurs().withinBounds(frCorner));
			Logger.recordOutput("CORNER DETECT/alliance zone/BL", FieldConstants.allianceZone.getOurs().withinBounds(blCorner));
			Logger.recordOutput("CORNER DETECT/alliance zone/BR", FieldConstants.allianceZone.getOurs().withinBounds(brCorner));
		});

		LoggedTunable<double[]> targetAnglesTunable = new LoggedTunable<>() {
			private final LoggedTunable<Angle> targetAngleOffset = LoggedTunable.from("Automations/Bump Mitigation/Target Angle", Degrees::of, 15.0);

			private double[] cache = this.calculateTargetAngles(this.targetAngleOffset.get().in(Radians));

			@Override
			public boolean hasChanged(int id) {
				return this.targetAngleOffset.hasChanged(id);
			}

			@Override
			public double[] get() {
				if (this.targetAngleOffset.hasChanged(this.hashCode())) {
					this.cache = this.calculateTargetAngles(this.targetAngleOffset.get().in(Radians));
				}
				return this.cache;
			}

			private double[] calculateTargetAngles(double offset) {
				return new double[] {
					0 - offset,
					0 + offset,
					Math.PI/2 - offset,
					Math.PI/2 + offset,
					Math.PI - offset,
					Math.PI + offset,
					-Math.PI/2 - offset,
					-Math.PI/2 + offset,
				};
			}
		};

		var bumpLookahead = LoggedTunable.from("Automations/Bump Mitigation/Lookahead Time", Seconds::of, 0.5);
		new Trigger(this.automationsLoop, () -> {
			var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
			var lookaheadTrans = robotPose.getTranslation().plus(new Translation2d(
				this.drive.getFieldMeasuredSpeeds().vxMetersPerSecond * bumpLookahead.get().in(Seconds),
				this.drive.getFieldMeasuredSpeeds().vyMetersPerSecond * bumpLookahead.get().in(Seconds)
			));
			return (FieldConstants.anyBump.getOurs().withinBounds(robotPose.getTranslation()) ||
			FieldConstants.anyBump.getOurs().withinBounds(lookaheadTrans) ||
			FieldConstants.anyBump.getTheirs().withinBounds(robotPose.getTranslation()) ||
			FieldConstants.anyBump.getTheirs().withinBounds(lookaheadTrans))
			&& Math.abs(turnAxis.getAsDouble()) <= 0.0;
		}).whileTrue(this.drive.rotationalSubsystem.pidControlledHeading(() -> {
			var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
			var robotRotation = robotPose.getRotation().getRadians();

			var targetAngles = targetAnglesTunable.get();

			double targetAngleRads = 0;
			double lowestOffset = Math.PI;
			for (double targetAngleCandidate : targetAngles) {
				double candidateOffset = Math.abs(targetAngleCandidate - robotRotation);
				if (candidateOffset < lowestOffset) {
					targetAngleRads = targetAngleCandidate;
					lowestOffset = candidateOffset;
				}
			}

			return new Rotation2d(targetAngleRads);
		}));

		var trenchLookahead = LoggedTunable.from("Automations/Trench Mitigation/Lookahead Time", Seconds::of, 0.5);
		new Trigger(this.automationsLoop, () -> {
			var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
			var lookaheadTrans = robotPose.getTranslation().plus(new Translation2d(
				this.drive.getFieldMeasuredSpeeds().vxMetersPerSecond * trenchLookahead.get().in(Seconds),
				this.drive.getFieldMeasuredSpeeds().vyMetersPerSecond * trenchLookahead.get().in(Seconds)
			));
			return /*(*/(FieldConstants.anyTrench.getOurs().withinBounds(robotPose.getTranslation()) ||
			FieldConstants.anyTrench.getOurs().withinBounds(lookaheadTrans)) //||
			// FieldConstants.anyTrench.getTheirs().withinBounds(robotPose.getTranslation()) ||
			// FieldConstants.anyTrench.getTheirs().withinBounds(lookaheadTrans))
			&& Math.abs(turnAxis.getAsDouble()) <= 0.0
			&& this.drive.getFieldMeasuredSpeeds().vxMetersPerSecond * (AllianceFlipUtil.getAlliance() == Alliance.Blue ? 1 : -1) < 0;
		}).whileTrue(this.drive.rotationalSubsystem.pidControlledHeading(() -> {
			return new Rotation2d(AllianceFlipUtil.getAlliance() == Alliance.Blue ? 0 : Math.PI);
		}).alongWith(this.shooter.hood.idle())
		.alongWith(this.intake.slam.idle())
		);

		// Setup position reset command
		// this.driveController.leftStickButton().and(this.driveController.rightStickButton()).onTrue(Commands.runOnce(() -> RobotState.getInstance().resetPose(
		// 	new Pose2d(
		// 		14.45,
		// 		5,
		// 		Rotation2d.kZero
		// 	)
		// )));

		this.driveController.a().whileTrue(this.intake.intake(this.extension));
	}
}
