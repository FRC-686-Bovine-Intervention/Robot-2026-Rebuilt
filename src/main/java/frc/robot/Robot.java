// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.leds.Leds;
import frc.util.Environment;
import frc.util.LoggedTracer;
import frc.util.Perspective;
import frc.util.VirtualSubsystem;
import frc.util.flipping.AllianceFlipped;
import frc.util.robotStructure.Mechanism3d;

public class Robot extends LoggedRobot {
	private final RobotContainer robotContainer;

	public Robot() {
		Leds.getInstance();

		System.out.println("[Init Robot] Recording AdvantageKit Metadata");
		Logger.recordMetadata("Robot", RobotType.getRobot().name());
		Logger.recordMetadata("Mode", RobotType.getMode().name());
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		Logger.recordMetadata("GitDirty",
			switch (BuildConstants.DIRTY) {
				case 0 -> "All changes committed";
				case 1 -> "Uncomitted changes";
				default -> "Unknown";
			}
		);

		// Set up data receivers & replay source
		System.out.println("[Init Robot] Configuring AdvantageKit for " + RobotType.getMode().name() + " " + RobotType.getRobot().name());
		switch (RobotType.getMode()) {
			// Running on a real robot, log to a USB stick
			case REAL:
				Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
				Logger.addDataReceiver(new NT4Publisher());
			break;

			// Running a physics simulator, log to local folder
			case SIM:
				Logger.addDataReceiver(new WPILOGWriter("logs/sim"));
				Logger.addDataReceiver(new NT4Publisher());
			break;

			// Replaying a log, set up replay source
			case REPLAY:
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			break;
		}

		System.out.println("[Init Robot] Starting AdvantageKit");
		Logger.start();

		System.out.println("[Init Robot] Starting Command Logger");
		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction =
		(Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);
			// Logger.recordOutput(
			//         "Commands/Unique/" + name + "_" + Integer.toHexString(command.hashCode()), active.booleanValue());
			// if(command.getRequirements().size() == 0) {
			//   Logger.recordOutput("Commands/No Requirements/" + name, count > 0);
			// }
			for(Subsystem subsystem : command.getRequirements()) {
				Logger.recordOutput("Commands/" + subsystem.getName(), (count > 0 ? name : "none"));
				// Logger.recordOutput("Commands/" + subsystem.getName() + "/" + name, count > 0);
			}
		};

		CommandScheduler.getInstance()
			.onCommandInitialize(
				(Command command) -> {
					logCommandFunction.accept(command, true);
				}
			)
		;
		CommandScheduler.getInstance()
			.onCommandFinish(
				(Command command) -> {
					logCommandFunction.accept(command, false);
				}
			)
		;
		CommandScheduler.getInstance()
			.onCommandInterrupt(
				(Command command) -> {
					logCommandFunction.accept(command, false);
				}
			)
		;

		System.out.println("[Init Robot] Instantiating RobotContainer");
		this.robotContainer = new RobotContainer();
		System.out.println("[Init Robot] Starting Elastic Layout Webserver");
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath() + "/elastic");

		SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
		Perspective.getCurrent();

		final var activeButtonLoop = new EventLoop();
		activeButtonLoop.bind(() -> {
			LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem");

			Environment.periodic();
			LoggedTracer.logEpoch("CommandScheduler Periodic/Environment Periodic");

			Perspective.periodic();
			LoggedTracer.logEpoch("CommandScheduler Periodic/Perspective Periodic");

			VirtualSubsystem.periodicAll();
			LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic");

			this.robotContainer.apriltagVision.periodic();

			this.robotContainer.drive.structureRoot.setPose(RobotState.getInstance().getEstimatedGlobalPose());
			RobotState.getInstance().log();
			LoggedTracer.logEpoch("CommandScheduler Periodic/RobotState Log");

			this.robotContainer.objectVision.periodic();
			LoggedTracer.logEpoch("CommandScheduler Periodic/ObjectVision Periodic");

			Mechanism3d.logAscopeComponents();
			LoggedTracer.logEpoch("CommandScheduler Periodic/Mechanism3d LogAscopeComponents");

			//Mechanism3d.logAscopeAxes();
			LoggedTracer.logEpoch("CommandScheduler Periodic/Mechanism3d LogAscopeAxes");

			LoggedTracer.logEpoch("CommandScheduler Periodic/Log Intake Gamepieces");

			this.robotContainer.automationsLoop.poll();
			LoggedTracer.logEpoch("CommandScheduler Periodic/Automations");

			CommandScheduler.getInstance().getDefaultButtonLoop().poll();
			LoggedTracer.logEpoch("CommandScheduler Periodic/Triggers");
		});
		CommandScheduler.getInstance().setActiveButtonLoop(activeButtonLoop);

		var hubCenter = AllianceFlipped.fromBlue(new Pose3d(new Translation3d(FieldConstants.hubCenter.getBlue().getX(), FieldConstants.hubCenter.getBlue().getY(), FieldConstants.hubHeight.in(Meters)), Rotation3d.kZero));
		var outpostCenter = AllianceFlipped.fromBlue(new Pose3d(new Translation3d(0.0, FieldConstants.outpostCenterY.getBlue().in(Meters), 0.0), Rotation3d.kZero));
		var towerRungsCenter = AllianceFlipped.fromBlue(new Pose3d(new Translation3d(FieldConstants.towerRungsCenter.getBlue().getX(), FieldConstants.towerRungsCenter.getBlue().getY(), 0.0), Rotation3d.kZero));
		var bottomUpright = new Pose3d(new Translation3d(FieldConstants.towerRungsCenter.getBlue().getX(), FieldConstants.towerRungsCenter.getBlue().getY() - FieldConstants.towerInnerUprightDistance.div(2).in(Meters), 0.01), new Rotation3d(Rotation2d.kCCW_90deg));
		var topUpright = new Pose3d(new Translation3d(FieldConstants.towerRungsCenter.getBlue().getX(), FieldConstants.towerRungsCenter.getBlue().getY() + FieldConstants.towerInnerUprightDistance.div(2).in(Meters), 0.01), new Rotation3d(Rotation2d.k180deg));
		var backBottomUpright = new Pose3d(new Translation3d(FieldConstants.towerRungsCenter.getBlue().getX() - Inches.of(27.639411).in(Meters), FieldConstants.towerRungsCenter.getBlue().getY() - FieldConstants.towerInnerUprightDistance.div(2).in(Meters), 0.01), new Rotation3d(Rotation2d.kZero));
		var backTopUpright = new Pose3d(new Translation3d(FieldConstants.towerRungsCenter.getBlue().getX() - Inches.of(27.639411).in(Meters), FieldConstants.towerRungsCenter.getBlue().getY() + FieldConstants.towerInnerUprightDistance.div(2).in(Meters), 0.01), new Rotation3d(Rotation2d.kCW_90deg));

		Logger.recordOutput("DEBUG/hubCenter/blue", hubCenter.getBlue());
		Logger.recordOutput("DEBUG/hubCenter/red", hubCenter.getRed());
		Logger.recordOutput("DEBUG/outpostCenter/blue", outpostCenter.getBlue());
		Logger.recordOutput("DEBUG/outpostCenter/red", outpostCenter.getRed());
		Logger.recordOutput("DEBUG/towerRungsCenter/blue", towerRungsCenter.getBlue());
		Logger.recordOutput("DEBUG/towerRungsCenter/red", towerRungsCenter.getRed());
		Logger.recordOutput("DEBUG/bottomUpright", bottomUpright);
		Logger.recordOutput("DEBUG/topUpright", topUpright);
		Logger.recordOutput("DEBUG/backBottomUpright", backBottomUpright);
		Logger.recordOutput("DEBUG/backTopUpright", backTopUpright);
	}

	@Override
	public void robotPeriodic() {
		LoggedTracer.reset();

		CommandScheduler.getInstance().run();
		LoggedTracer.logEpoch("CommandScheduler Periodic/Commands");
		LoggedTracer.logEpoch("CommandScheduler Periodic");

		VirtualSubsystem.postCommandPeriodicAll();
		LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic");
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		this.robotContainer.autoManager.endAuto();
		this.robotContainer.autoManager.startAuto();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		this.robotContainer.autoManager.endAuto();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
