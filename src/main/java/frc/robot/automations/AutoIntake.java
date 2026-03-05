package frc.robot.automations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.EdgeDetector;
import frc.util.misc.Cluster;

public class AutoIntake implements Runnable{
	private final Drive drive;
	private final Intake intake;
	private final ObjectVision objectVision;
	private final Trigger trigger;
	private final Supplier<ChassisSpeeds> desiredSpeedsSupplier;
	private final Command command;

	private Cluster chosenCluster;
	private Translation2d start;
	private boolean hasReachedStart = false;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public AutoIntake(Drive drive, Intake intake, ObjectVision objectVision, Trigger trigger, Supplier<ChassisSpeeds> desiredSpeedsSupplier) {
		this.drive = drive;
		this.intake = intake;
		this.objectVision = objectVision;
		this.trigger = trigger;
		this.desiredSpeedsSupplier = desiredSpeedsSupplier;
		this.command = new Command() {
			{
				this.addRequirements(drive.translationSubsystem);
				this.setName("AutoIntake");
			}

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				double[] speeds = getDriveSpeeds();
				drive.translationSubsystem.driveVelocity(speeds[0], speeds[1]);
			}

			@Override
			public void end(boolean interrupted) {
				hasReachedStart = false;
				start = null;
				chosenCluster = null;
			}
		}.alongWith(drive.rotationalSubsystem.pidControlledHeading(() -> new Rotation2d(getDriveSpeeds()[2]))).repeatedly();
	}
	@Override
	public void run() {
		edgeDetector.update(
			// this.intake.rollers.getCurrentCommand().getName() != "Idle"
			// && this.drive.translationSubsystem.getCurrentCommand() == null
			// && this.drive.rotationalSubsystem.getCurrentCommand() == null
			trigger.getAsBoolean()
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			startAutoIntake();
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}

	private void startAutoIntake() {
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

	private double[] getDriveSpeeds() {
		if (start != null) {
			var pose = RobotState.getInstance().getEstimatedGlobalPose();
			if (!hasReachedStart) {
				var startPos = start.minus(pose.getTranslation());
				var angle = Math.atan2(startPos.getY(), startPos.getX());
				var startRobotRelative = new Pose2d(startPos, Rotation2d.kZero).relativeTo(pose);
				var velocity = desiredSpeedsSupplier.get();
				var vx = velocity.vxMetersPerSecond;
				var lambda = startRobotRelative.getY()/startRobotRelative.getX();
				Logger.recordOutput("DEBUG/AutoIntake/DesiredSpeeds", new Translation2d(vx, vx * lambda));
				return new double[] {vx, vx * lambda, angle};
			} else {
				var startPos = start.minus(pose.getTranslation());
				var angle = Math.atan2(startPos.getY(), startPos.getX());
				var startRobotRelative = new Pose2d(startPos, Rotation2d.kZero).relativeTo(pose);
				var velocity = desiredSpeedsSupplier.get();
				var vx = velocity.vxMetersPerSecond;
				var lambda = startRobotRelative.getY()/startRobotRelative.getX();
				return new double[] {vx, vx * lambda, angle};
			}
		}
		return new double[] {0, 0, 0};
	}
}
