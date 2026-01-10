package frc.util.geometry;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RollingAveragePose2d {
	private final int windowSize;
	private final Queue<Pose2d> poses;

	private double sumX;
	private double sumY;
	private double sumCos;
	private double sumSin;

	public RollingAveragePose2d(int windowSize) {
		this.windowSize = windowSize;
		this.poses = new LinkedList<>();
	}

	public void addPose(Pose2d pose) {
		poses.add(pose);
		sumX += pose.getX();
		sumY += pose.getY();
		sumCos += pose.getRotation().getCos();
		sumSin += pose.getRotation().getSin();

		if (poses.size() > windowSize) {
			Pose2d removed = poses.poll();
			sumX -= removed.getX();
			sumY -= removed.getY();
			sumCos -= removed.getRotation().getCos();
			sumSin -= removed.getRotation().getSin();
		}
	}

	public Pose2d getAveragePose() {
		if (poses.isEmpty()) {
			return new Pose2d();
		}
		int size = poses.size();

		double avgX = sumX / size;
		double avgY = sumY / size;
		double avgCos = sumCos / size;
		double avgSin = sumSin / size;
		double avgTheta = Math.atan2(avgSin, avgCos);

		return new Pose2d(avgX, avgY, new Rotation2d(avgTheta));
	}

	public void reset() {
		poses.clear();
		sumX = sumY = 0.0;
		sumCos = sumSin = 0.0;
	}
}
