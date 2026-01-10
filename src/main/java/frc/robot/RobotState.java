package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class RobotState {
	private static RobotState instance;
	public static RobotState getInstance() {if (instance == null) {instance = new RobotState();} return instance;}

	private Pose2d odometryPose = Pose2d.kZero;
	private Rotation3d gyroOffset = Rotation3d.kZero;
	private static final Matrix<N3, N1> odometryStateStdDevs = VecBuilder.fill(0.003, 0.003, 0.002);
	private final Matrix<N3, N1> qStdDevs;

	private static final double poseBufferSizeSecs = 2.0;
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSecs);
	private Pose2d estimatedGlobalPose = Pose2d.kZero;

	private final SimpleMatrix forwardKinematics;

	private static final LoggedTunable<Time> txtyStaleTime = LoggedTunable.from("RobotState/TxTy Stale Time", Seconds::of, 0.2);
	private final Map<Integer, TxTyObservation> txtyObservations = new HashMap<>(FieldConstants.apriltagLayout.getTags().size());

	private RobotState() {
		this.qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
		for (int i = 0; i < 3; i++) {
			this.qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
		}
		var inverseKinematics = new SimpleMatrix(DriveConstants.moduleTranslations.length * 2, 3);
		for (int i = 0; i < DriveConstants.moduleTranslations.length; i++) {
			inverseKinematics.setRow(i * 2 + 0, 0, 1, 0, -DriveConstants.moduleTranslations[i].getY());
			inverseKinematics.setRow(i * 2 + 1, 0, 0, 1, +DriveConstants.moduleTranslations[i].getX());
		}
		this.forwardKinematics = inverseKinematics.pseudoInverse();
	}

	public void log() {
		Logger.recordOutput("RobotState/OdometryPose", this.odometryPose);
		Logger.recordOutput("RobotState/EstimatedGlobalPose", this.getEstimatedGlobalPose());

		for (var observation : this.txtyObservations.values()) {
			Logger.recordOutput("RobotState/TxTyObservations/Timestamps/Tag " + Integer.toString(observation.tagID()), observation.timestamp());
			Logger.recordOutput("RobotState/TxTyObservations/Poses/Tag " + Integer.toString(observation.tagID()), observation.pose());
		}
	}

	public Pose2d getEstimatedGlobalPose() {
		return this.estimatedGlobalPose;
	}

	public void resetPose(Pose2d pose) {
		var gyroOffsetYaw = this.gyroOffset.toRotation2d();
		var gyroOffsetNoYaw = this.gyroOffset.minus(new Rotation3d(gyroOffsetYaw));

		this.gyroOffset = new Rotation3d(pose.getRotation().minus(this.odometryPose.getRotation().minus(gyroOffsetYaw))).plus(gyroOffsetNoYaw);
		this.estimatedGlobalPose = pose;
		this.odometryPose = pose;
		this.poseBuffer.clear();
	}

	public void addOdometryObservation(OdometryObservation observation) {
		var moduleDeltasVector = new SimpleMatrix(DriveConstants.moduleTranslations.length * 2, 1);
		for (int i = 0; i < DriveConstants.moduleTranslations.length; i++) {
			var dx = observation.endModulePositions()[i].distanceMeters - observation.startModulePositions()[i].distanceMeters;
			var dTheta = observation.endModulePositions()[i].angle.minus(observation.startModulePositions()[i].angle);

			double s;
			double c;
			if (Math.abs(dTheta.getRadians()) < 1E-9) {
				s = 1.0 - 1.0 / 6.0 * dTheta.getRadians() * dTheta.getRadians();
				c = 0.5 * dTheta.getRadians();
			} else {
				s = dTheta.getSin() / dTheta.getRadians();
				c = (1 - dTheta.getCos()) / dTheta.getRadians();
			}

			var x = dx * s;
			var y = dx * c;

			var moduleDisplacementX = x * observation.startModulePositions()[i].angle.getCos() - y * observation.startModulePositions()[i].angle.getSin();
			var moduleDisplacementY = x * observation.startModulePositions()[i].angle.getSin() + y * observation.startModulePositions()[i].angle.getCos();

			moduleDeltasVector.set(i * 2 + 0, 0, moduleDisplacementX);
			moduleDeltasVector.set(i * 2 + 1, 0, moduleDisplacementY);
		}

		var chassisDeltaVector = this.forwardKinematics.mult(moduleDeltasVector);
		var twist = new Twist2d(
			chassisDeltaVector.get(0, 0),
			chassisDeltaVector.get(1, 0),
			chassisDeltaVector.get(2, 0)
		);

		var lastOdometryPose = this.odometryPose;
		this.odometryPose = this.odometryPose.exp(twist);

		if (observation.gyroRotation.isPresent()) {
			this.odometryPose = new Pose2d(this.odometryPose.getTranslation(), observation.gyroRotation.get().plus(this.gyroOffset).toRotation2d());
		}

		this.poseBuffer.addSample(observation.timestamp(), this.odometryPose);

		var finalTwist = lastOdometryPose.log(this.odometryPose);
		this.estimatedGlobalPose = this.estimatedGlobalPose.exp(finalTwist);
	}

	public void addVisionObservation(VisionObservation observation) {
		try {
			if (this.poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSecs > observation.timestamp()) {
				return;
			}
		} catch (NoSuchElementException e) {
			return;
		}
		var sample = this.poseBuffer.getSample(observation.timestamp());
		if (sample.isEmpty()) {return;}

		var sampleToOdometryTransform = new Transform2d(sample.get(), this.odometryPose);
		var odometryToSampleTransform = new Transform2d(this.odometryPose, sample.get());

		var globalEstimateAtTime = this.estimatedGlobalPose.plus(odometryToSampleTransform);

		var r = new double[3];
		for (int i = 0; i < 3; i++) {
			r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
		}
		var visionK = new Matrix<>(Nat.N3(), Nat.N3());
		for (int row = 0; row < 3; row++) {
			double stdDev = this.qStdDevs.get(row, 0);
			if (stdDev == 0.0) {
				visionK.set(row, row, 0.0);
			} else {
				visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
			}
		}

		var globalEstimateAtTimeToObservation = new Transform2d(globalEstimateAtTime, observation.pose());
		var kTimesTransform = visionK.times(
			VecBuilder.fill(
				globalEstimateAtTimeToObservation.getX(),
				globalEstimateAtTimeToObservation.getY(),
				globalEstimateAtTimeToObservation.getRotation().getRadians()
			)
		);
		var scaledTransform = new Transform2d(
			kTimesTransform.get(0, 0),
			kTimesTransform.get(1, 0),
			Rotation2d.fromRadians(kTimesTransform.get(2, 0))
		);

		this.estimatedGlobalPose = globalEstimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
	}

	public void addTxTyObservation(TxTyObservation observation) {
		var tagID = observation.tagID();
		var previousObservation = this.txtyObservations.get(tagID);
		if (previousObservation == null || observation.timestamp() > previousObservation.timestamp()) {
			this.txtyObservations.put(tagID, observation);
		}
	}

	public Optional<Pose2d> getRobotPoseFromTag(int tagID) {
		var observation = this.txtyObservations.get(tagID);

		if (observation == null) {
			return Optional.empty();
		}

		if (Timer.getTimestamp() - observation.timestamp() >= txtyStaleTime.get().in(Seconds)) {
			return Optional.empty();
		}

		var odometrySample = this.poseBuffer.getSample(observation.timestamp());

		if (odometrySample.isEmpty()) {
			return Optional.empty();
		}

		return Optional.of(observation.pose().plus(new Transform2d(odometrySample.get(), this.odometryPose)));
	}

	public static record OdometryObservation(
		double timestamp,
		Optional<Rotation3d> gyroRotation,
		SwerveModulePosition[] startModulePositions,
		SwerveModulePosition[] endModulePositions
	) {}
	public static record VisionObservation(
		double timestamp,
		Pose2d pose,
		Matrix<N3, N1> stdDevs
	) {}
	public static record TxTyObservation(
		double timestamp,
		int tagID,
		Pose2d pose
	) {}
}
