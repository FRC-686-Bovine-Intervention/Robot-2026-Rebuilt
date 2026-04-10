package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.util.PIDGains;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class FollowTrajectoryCommand extends Command {
	private final Drive drive;
	private final Trajectory<SwerveSample> trajectory;
	private final Timer trajectoryTimer = new Timer();
	private final boolean endWhenFinished;

	private static final LoggedTunable<Distance> MAX_ERROR = LoggedTunable.from("Drive/Trajectory Following/Max Error", Inches::of, 36.0);
	private static final LoggedTunable<PIDGains> FOLLOWING_TRANS_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Following PIDs/Trans PID", new PIDGains(
		6.0,
		0.0,
		0.0
	));
	private static final LoggedTunable<PIDGains> FOLLOWING_ROT_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Following PIDs/Rot PID", new PIDGains(
		3.5,
		0.0,
		0.0
	));
	private static final LoggedTunable<PIDGains> CATCHUP_TRANS_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Catchup PIDs/Trans PID", new PIDGains(
		8.0,
		0.0,
		0.0
	));
	private static final LoggedTunable<PIDGains> CATCHUP_ROT_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Catchup PIDs/Rot PID", new PIDGains(
		3.5,
		0.0,
		0.0
	));

	private final PIDController followingTranslationalPID = FollowTrajectoryCommand.FOLLOWING_TRANS_PID_GAINS.get().update(new PIDController(0.0, 0.0, 0.0));
	private final PIDController followingRotationalPID = FollowTrajectoryCommand.FOLLOWING_ROT_PID_GAINS.get().update(new PIDController(0.0, 0.0, 0.0));

	private final PIDController catchupTranslationalPID = FollowTrajectoryCommand.CATCHUP_TRANS_PID_GAINS.get().update(new PIDController(0.0, 0.0, 0.0));
	private final PIDController catchupRotationalPID = FollowTrajectoryCommand.CATCHUP_ROT_PID_GAINS.get().update(new PIDController(0.0, 0.0, 0.0));

	public FollowTrajectoryCommand(Drive drive, Trajectory<SwerveSample> trajectory, boolean endWhenFinished) {
		this.drive = drive;
		this.setName("Follow Trajectory");
		this.addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
		this.trajectory = trajectory;
		this.endWhenFinished = endWhenFinished;
	}

	@Override
	public void initialize() {
		this.trajectoryTimer.reset();
		if (FollowTrajectoryCommand.FOLLOWING_TRANS_PID_GAINS.hasChanged(this.hashCode())) {
			FollowTrajectoryCommand.FOLLOWING_TRANS_PID_GAINS.get().update(this.followingTranslationalPID);
		}
		if (FollowTrajectoryCommand.FOLLOWING_ROT_PID_GAINS.hasChanged(this.hashCode())) {
			FollowTrajectoryCommand.FOLLOWING_ROT_PID_GAINS.get().update(this.followingRotationalPID);
		}
		if (FollowTrajectoryCommand.CATCHUP_TRANS_PID_GAINS.hasChanged(this.hashCode())) {
			FollowTrajectoryCommand.CATCHUP_TRANS_PID_GAINS.get().update(this.catchupTranslationalPID);
		}
		if (FollowTrajectoryCommand.CATCHUP_ROT_PID_GAINS.hasChanged(this.hashCode())) {
			FollowTrajectoryCommand.CATCHUP_ROT_PID_GAINS.get().update(this.catchupRotationalPID);
		}
	}

	@Override
	public void execute() {
		var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
		var sample = this.trajectory.sampleAt(this.trajectoryTimer.get(), false).get();
		var errX = sample.x - robotPose.getX();
		var errY = sample.y - robotPose.getY();
		var errTheta = MathUtil.angleModulus(sample.heading - robotPose.getRotation().getRadians());
		var distanceFromSetpointMeters = Math.hypot(errX, errY);

		var errXNorm = errX / distanceFromSetpointMeters;
		var errYNorm = errY / distanceFromSetpointMeters;

		if (distanceFromSetpointMeters > FollowTrajectoryCommand.MAX_ERROR.get().in(Meters)) {
			this.trajectoryTimer.stop();
		} else {
			this.trajectoryTimer.start();
		}

		double transPidOut;
		double rotPidOut;

		double vX;
		double vY;
		double omega;
		// double aX;
		// double aY;
		// double alpha;
		if (this.trajectoryTimer.isRunning()) {
			vX = sample.vx;
			vY = sample.vy;
			omega = sample.omega;
			// aX = sample.ax;
			// aY = sample.ay;
			// alpha = sample.alpha;
			transPidOut = this.followingTranslationalPID.calculate(0.0, distanceFromSetpointMeters);
			rotPidOut = this.followingRotationalPID.calculate(0.0, errTheta);
		} else {
			vX = 0.0;
			vY = 0.0;
			omega = 0.0;
			// aX = 0.0;
			// aY = 0.0;
			// alpha = 0.0;
			transPidOut = this.catchupTranslationalPID.calculate(0.0, distanceFromSetpointMeters);
			rotPidOut = this.catchupRotationalPID.calculate(0.0, errTheta);
		}

		var pidX = transPidOut * errXNorm;
		var pidY = transPidOut * errYNorm;
		var pidOmega = rotPidOut;

		var fieldVX = vX + pidX;
		var fieldVY = vY + pidY;
		var fieldOmega = omega + pidOmega;

		var robotVX = fieldVX * +robotPose.getRotation().getCos() + fieldVY * +robotPose.getRotation().getSin();
		var robotVY = fieldVX * -robotPose.getRotation().getSin() + fieldVY * +robotPose.getRotation().getCos();
		var robotOmega = fieldOmega;

		// var robotAX = aX * +robotPose.getRotation().getCos() + aY * +robotPose.getRotation().getSin();
		// var robotAY = aX * -robotPose.getRotation().getSin() + aY * +robotPose.getRotation().getCos();
		// var robotAlpha = alpha;

		Logger.recordOutput("Trajectory/Setpoint Pose", sample.getPose());
		Logger.recordOutput("Trajectory/Is Iterating Path", this.trajectoryTimer.isRunning());
		// Logger.recordOutput("Trajectory/Setpoint Speeds", DriveConstants.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(sample.getChassisSpeeds(), sample.getPose().getRotation())));
		// Logger.recordOutput("Trajectory/Raw sample", sample);

		this.drive.runRobotSpeeds(
			robotVX,
			robotVY,
			robotOmega
		);
	}

	@Override
	public void end(boolean interrupted) {
		this.trajectoryTimer.stop();
		this.drive.stop();
	}

	@Override
	public boolean isFinished() {
		return this.endWhenFinished && this.trajectoryTimer.get() > this.trajectory.getTotalTime();
	}
}
