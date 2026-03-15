package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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

	private static final LoggedTunable<Distance> MAX_ERROR = LoggedTunable.from("Drive/Trajectory Following/Max Error", Inches::of, 24.0);
	private static final LoggedTunable<PIDGains> TRANS_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Trans PID", new PIDGains(
		2.0,
		0.0,
		0.0
	));
	private static final LoggedTunable<PIDGains> ROT_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Rot PID", new PIDGains(
		3.5,
		0.0,
		0.0
	));

	private final PIDController translationalPID = TRANS_PID_GAINS.get().update(new PIDController(0.0, 0.0, 0.0));
	private final PIDController rotationalPID = ROT_PID_GAINS.get().update(new PIDController(0.0, 0.0, 0.0));

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
		if (TRANS_PID_GAINS.hasChanged(this.hashCode())) {
			TRANS_PID_GAINS.get().update(this.translationalPID);
		}
		if (ROT_PID_GAINS.hasChanged(this.hashCode())) {
			ROT_PID_GAINS.get().update(this.rotationalPID);
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

		if (distanceFromSetpointMeters > MAX_ERROR.get().in(Meters)) {
			this.trajectoryTimer.stop();
		} else {
			this.trajectoryTimer.start();
		}

		var transPidOut = this.translationalPID.calculate(0.0, distanceFromSetpointMeters);
		var rotPidOut = this.rotationalPID.calculate(0.0, errTheta);

		var pidX = transPidOut * errXNorm;
		var pidY = transPidOut * errYNorm;
		var pidOmega = rotPidOut;

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
		} else {
			vX = 0.0;
			vY = 0.0;
			omega = 0.0;
			// aX = 0.0;
			// aY = 0.0;
			// alpha = 0.0;
		}

		var fieldVX = vX + pidX;
		var fieldVY = vY + pidY;
		var fieldOmega = omega + pidOmega;

		var robotVX = fieldVX * +robotPose.getRotation().getCos() + fieldVY * +robotPose.getRotation().getSin();
		var robotVY = fieldVX * -robotPose.getRotation().getSin() + fieldVY * +robotPose.getRotation().getCos();
		var robotOmega = fieldOmega;

		// var robotAX = aX * +robotPose.getRotation().getCos() + aY * +robotPose.getRotation().getSin();
		// var robotAY = aX * -robotPose.getRotation().getSin() + aY * +robotPose.getRotation().getCos();
		// var robotAlpha = alpha;

		// Logger.recordOutput("Trajectory/Setpoint Pose", sample.getPose());
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
