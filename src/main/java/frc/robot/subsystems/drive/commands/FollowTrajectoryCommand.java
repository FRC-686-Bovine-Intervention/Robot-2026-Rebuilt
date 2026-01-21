package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.PIDGains;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class FollowTrajectoryCommand extends Command {
	private final Drive drive;
	private final Trajectory<SwerveSample> trajectory;
	private final Timer trajectoryTimer = new Timer();
	private final boolean endWhenFinished;

	private static final LoggedTunable<Distance> MAX_ERROR = LoggedTunable.from("Drive/Trajectory Following/Max Error", Inches::of, 2400.0);
	private static final LoggedTunable<PIDGains> TRANS_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Trans PID", new PIDGains(
		1.0,
		0.0,
		0.0
	));
	private static final LoggedTunable<PIDGains> ROT_PID_GAINS = LoggedTunable.from("Drive/Trajectory Following/Rot PID", new PIDGains(
		1.0,
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

		sample.ax;

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

		double ffX;
		double ffY;
		double ffOmega;
		if (this.trajectoryTimer.isRunning()) {
			ffX = sample.vx;
			ffY = sample.vy;
			ffOmega = sample.omega;
		} else {
			ffX = 0.0;
			ffY = 0.0;
			ffOmega = 0.0;
		}

		var fieldX = ffX + pidX;
		var fieldY = ffY + pidY;
		var fieldOmega = ffOmega + pidOmega;

		var robotX = fieldX * +robotPose.getRotation().getCos() - fieldY * -robotPose.getRotation().getSin();
		var robotY = fieldX * -robotPose.getRotation().getSin() + fieldY * +robotPose.getRotation().getCos();
		var robotOmega = fieldOmega;

		Logger.recordOutput("Trajectory/Setpoint Pose", sample.getPose());
		Logger.recordOutput("Trajectory/Setpoint Speeds", DriveConstants.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(sample.getChassisSpeeds(), sample.getPose().getRotation())));

		this.drive.runRobotSpeeds(robotX, robotY, robotOmega);
	}

	@Override
	public void end(boolean interrupted) {
		this.trajectoryTimer.stop();
	}

	@Override
	public boolean isFinished() {
		return this.endWhenFinished && this.trajectoryTimer.get() > this.trajectory.getTotalTime();
	}
}
