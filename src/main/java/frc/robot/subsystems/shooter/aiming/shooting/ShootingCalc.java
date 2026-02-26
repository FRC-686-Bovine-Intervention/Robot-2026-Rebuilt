package frc.robot.subsystems.shooter.aiming.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ShootingCalc {
	public void calculate(Translation2d robotPos, ChassisSpeeds fieldSpeeds, Translation3d aimPoint);

	public double getTargetFlywheelSurfaceVeloMPS();
	public double getTargetHoodAngleRads();
	public double getTargetAzimuthHeadingRads();

	public Translation3d getAimPoint();

	public double getTOFSeconds();
}
