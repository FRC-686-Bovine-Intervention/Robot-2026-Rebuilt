package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.math.MathExtraUtil;

public class PhysicsAiming extends Aiming{
    private double hoodAngleRads;
    private double flywheelSpeedMS;
    private double robotRotationRads;

    public double getHoodAngleRads() {
        return this.hoodAngleRads;
    }

    public double getFlywheelSpeedMS() {
        return this.flywheelSpeedMS;
    }

    public double getRobotRotationRads() {
        return this.robotRotationRads;
    }

    @Override
    protected void calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation3d aimPoint) {
        var robotHubSpaceCartesian = robotPose.relativeTo(new Pose2d(aimPoint.toTranslation2d(), Rotation2d.kZero));
        double radius = Math.sqrt(Math.pow(robotHubSpaceCartesian.getX(), 2) + Math.pow(robotHubSpaceCartesian.getY(), 2));
        
        var radialUnitVectorCartesian = new double[] {-robotHubSpaceCartesian.getX() / radius, -robotHubSpaceCartesian.getY() / radius};
        var tangentialUnitVectorCartesian = new double[] {-radialUnitVectorCartesian[1], radialUnitVectorCartesian[0]};
        var robotSpeedsArray = new double[] { robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond };

        double radialVelocity = MathExtraUtil.dotProduct(radialUnitVectorCartesian, robotSpeedsArray);
        double tangentialVelocity = MathExtraUtil.dotProduct(tangentialUnitVectorCartesian, robotSpeedsArray);

        var vals = ShooterConstants.aimingPolynomial.evaluate(new double[] {radius, radialVelocity});
        double angleOffsetRads = Math.atan2(tangentialVelocity, -Math.abs(radialVelocity)); //NEED A DOUBLE-CHECK ON THAT

        double robotAngleRads = Math.atan2(robotHubSpaceCartesian.getY(), robotHubSpaceCartesian.getX());

        this.robotRotationRads = angleOffsetRads + robotAngleRads;
        this.hoodAngleRads = vals[0];
        this.flywheelSpeedMS = vals[1];
    }
}