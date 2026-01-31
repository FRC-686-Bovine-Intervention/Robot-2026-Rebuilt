package frc.robot.subsystems.shooter.aiming.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class InterpolationShootingCalc implements ShootingCalc {

    @Override
    public void calculate(Translation2d robotPos, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'calculate'");
    }

    @Override
    public double getTargetFlywheelSurfaceVeloMPS() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetFlywheelSurfaceVeloMPS'");
    }

    @Override
    public double getTargetHoodAngleRads() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetHoodAngleRads'");
    }

    @Override
    public double getTargetAzimuthHeadingRads() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetAzimuthHeadingRads'");
    }

    @Override
    public Translation3d getAimPoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAimPoint'");
    }
    
}
