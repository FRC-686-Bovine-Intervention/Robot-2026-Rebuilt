package frc.robot.subsystems.shooter.aiming;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.loggerUtil.tunables.LoggedTunable;
import lombok.Getter;

public class InterpolationAiming extends Aiming {
    private Translation3d aimPoint;
    // private Pose2d shotPose;
    // private ChassisSpeeds shotSpeeds;
    private double effectiveDistanceMeters;
    private double targetHoodAngleRads;
    private double targetFlywheelVeloMPS;
    @Getter
    private double targetDriveHeadingRads;
    private static final LoggedTunable<Time> lookaheadTime = LoggedTunable.from("Shooter/Aiming/Lookahead Seconds", Seconds::of, 0.035);
    private static final LoggedTunable<Distance> azimuthTolerance = LoggedTunable.from("Shooter/Aiming/Tolerance/Azimuth", Centimeters::of, 100);
    private static final LoggedTunable<Distance> altitudeDegsTolerance = LoggedTunable.from("Shooter/Aiming/Tolerance/Altitude", Centimeters::of, 46);
    private static final LoggedTunable<Angle> customAzimuthOffset = LoggedTunable.from("Shooter/Aiming/Custom Azimuth Offset", Radians::of, 0.0);
    
public Command aim(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds, Supplier<Translation3d> target) {
    



        final var aiming = this;
        return new Command() {
            {
                this.setName("Aim");
                this.addRequirements(aiming);
            }

            @Override
            public void initialize() {
                aiming.calculate(robotPose.get().getTranslation(), fieldSpeeds.get(), target.get());
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
 private void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeeds, Translation3d goal) {
        // this.calculate(
        //     robotPos,
        //     fieldRelativeSpeeds,
        //     goal.centerPoint,
        //     goal.type.select(ShooterConstants.highGoalTargetPivotAltitudeRads, ShooterConstants.lowGoalTargetPivotAltitudeRads),
        //     goal.type.select(ShooterConstants.highGoalTargetFlywheelVeloMPS, ShooterConstants.lowGoalTargetFlywheelVeloMPS),
        //     goal.type.select(ShooterConstants.highGoalTargetDrivetrainOffsetRads, ShooterConstants.lowGoalTargetDrivetrainOffsetRads)
        // );
 }

 private void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeeds, Translation3d aimPoint, InterpolatingDoubleTreeMap pivotAltitudeMap, InterpolatingDoubleTreeMap flywheelVeloMap) {
        this.aimPoint = aimPoint;

        var predictedX = robotPos.getX() + fieldRelativeSpeeds.vxMetersPerSecond * lookaheadTime.get().in(Seconds);
        var predictedY = robotPos.getY() + fieldRelativeSpeeds.vyMetersPerSecond * lookaheadTime.get().in(Seconds);

        var predictedToTargetX = this.aimPoint.getX() - predictedX;
        var predictedToTargetY = this.aimPoint.getY() - predictedY;

        this.targetDriveHeadingRads = Math.atan2(predictedToTargetY, predictedToTargetX);
        this.effectiveDistanceMeters = Math.hypot(predictedToTargetX, predictedToTargetY);
        Logger.recordOutput("Shooter/Aiming/Effective Distance", this.effectiveDistanceMeters);

        this.targetHoodAngleRads = pivotAltitudeMap.get(this.effectiveDistanceMeters);
        this.targetFlywheelVeloMPS = flywheelVeloMap.get(this.effectiveDistanceMeters);

        Logger.recordOutput("Shooter/Aiming/Aim Point", this.aimPoint);
        Logger.recordOutput("Shooter/Aiming/Shot Pose", new Pose2d(
            new Translation2d(
                predictedX,
                predictedY
            ),
            Rotation2d.fromRadians(
                this.targetDriveHeadingRads
            )
        ));
    }


}
