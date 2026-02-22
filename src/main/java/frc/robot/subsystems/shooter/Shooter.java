package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheel leftFlywheel;
	public final Flywheel rightFlywheel;
	public final Hood hood;
	public final AimingSystem aimingSystem;
	public Command aimLeftFlywheelAtHub() {
		return this.leftFlywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimRightFlywheelAtHub() {
		return this.rightFlywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimHoodAtHub() {
		return this.hood.genAngleCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetHoodAngleRads);
	}
	public Command aimDriveAtHub(Drive.Rotational rotational) {
		return rotational.pidControlledHeading(() -> Rotation2d.fromRadians(this.aimingSystem.shootingCalc.getTargetAzimuthHeadingRads()));
	}

	public Command aimLeftFlywheelToPass() {
		return this.leftFlywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimRightFlywheelToPass() {
		return this.rightFlywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimHoodToPass() {
		return this.hood.genAngleCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetHoodAngleRads);
	}
	public Command aimDriveToPass(Drive.Rotational rotational) {
		return rotational.pidControlledHeading(() -> Rotation2d.fromRadians(this.aimingSystem.passingCalc.getTargetAzimuthHeadingRads()));
	}
}
