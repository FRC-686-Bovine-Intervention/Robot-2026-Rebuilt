package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
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
		return rotational.genHeadingPIDCommand(
			"Aim at Hub",
			LoggedTunable.from("Shooting/Aiming/Rotational PID", new PIDConstants(5.0, 0.0, 0.0)),
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.shootingCalc::getTargetAzimuthHeadingRads
		);
	}

	public Command aimLeftFlywheelToPass() {
		return this.leftFlywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimRightFlywheelToPass() {
		return this.rightFlywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimHoodToPass() {
		return this.hood.genAngleCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetHoodAngleRads);
	}
	public Command aimDriveToPass(Drive.Rotational rotational) {
		return rotational.genHeadingPIDCommand(
			"Aim to Pass",
			LoggedTunable.from("Shooting/Passing/Rotational PID", new PIDConstants(0.0, 0.0, 0.0)),
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.passingCalc::getTargetAzimuthHeadingRads
		);
	}
}
