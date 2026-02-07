package frc.robot.auto.routines;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoRoutine;
import frc.robot.auto.FreeRangeAutoSelectorIO;
import frc.robot.auto.FreeRangeAutoSelectorIOInputsAutoLogged;
import frc.robot.subsystems.vision.object.ObjectVision;

public class FreeRangeAuto extends AutoRoutine {
	private final FreeRangeAutoSelectorIO io;
	private final FreeRangeAutoSelectorIOInputsAutoLogged inputs = new FreeRangeAutoSelectorIOInputsAutoLogged();

	private final Drive drive;
	private final Intake intake;
	private final Shooter shooter;
	private final Climb climb;
	private final Rollers rollers;
	private final ObjectVision vision;

	public FreeRangeAuto(RobotContainer robot) {
		super("Free Range", List.of());
		this.io = robot.freeRangeAutoSelectorIO;
		this.drive = robot.drive;
		this.intake = robot.intake;
		this.shooter = robot.shooter;
		this.climb = robot.climb;
		this.rollers = robot.rollers;
	}
	
	@Override
	public Command generateCommand() {
		return new Command() {
			double[] topLeft = new double[2];
			double[] bottomRight = new double[2];
			boolean shouldKeepOut;
			boolean shouldClimb;
			boolean shouldNeutralZone;
			boolean shouldAvoidBump;
			boolean shouldNotAutoIntake;
			boolean shouldStopToShoot;

			boolean hasFullHopper = false;
			{
				setName("Free Range Auto");
				addRequirements(drive, intake, shooter, climb, rollers);
			}

			@Override
			public void initialize() {
				io.updateInputs(inputs);
				topLeft = inputs.keepOutTopLeft;
				bottomRight = inputs.keepOutBottomRight;
				shouldKeepOut = inputs.shouldKeepOut;
				shouldClimb = inputs.shouldClimb;
				shouldNeutralZone = inputs.shouldNeutralZone;
				shouldAvoidBump = inputs.shouldAvoidBump;
				shouldNotAutoIntake = inputs.shouldNotAutoIntake;
				shouldStopToShoot = inputs.shouldStopToShoot;
			}

			@Override
			public void execute() {
				if (!shouldNotAutoIntake && !hasFullHopper) {
					autoIntake(topLeft, bottomRight).schedule();
				} else if (hasFullHopper && shouldStopToShoot) {
					// Should shoot if it has a full hopper
				}
			}

			@Override
			public boolean isFinished() {
			
			}
		}.asProxy();
	}

	private Command autoIntake(double[] topLeft, double[] bottomRight) {
		return new Command() {
			{
				setName("Auto Intake");
				addRequirements(intake);
			}

			@Override
			public void initialize() {
				intake.deploy();
			}

			@Override
			public void execute() {
				intake.intake();
			}

			@Override
			public boolean isFinished() {
				return false;
			}
		};
	}
}
