package frc.util.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.controllers.Joystick.Axis;

public class XboxController {
	public final edu.wpi.first.wpilibj.XboxController hid;
	public final Joystick leftStick;
	public final Joystick rightStick;

	public final Axis leftTrigger;
	public final Axis rightTrigger;

	public XboxController(int port) {
		hid = new edu.wpi.first.wpilibj.XboxController(port);

		leftStick = new Joystick(hid::getLeftX, hid::getLeftY).invertY();
		rightStick = new Joystick(hid::getRightX, hid::getRightY).invertY();

		leftTrigger = new Axis(hid::getLeftTriggerAxis);
		rightTrigger = new Axis(hid::getRightTriggerAxis);
	}

	public Trigger a() {return hid.a(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger b() {return hid.b(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger x() {return hid.x(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger y() {return hid.y(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger leftBumper() {return hid.leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger rightBumper() {return hid.rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger start() {return hid.start(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger back() {return hid.back(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger leftStickButton() {return hid.leftStick(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger rightStickButton() {return hid.rightStick(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povCenter() {return hid.povCenter(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povUp() {return hid.povUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povUpRight() {return hid.povUpRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povRight() {return hid.povRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povDownRight() {return hid.povDownRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povDown() {return hid.povDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povDownLeft() {return hid.povDownLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povLeft() {return hid.povLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povUpLeft() {return hid.povUpLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}

	public boolean isConnected() {
		return hid.isConnected();
	}

	public void setRumble(RumbleType rumbleType, double value) {
		hid.setRumble(rumbleType, value);
	}
	public Command rumble(RumbleType rumbleType, double value) {
		return Commands.startEnd(
			() -> setRumble(rumbleType, value),
			() -> setRumble(rumbleType, 0)
		);
	}
}
