package frc.util.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.controllers.Joystick.Axis;

public class XboxController {
	public final edu.wpi.first.wpilibj.XboxController hid;
	public final Joystick leftStick;
	public final Joystick rightStick;

	public final Axis leftTrigger;
	public final Axis rightTrigger;

	public final RumbleSystem leftRumble;
	public final RumbleSystem rightRumble;

	public XboxController(int port, String name) {
		this.hid = new edu.wpi.first.wpilibj.XboxController(port);

		this.leftStick = new Joystick(this.hid::getLeftX, this.hid::getLeftY).invertY();
		this.rightStick = new Joystick(this.hid::getRightX, this.hid::getRightY).invertY();

		this.leftTrigger = new Axis(this.hid::getLeftTriggerAxis);
		this.rightTrigger = new Axis(this.hid::getRightTriggerAxis);

		this.leftRumble = new RumbleSystem(name + "/Rumble/Left") {
			@Override
			public void setRumble(double rumble) {
				hid.setRumble(RumbleType.kLeftRumble, rumble);
			}
		};
		this.rightRumble = new RumbleSystem(name + "/Rumble/Right") {
			@Override
			public void setRumble(double rumble) {
				hid.setRumble(RumbleType.kRightRumble, rumble);
			}
		};
	}

	public Trigger a()                {return this.hid.a(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger b()                {return this.hid.b(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger x()                {return this.hid.x(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger y()                {return this.hid.y(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger leftBumper()       {return this.hid.leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger rightBumper()      {return this.hid.rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger start()            {return this.hid.start(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger back()             {return this.hid.back(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger leftStickButton()  {return this.hid.leftStick(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger rightStickButton() {return this.hid.rightStick(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povCenter()        {return this.hid.povCenter(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povUp()            {return this.hid.povUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povUpRight()       {return this.hid.povUpRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povRight()         {return this.hid.povRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povDownRight()     {return this.hid.povDownRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povDown()          {return this.hid.povDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povDownLeft()      {return this.hid.povDownLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povLeft()          {return this.hid.povLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}
	public Trigger povUpLeft()        {return this.hid.povUpLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);}

	public boolean isConnected() {
		return this.hid.isConnected();
	}

	public static abstract class RumbleSystem extends SubsystemBase {
		public RumbleSystem(String name) {
			super(name);
		}

		public abstract void setRumble(double rumble);
	}
}
