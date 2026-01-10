package frc.util.controllers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard3x3 {
	private final edu.wpi.first.wpilibj.Joystick hid;
	public final Joystick joystick;

	public ButtonBoard3x3(int port) {
		hid = new edu.wpi.first.wpilibj.Joystick(port);

		joystick = new Joystick(hid::getX, hid::getY);
	}
	/**---<p>---<p>x--*/
	public Trigger x1y1() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(7));}
	/**---<p>---<p>-x-*/
	public Trigger x2y1() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(8));}
	/**---<p>---<p>--x*/
	public Trigger x3y1() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(9));}
	/**---<p>x--<p>---*/
	public Trigger x1y2() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(4));}
	/**---<p>-x-<p>---*/
	public Trigger x2y2() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(5));}
	/**---<p>--x<p>---*/
	public Trigger x3y2() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(6));}
	/**x--<p>---<p>---*/
	public Trigger x1y3() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(1));}
	/**-x-<p>---<p>---*/
	public Trigger x2y3() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(2));}
	/**--x<p>---<p>---*/
	public Trigger x3y3() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(3));}

	public Trigger povCenter() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> joystick.magnitude() < 0.5);}
	public Trigger povUp() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(90));}
	public Trigger povUpRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(45));}
	public Trigger povRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(0));}
	public Trigger povDownRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(-45));}
	public Trigger povDown() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(-90));}
	public Trigger povDownLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(-135));}
	public Trigger povLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(0, true));}
	public Trigger povUpLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> atAngle(135));}

	public boolean isConnected() {
		return hid.isConnected();
	}

	private static final double tolerance = 0.2;
	private boolean atAngle(double angle) {
		return atAngle(angle, false);
	}
	private boolean atAngle(double angle, boolean useLeft) {
		return joystick.magnitude() > 0.5 &&
			Math.abs((useLeft ? joystick.radsFromNegXCCW() : joystick.radsFromPosXCCW()) - Units.degreesToRadians(angle)) < tolerance;
	}
}
