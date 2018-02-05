/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5243.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	Joystick rightstick;
	Joystick leftstick;
	
	//Button switchToPlayback;
	
	public void init() {
		rightstick = new Joystick(1);
		leftstick = new Joystick(0);
	}
	
	public Joystick getLeftStick() {
		return leftstick;
	}
	
	public Joystick getRightStick() {
		return rightstick;
	}
}
