package org.usfirst.frc.team5243.robot.subsystems;

import org.usfirst.frc.team5243.robot.Robot;
import org.usfirst.frc.team5243.robot.RobotMap;
import org.usfirst.frc.team5243.robot.commands.TankDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	VictorSP frontLeft;
	VictorSP frontRight;
	VictorSP backLeft;
	VictorSP backRight;
	SpeedControllerGroup left;
	SpeedControllerGroup right;
	AHRS ahrs = Robot.ahrs;
	
	DifferentialDrive drive;
	
	public DriveSubsystem() {
		frontLeft = new VictorSP(RobotMap.frontLeft);
		frontRight = new VictorSP(RobotMap.frontRight);
		backLeft = new VictorSP(RobotMap.backLeft);
		backRight = new VictorSP(RobotMap.backRight);
		left = new SpeedControllerGroup(frontLeft, backLeft);
		right = new SpeedControllerGroup(frontRight, backRight);
		// = Robot.ahrs;
		drive = new DifferentialDrive(left, right);
		drive.setSafetyEnabled(false);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here. 
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new TankDrive());
    }
    
    public void tankDrive() {
    	drive.tankDrive(-Robot.oi.getLeftStick().getY(), -Robot.oi.getRightStick().getY());
    }
    
    public void moveForward(double distance) {
    	double lastYaw = ahrs.getYaw();
    	double mult = 22;
    	for (int i = 0; i < distance * mult; i++) {
    		drive.tankDrive(.8, .7);
    		Timer.delay(.005);
    		double curYaw = ahrs.getYaw();
    		turn(curYaw - lastYaw);
    		lastYaw = curYaw;
    		
    	}
    	drive.tankDrive(0, 0);
    }
    
    public void turn(double degrees) {
    	double mult = .19;
    	for (int i= 0; i < degrees * mult; i++) {
    		drive.tankDrive(degrees, -degrees);
    		Timer.delay(.01);
    	}
    	drive.tankDrive(0, 0);
    }
    
}

