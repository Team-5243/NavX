package org.usfirst.frc.team5243.robot;

import org.usfirst.frc.team5243.robot.commands.Auton;
import org.usfirst.frc.team5243.robot.subsystems.DriveSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program providing a real-time display of navX-MXP values.
 *
 * In the operatorControl() method, all data from the navX-MXP is retrieved and
 * output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles - Compass Heading and 9-Axis Fused Heading
 * (requires Magnetometer calibration) - Linear Acceleration Data - Motion
 * Indicators - Estimated Velocity and Displacement - Quaternion Data - Raw
 * Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for
 * debugging connectivity issues after initial installation of the navX-MXP
 * sensor.
 *
 */
public class Robot extends TimedRobot {
	public static AHRS ahrs = new AHRS(SPI.Port.kMXP);
	Joystick stick;
	public static OI oi;
	public static DriveSubsystem driveSubsystem = new DriveSubsystem();
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	public void robotInit() {
		oi = new OI();
		oi.init();
		stick = oi.leftstick;
		/*try {
			/* Communicate w/navX-MXP via the MXP SPI Bus. */
			/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
			/*
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 
			//ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}*/
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Runs during autonomous mode
	 */

	public void autonomousInit() {
		Auton auton = new Auton();
		auton.start();
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Display navX-MXP Sensor Data on Smart Dashboard
	 */

	public void teleopInit() {

	}

	public void teleopPeriodic() {

		Timer.delay(0.020); /* wait for one motor update time period (50Hz) */

		boolean zero_yaw_pressed = stick.getTrigger();
		if (zero_yaw_pressed) {
			ahrs.zeroYaw();
		}

		SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());

		SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
		SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

		SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
		SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
		SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
		SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

		/* Display estimates of velocity/displacement. Note that these values are */
		/* not expected to be accurate enough for estimating robot position on a */
		/* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
		/* of these errors due to single (velocity) integration and especially */
		/* double (displacement) integration. */

		SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
		SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
		SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
		SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

		/* Display Raw Gyro/Accelerometer/Magnetometer Values */
		/* NOTE: These values are not normally necessary, but are made available */
		/* for advanced users. Before using this data, please consider whether */
		/* the processed data (see above) will suit your needs. */

		SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
		SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
		SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
		SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
		SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
		SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
		SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
		SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
		SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
		SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
		/* Omnimount Yaw Axis Information */
		/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */

		Scheduler.getInstance().run();
	}

}