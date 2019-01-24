/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6520.robot;

import org.usfirst.frc.team6520.robot.subsystems.SS_Drivebase;
import org.usfirst.frc.team6520.robot.subsystems.SS_Vision;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static SS_Drivebase ss_drivebase = new SS_Drivebase();
	public static SS_Vision ss_vision = new SS_Vision();
	
	public static VictorSP leftmotor = new VictorSP(2);
	public static VictorSP rightmotor = new VictorSP(1);
	public static Encoder encoder = new Encoder(4,5,false,EncodingType.k4X);
	
	public static void update(){
		SmartDashboard.putNumber("encoder", RobotMap.encoder.getRaw());
	}
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}