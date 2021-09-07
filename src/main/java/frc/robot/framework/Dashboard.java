/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */ /* Open Source Software - may be modified and shared by FRC teams. The code */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project. */ 
/*----------------------------------------------------------------------------*/ 
package frc.robot.framework; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
/** 
* Add your docs here. 
*/ 
public class Dashboard { 
    public static void init(){ 
        //Update the positions of moveable parts 
        SmartDashboard.putString("Climber Pin Status", "unknown"); 
        SmartDashboard.putString("Intake Arm Position", "unknown"); 
        SmartDashboard.putString("Color Arm Position", "unknown"); 
        //Update the encoder counts 
        SmartDashboard.putNumber("Intake Arm Encoder: ", 0); 
        SmartDashboard.putNumber("Color Arm Encoder: ", 0); 
        SmartDashboard.putNumber("Climber Encoder: ", 0); 
        SmartDashboard.putNumber("Left Side Encoder: ", 0); 
        SmartDashboard.putNumber("Right Side Encoder: ", 0); 
        //Color Information 
        SmartDashboard.putNumber("ColorChanges", 0); 
        SmartDashboard.putString("Detected Color", "Unknown"); 
        SmartDashboard.putString("Game Color Target", "Not Provided"); 
        //Update the encoder velocity rates 
        SmartDashboard.putNumber("Conveyor Upper RPM: ", Sensors.IntakeConveyor_E.getVelocity()); 
        SmartDashboard.putNumber("Conveyor Bottom RPM", Sensors.IntakeConveyor_E.getVelocity()); 
        SmartDashboard.putNumber("Intake Sweeper RPM: ", Sensors.IntakeArmSweeper_E.getVelocity()); 
        SmartDashboard.putNumber("Shooter RPM: ", Sensors.Shooter_E.getVelocity());
        SmartDashboard.putNumber("Left Side Speed In/sec", Sensors.DriveL_E.getRate());
        SmartDashboard.putNumber("Right Side Speed In/Sec", Sensors.DriveR_E.getRate()); 
        SmartDashboard.putBoolean("Shooter at Speed", false); 
        //Update Autonomous 
        SmartDashboard.putNumber("Autonomous Delay", 0); 
        SmartDashboard.putNumber("Current Auton Step", 0); 
        SmartDashboard.putNumber("Balls Shot in Auton", 0); 
    } //End of init method 

    public static double getAutonDelay(){ 
        return SmartDashboard.getNumber("Autonomous Delay", 0); 
    } 

    public static String getAutonChoice(){
        // Does nothing
        //  Added because called from Robot.java
        return "";
    }
}
