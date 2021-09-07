/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */ /* Open Source Software - may be modified and shared by FRC teams. The code */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project. */ 
/*----------------------------------------------------------------------------*/ 
package frc.robot.robotFunctions; 
import frc.robot.framework.PIDControllers; 
import com.revrobotics.ControlType; 
import frc.robot.Robot; 
import frc.robot.framework.Motors; 
import frc.robot.framework.Sensors; 
public class Intake { 
    public static Boolean ballStatus; 
    public static int ballCount; 
    public static void init() { 
        Motors.IntakeArmLift.clearFaults(); 
        Motors.IntakeArmSweeper.clearFaults(); 
    } 


    public static void resetEncoders(){ 
        Sensors.IntakeLift_E.setPosition(0); 
        Sensors.IntakeArmSweeper_E.setPosition(0); 
    } 


    public static void runIntakeArmLiftManual(String direction, double speed) {
        if (direction == "Lift"){ 
            Motors.IntakeArmLift.set(-speed); 
        } else if (direction == "Lower"){
            Motors.IntakeArmLift.set(speed); 
        } else 
            stopIntakeArmLiftMotor(); 
    } //End of runIntakeArmLiftManual 


    public static void runIntakeArmLiftAuto(String direction, int desiredCounts) {
        if (direction == "Lift"){ 
            PIDControllers.PIDIntakeArmLift.setReference(0, ControlType.kPosition); 
        }else if (direction == "Lower"){ 
            PIDControllers.PIDIntakeArmLift.setReference(desiredCounts, ControlType.kPosition); 
        } 
    } //End of runIntakeArmLiftAuto 


    public static void stopIntakeArmLiftMotor(){ 
        Motors.IntakeArmLift.set(0); 
    } //End of stopIntakeArmLiftMotor 


    public static void runIntakeArmSweeperManual(String direction, double speed) {
        if (direction == "In"){ 
            Motors.IntakeArmSweeper.set(speed); 
        } else if (direction == "Out"){ 
            Motors.IntakeArmSweeper.set(-speed); 
        } 
    } //End of runIntakeArmSweeperManual 


    public static void runIntakeArmSweeperAuto(String direction, double speed){

    } 
    public static void stopIntakeArmSweeperMotor(){ 
        Motors.IntakeArmSweeper.set(0); 
    } // end of stopIntakeArmSweeperMotor

    public static String getIntakeArmPosition(double desiredCounts){ 
        String returnPosition = "unknown"; 
        if(Sensors.IntakeLift_E.getPosition() == 0){ 
            returnPosition = "Raised"; 
        }else if (Sensors.IntakeLift_E.getPosition() >= 0 && Sensors.IntakeLift_E.getPosition() < desiredCounts ){ 
            returnPosition = "In Between"; 
        }else if (Sensors.IntakeLift_E.getPosition() == desiredCounts){ 
            returnPosition = "Lowered"; 
        } 
        return returnPosition; 
    } //end of getIntakeArmPosition 
} // End of Class
