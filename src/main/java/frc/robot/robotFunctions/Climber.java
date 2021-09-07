/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */ /* Open Source Software - may be modified and shared by FRC teams. The code */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project. */ 
/*----------------------------------------------------------------------------*/ 
package frc.robot.robotFunctions; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.revrobotics.ControlType; 
import frc.robot.framework.*; 
/** 
* Add your docs here. 
*/ 
public class Climber { 
    //public static CANSparkMax climberMotor = Motors.climberMotor; 
        public static void init() { 
        Motors.ClimberMotor.clearFaults(); 
        Motors.ClimberSolenoid.clearStickyFaults(); 
        resetEncoders(); 
    } //End of init method 


    public static void resetEncoders(){ 
        Sensors.Climber_E.setPosition(0); 
    } 


    public static void runClimberManual(String direction, Double speed) {
        if (direction == "Lift") { 
            Motors.ClimberMotor.set(speed); 
        } else if (direction == "Lower") { 
            Motors.ClimberMotor.set(-speed); 
        } else { 
            stopClimberMotor(); 
        } 
    }


    public static void stopClimberMotor() { 
        Motors.ClimberMotor.set(0);
    } 


    public static void retractPin(){ 
        Motors.ClimberSolenoid.set(ControlMode.PercentOutput, 1); 
    } 


    public static void engagePin() { 
        Motors.ClimberSolenoid.set(ControlMode.PercentOutput, 0); 
    } 


    public static String getClimberPinStatus() { 
        return Motors.ClimberSolenoid.getMotorOutputPercent() > 0.1 ? "Retracted" : "Engaged"; 
    } 

    public static void runClimberAuto(String direction, double RPM, int desiredCounts) {
        if (direction == "Lift"){ 
            PIDControllers.PIDClimber.setReference(desiredCounts, ControlType.kPosition); 
        } else if (direction == "Lower"){ 
            PIDControllers.PIDClimber.setReference(0, ControlType.kPosition); 
        } 
    }//End of runClimberAuto Method 

} //End of Class
