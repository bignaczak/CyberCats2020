/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */ /* Open Source Software - may be modified and shared by FRC teams. The code */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project. */ 
/*----------------------------------------------------------------------------*/ 
package frc.robot.robotFunctions;

import com.ctre.phoenix.sensors.PigeonIMU.PigeonState; 
import frc.robot.Robot; 
import frc.robot.framework.*; 


public class DriveBase { 
    public static void init() { 
        Motors.MotorFL.clearFaults(); 
        Motors.MotorBL.clearFaults(); 
        Motors.MotorFR.clearFaults(); 
        Motors.MotorBR.clearFaults(); 
        resetEncoders(); 
    } //End of init method 

    public static void resetEncoders(){ 
        Sensors.DriveL_E.reset(); 
        Sensors.DriveR_E.reset(); 
    } 

    public static void drive(boolean rightSideNeedsInversion){ 
        double fwd = -Utilities.deaden(Controls.joystick.getRawAxis(1), 0.1); double strafe = Utilities.deaden(Controls.joystick.getRawAxis(0), 0.1); 
        /**** 
        //Run Field Oriented if Desired and IMU is installed and ready 
        if (Robot.fieldOrient && Sensors.Gyro.getState() == PigeonState.Ready){ double[] ypr = new double[3]; 
        Sensors.Gyro.getYawPitchRoll(ypr); 
        double gyroInputInRadians = Math.toRadians(ypr[0]);
        double temp = fwd*Math.cos(gyroInputInRadians) + 
        strafe*Math.sin(gyroInputInRadians); 
        strafe = -fwd*Math.sin(gyroInputInRadians) + _x*Math.cos(gyroInputInRadians); fwd = temp; 
        } 
        ***/ 
        double L = fwd + strafe; 
        double R = fwd - strafe; 
        R = rightSideNeedsInversion? -R: R; //Invert RightSide if not set in the SparkMax Controller 
        Motors.MotorFL.set(L); 
        Motors.MotorBL.set(L); 
        Motors.MotorFR.set(R); 
        Motors.MotorBR.set(R); 
        /** 
        Motors.MotorFL.set(Utilities.bound(ramp(L),1)); 
        Motors.MotorBL.set(Utilities.bound(ramp(L),1)); 
        Motors.MotorFR.set(Utilities.bound(ramp(R),1)); 
        Motors.MotorBR.set(Utilities.bound(ramp(R),1)); 
        **/ 
    } // end of drive method 


    private static double ramp(double input){ 
        double rampedOutput = 0; 
        if (input > 0){ 
            rampedOutput = Math.min(1.5*input*input + 0.1, 1); 
        }else if (input < 0){ 
            rampedOutput = Math.min(-1, -1.5*input*input -.1); 
        }else{ 
            rampedOutput = 0; 
        } 
        return rampedOutput; 
    } //End of ramp method


    public static double autonPIDBehavior(double k, double a, double current, double Target){ /** 
        * Math Ramping down to target based on encoder count 
        * Function is k*SquareRoot(a + (Target - current)/Target) 
        * 
        * k is overall scaling factor 
        * a is offset since as target reaches current we get 0 
        */ 
        return k*Math.sqrt(a + (Target - current)/Target); 
    } 


    public static void stopAllDrivetrainMotors(){ 
        Motors.MotorFL.set(0); 
        Motors.MotorFR.set(0); 
        Motors.MotorBL.set(0); 
        Motors.MotorBR.set(0); 
    } //End of stopAllDrivetrainMotors 


    public static double[] encoderDriveCompensation(double left, double right){ double leftRate = 0; 
        double rightRate = 0; 
        leftRate = Math.abs(Sensors.DriveL_E.getRate()); 
        rightRate = Math.abs(Sensors.DriveR_E.getRate()); 
        double encoderCompensationThreshold = 1; 
        double compensationFactor = 1; //Do not go above 1. at 1 you are splitting difference and providing max compensation 
        double difference = Math.abs(leftRate - rightRate); 
        double average = (Math.abs(leftRate) + Math.abs(rightRate))/2; 
        double leftAdjustment = 1; //Set default to no compensation 
        double rightAdjustment = 1; //Set default to no compensation 
        //Don't compensate for encoders when turning 
        if( Math.abs(Controls.joystick.getRawAxis(0)) >= 0.1 ) { 
            leftAdjustment = 1; 
            rightAdjustment = 1;
        //Only calculate compensation if outside threshold 
        } else if (difference >= encoderCompensationThreshold){ 
            //Left Side is Faster 
            if (leftRate - rightRate > 0){ 
                leftAdjustment = left*(1 - compensationFactor*(0.5*difference/average)); rightAdjustment = right*(1 + compensationFactor*(0.5*difference/average)); //Right Side is Faster 
            }else { 
                leftAdjustment = left*(1 + compensationFactor*(0.5*difference/average)); rightAdjustment = right*(1 - compensationFactor*(0.5*difference/average)); } 
            //Provide no compensation as within threshold 
            }else{ 
                leftAdjustment = 1; 
                rightAdjustment = 1; 
            } 
            double compensationfactors[] = {leftAdjustment,rightAdjustment}; return compensationfactors; 
    } 
} //End of Class
