package frc.robot.auton; 

import com.revrobotics.*; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import com.revrobotics.CANEncoder; 
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import frc.robot.framework.PIDControllers; 
import frc.robot.Robot; 
import frc.robot.framework.*; 
import frc.robot.robotFunctions.*; 
public class AutonWeek1 { 
    private static int step = 1; 
    private static boolean initialPass = true; 
    public static double initialDelayTime = 0.0; 


    public static void init() { 
        //Reset the internal variables 
        step = 1; 
        initialPass = true; 
    } 


    public static void run(double delay) { 
        SmartDashboard.putNumber("Current Auton Step", step); 
        switch(step) { 
            case 1: //Initial delay complete before shooting 
            if (initialPass){ 
                initialDelayTime = Timer.getFPGATimestamp() + delay; 
                System.out.println("In Step 1"); 
                System.out.println("Current Time = " + Timer.getFPGATimestamp() + " Set Time to Delay " + initialDelayTime); 
                initialPass = false; 
            }

            if (Timer.getFPGATimestamp() - initialDelayTime <= 0){ 
                //Do nothing, couting down
            } else{ 
                initialPass = true; 
                step += 1; 
            }

            break; 

        case 2: //Shoot Balls 
            if(!AutomatedFunctions.runShooterAuton(500)){ 
            //Do nothing, running the routine 
            } else{ 
                Shooter.stopShooterMotor(); 
                Conveyor.stopConveyorUpperMotor(); 
                step += 1; 
            } 
            break; 

        case 3: //Move Backwards 1 Robot Length 
            //if (!AutomatedFunctions.runLinearDistanceAuton("PWM", 0.5, -Robot.robotLen))
            if (!AutomatedFunctions.runLinearDistanceWithoutEncoders("Backwards", 0.25, 0.75)){ 
                ///Do Nothing, moving backwards 
            } else{ 
                DriveBase.stopAllDrivetrainMotors(); 
                step +=1; 
            } 
                break; 
        } 
    } //End of run method 
} //End of class
