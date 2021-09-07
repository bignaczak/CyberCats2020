package frc.robot.framework; 

import com.revrobotics.CANPIDController; 
import edu.wpi.first.wpilibj.controller.PIDController; 
import frc.robot.framework.Motors; 
public class PIDControllers { 
    
    //public static CANPIDController kPIDClimb = new CANPIDController(Motors.climberMotor); 
    public static CANPIDController PIDConveyor = new CANPIDController(Motors.IntakeConveyor); 
    public static CANPIDController PIDIntakeBottom = new CANPIDController(Motors.IntakeBottom); 
    public static CANPIDController PIDIntakeArmLift = new CANPIDController(Motors.IntakeArmLift); 
    public static CANPIDController PIDintakeArmSweeper = new CANPIDController(Motors.IntakeArmSweeper); 
    public static CANPIDController PIDShoot = new CANPIDController(Motors.Shooter); 
    public static PIDController PIDSpinnerSpin = new PIDController( .1, 0, 0); 
    public static PIDController PIDSpinnerLift = new PIDController( .1, 0, 0); 
    public static PIDController PIDRightDrive = new PIDController(.1, 0, 0); //Main Drive PID encoders are in Roborio due to encoder input 
    public static PIDController PIDLefttDrive = new PIDController(.1, 0, 0); //Main Drive PID encoders are in Roborio due to encoder input 
    public static CANPIDController PIDDriveL = new CANPIDController(Motors.MotorFL); //Only need 1 per side 
    public static CANPIDController PIDDriveR = new CANPIDController(Motors.MotorFR); //Only need 1 per side 
    public static CANPIDController PIDClimber = new CANPIDController(Motors.ClimberMotor); 
    
    //Set the Appropriate kP,kI,kD values for each built in SparkMAx Neo Hall Effect Sensor
    public static void configurePIDControllers(){ 
        PIDClimber.setP(0.4); 
        PIDClimber.setI(0); 
        PIDClimber.setD(0); 
        PIDIntakeArmLift.setP(0.4); 
        PIDIntakeArmLift.setI(0); 
        PIDIntakeArmLift.setD(0); 
        PIDShoot.setP(0.75); 
        PIDShoot.setI(0); 
        PIDShoot.setD(0); 
    }//End of configurePIDControllers Method
    
} //End of Class
