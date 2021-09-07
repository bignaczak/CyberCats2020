package frc.robot.robotFunctions; 
import com.revrobotics.ControlType; 
import frc.robot.framework.*; 
/** 
* Add your docs here. 
*/ 
public class Shooter { 
    public static void init() { 
        Motors.Shooter.clearFaults(); 
        resetEncoders(); 
    }

    public static void resetEncoders(){ 
        Sensors.Shooter_E.setPosition(0); 
    }

    public static void runShooterMotorManual(double speed) { 
        Motors.Shooter.set(speed); 
    }

    public static void runShooterMotorAuto(double speed){ 
        PIDControllers.PIDShoot.setReference(speed, ControlType.kVelocity);
    } 
    public static void stopShooterMotor(){ 
        Motors.Shooter.set(0); 
    }
}
