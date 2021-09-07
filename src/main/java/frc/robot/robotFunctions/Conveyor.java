package frc.robot.robotFunctions; 
import frc.robot.framework.Motors; 
import frc.robot.framework.Sensors; 
public class Conveyor { 
    public static void init() { 
        Motors.IntakeConveyor.clearFaults(); 
        Motors.IntakeBottom.clearFaults(); 
        resetEncoders(); 
    } 


    public static void resetEncoders(){ 
        Sensors.IntakeBottom_E.setPosition(0); 
        Sensors.IntakeConveyor_E.setPosition(0); 
    } 


    public static void runConveyorUpperManual(String direction, double speed) { if (direction == "In"){ 
        Motors.IntakeConveyor.set(speed); 
        }else if (direction == "Out"){ 
        Motors.IntakeConveyor.set(-speed); 
        } 
    } // End of runConveyorFullManual 


    public static void runConveyorBottomManual (String direction, double speed){ 
        if (direction == "In" ){ 
            Motors.IntakeBottom.set(-speed); 
        } else if (direction == "Out"){ 
            Motors.IntakeBottom.set(speed); 
        } 
    } // End of runConveyorBottomManual 


    public static void stopConveyorUpperMotor(){ 
        Motors.IntakeConveyor.set(0); 
    }

    public static void stopConveyorBottomMotor(){
        Motors.IntakeBottom.set(0); 
    } 
} // End of Class
