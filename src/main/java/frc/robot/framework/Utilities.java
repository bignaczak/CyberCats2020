package frc.robot.framework; 

public class Utilities { 
    public static double deaden(double input, double deadband) { 
        deadband = Math.min(1, deadband); 
        deadband = Math.max(0, deadband); 
        if (Math.abs(input) - deadband < 0) { 
            return 0; 
        } 
        else { 
            return Math.signum(input) * ((Math.abs(input) - deadband) / (1 - deadband)); } 
        } 


    public static double bound(double input, double limitAbsolute){ 
        return Math.abs(input)<=Math.abs(limitAbsolute)?input:Math.signum(input)*limitAbsolute; 
    } 
    
}//end of Utilities Class
