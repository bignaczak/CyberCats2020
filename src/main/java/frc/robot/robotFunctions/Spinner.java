/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */ /* Open Source Software - may be modified and shared by FRC teams. The code */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project. */ 
/*----------------------------------------------------------------------------*/ 
// ☻ 
package frc.robot.robotFunctions; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.revrobotics.ColorMatch; 
import com.revrobotics.ColorMatchResult; 
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.util.Color; 
import frc.robot.framework.*; 
/** 
* Add your docs here. 
*/ 
public class Spinner { 
    // Temp. place holders 
    static double kr; 
    static double kg; 
    static double kb; 
    // physical things 
    // Comp. Colors 
    private final static Color kRedTarget = ColorMatch.makeColor(.476074, .391846, .132080); 
    private final static Color kGreenTarget = ColorMatch.makeColor(.215088, .550781, .233887);
    private final static Color kBlueTarget = ColorMatch.makeColor(.180664, .405273, .414307);
    private final static Color kYellowTarget = ColorMatch.makeColor(.325925, .547119, .126593); 
    // place holder color for unknown 
    private final static Color White = ColorMatch.makeColor(0, 0, 0); 
    // Color matcher constructor 
    private final static ColorMatch m_colorMatcher = new ColorMatch(); 
    public static Color Test = ColorMatch.makeColor(kr, kg, kb); 
    // counts changes in color 
    public static double Colorcount;
    public static Color kdetectedColor = White; //Initialize to a "unknown color" 
    public static Double targetEncoderCountPosition = 10.0; //need to test with correct values --- Double or int? 
    private static String gamedata; 
    private static char gameTargetColor; 
    public static String gameTargetColorString = "Null"; 
    public static String colorString = "Null"; 
    public static Color test2 = ColorMatch.makeColor(kr, kg, kb); 
    public static double YEET = .5; 
    
    public static void init() { 
        Motors.SpinnerSpin.clearStickyFaults(); 
        Motors.SpinnerLift.clearStickyFaults();//is this a victorSPX specific? 
    }

    public static void resetEncoders(){ 
        Sensors.LiftSpinner_E.reset(); 
    } 

    public static void runColorSpinMotorManual(String direction, double speed) {
        double spinSpeed = 0; 
        setColor(Sensors.Colorsensor.getColor()); 
        if (direction == "CW"){ 
            spinSpeed = speed; 
        }else if (direction == "CCW"){ 
            spinSpeed = -speed; 
        } 
        Motors.SpinnerSpin.set(ControlMode.PercentOutput, spinSpeed); 
    }// end of runColorSpinMotorManual 


    public static void stopColorSpinMotor(){ 
        Motors.SpinnerSpin.set(ControlMode.PercentOutput, 0); 
    }
    public static void runColorAsmLiftManual(String direction, Double speed) {
        if (direction == "Lift") { 
            Motors.SpinnerLift.set(ControlMode.PercentOutput, -speed); 
        } else if(direction == "Lower"){ 
            Motors.SpinnerLift.set(ControlMode.PercentOutput, speed); 
        } else { 
            stopColorAsmLiftMotor(); 
        } 
    } 


    public static void stopColorAsmLiftMotor(){ 
        Motors.SpinnerLift.set(ControlMode.PercentOutput, 0); 
    } 


    public static String getSpinnerLiftPosition() { 
        String returnPosition = "unknown"; 
        if(Sensors.LiftSpinner_E.getDistance() == 0){ 
            returnPosition = "Raised"; 
        }else if (Sensors.LiftSpinner_E.getDistance() >= 0 && Sensors.LiftSpinner_E.getDistance() < targetEncoderCountPosition ){ 
            returnPosition = "In Between"; 
        }else if (Sensors.LiftSpinner_E.getDistance() == targetEncoderCountPosition){
            returnPosition = "Lowered"; 
        } 
        return returnPosition; 
    }//End of getSpineerLiftPosition 


    public static void setColor(Color detectedColor){ 
    // constructors 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    // set the detected color to be the same as the closest color its at 
    if (match.color == kBlueTarget) { 
        colorString = "Blue"; 
        kdetectedColor = kBlueTarget; 
        } else if (match.color == kRedTarget) { 
        colorString = "Red"; 
        kdetectedColor = kRedTarget; 
        } else if (match.color == kGreenTarget) { 
        colorString = "Green"; 
        kdetectedColor = kGreenTarget; 
        } else if (match.color == kYellowTarget) { 
        colorString = "Yellow"; 
        kdetectedColor = kYellowTarget; 
        } else { 
        colorString = "Unknown"; 
        kdetectedColor = White; 
        } 
    System.out.println(colorString); 
    }

    public static void StoreColor(Color detectedColor) { 
        // If there is a change in color, increase a variable 
        if (detectedColor != Test) { 
        Colorcount += 1; 
        } 
        // *Stores* detected color, for next run comparison 
        Test = detectedColor; 
        // FOR TESTING | 
        // | 
        // | 
        // ↓ 
        // System.out.println("Test: " + Test.hashCode()); 
        // System.out.println("detectedColor: " + detectedColor.hashCode()); System.out.println("Colorcount: " + Colorcount); 
    }


    public static void Spin3Times(Boolean ButtonPressed) { setColor(Sensors.Colorsensor.getColor()); 
        StoreColor(kdetectedColor); 
        // As long as both the botton and the total rotations is below the wanted // amount, the speed will be .75 
        if (ButtonPressed == true){ 
            Colorcount = 0; 
        } 
        if (Colorcount < 31) { 
            Motors.SpinnerSpin.set(ControlMode.PercentOutput, 0.75); } else { 
            Motors.SpinnerSpin.set(ControlMode.PercentOutput, 0); } 
        }
        
        
    public static Color Picktargetcolor() { 
        try{ 
            gamedata = DriverStation.getInstance().getGameSpecificMessage();
            gameTargetColor = gamedata.charAt(0); 
        } 
        catch(Exception exception){ 
            System.out.println("No Color Provided"); 
            gameTargetColor = ' '; 
        } 

        switch(gameTargetColor){ 
            case 'R': 
                gameTargetColorString = "RED"; 
                break; 
            case 'B': 
                gameTargetColorString = "BLUE"; 
                break; 
            case 'Y':
                gameTargetColorString = "YELLOW"; 
                break; 
            case 'G': 
                gameTargetColorString = "GREEN"; 
                break; 
            case ' ': 
                gameTargetColorString = "NULL"; 
        } 

        if (gameTargetColorString == "BLUE") { 
            test2 = kRedTarget; 
            System.out.println("Target: RED"); 
        } 
        else if (gameTargetColorString == "RED"){ 
            test2 = kBlueTarget; 
            System.out.println("Target: BLUE"); 
        } 
        else if (gameTargetColorString == "GREEN"){ 
            test2 = kYellowTarget; 
            System.out.println("Target: Yellow"); 
        } 
        else if (gameTargetColorString == "YELLOW"){ 
            test2 = kGreenTarget; 
            System.out.println("Target: GREEN"); 
        } 
        else if (gameTargetColorString == "NULL"){ 
            test2 = White; 
        }

        return test2; 
    } 

    
    public static void GoToTarget(){ 
    Color TargetColor; 
    setColor(Sensors.Colorsensor.getColor()); 
    TargetColor = Picktargetcolor(); 
    if (kdetectedColor != TargetColor && gameTargetColorString != "NULL"){ 
        Motors.SpinnerSpin.set(ControlMode.PercentOutput, .28); 
    } 
    else{
        Motors.SpinnerSpin.set(ControlMode.PercentOutput, 0); } 
    } 
    /*public static void setrandomcolor(){ 
    if (Controls.joystick.getRawButtonPressed(9) == true) { YEET = Math.random(); 
    } 
    System.out.println("random#: " + YEET); 
    }*/ 
    public static void colorinit() { 
        // GOES IN ROBOTINIT 
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        Colorcount = 0; 
    } 
}
