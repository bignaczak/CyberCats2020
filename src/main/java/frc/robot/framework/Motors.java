/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */ /* Open Source Software - may be modified and shared by FRC teams. The code */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project. */ 
/*----------------------------------------------------------------------------*/ 
package frc.robot.framework; 

import com.revrobotics.CANSparkMax; 
import com.ctre.phoenix.motorcontrol.can.VictorSPX; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
/** 
* Add your docs here. 
*/ 
public class Motors { 
    //Motors: named after functions 
    //drive base motors 
    public static CANSparkMax MotorFL = new CANSparkMax(1, MotorType.kBrushless);//CAN ID: 1 
    public static CANSparkMax MotorBL = new CANSparkMax(2, MotorType.kBrushless);//CAN ID: 2 
    public static CANSparkMax MotorFR = new CANSparkMax(3, MotorType.kBrushless);//CAN ID: 3 
    public static CANSparkMax MotorBR = new CANSparkMax(4, MotorType.kBrushless);//CAN ID: 4 
    
    //for sucking in balls to the shooter 
    public static CANSparkMax IntakeConveyor = new CANSparkMax(5, MotorType.kBrushless);//CAN ID: 5 
    public static CANSparkMax IntakeBottom = new CANSparkMax(6, MotorType.kBrushless);//CAN ID: 6 
    public static CANSparkMax IntakeArmSweeper = new CANSparkMax(7, MotorType.kBrushless);//CAN ID: 7 
    public static CANSparkMax IntakeArmLift = new CANSparkMax(8, MotorType.kBrushless);//CAN ID: 8 
    
    //spin wheel 
    public static VictorSPX SpinnerLift = new VictorSPX(9);//CAN ID: 9 
    public static VictorSPX SpinnerSpin = new VictorSPX(10);//CAN ID: 10 
    public static VictorSPX ClimberSolenoid = new VictorSPX(13);//CAN ID: 13
    
    //to shoot the balls 
    public static CANSparkMax Shooter = new CANSparkMax(12, MotorType.kBrushless);//CAN ID: 12 
    
    //public static CANSparkMax shooterLower = new CANSparkMax(, MotorType.kBrushless);//CAN ID: 
    
    //lift robot for climb 
    public static CANSparkMax ClimberMotor = new CANSparkMax(11, MotorType.kBrushless);//CAN ID: 13 
    
    public static CANSparkMax getShooter(){
        return Shooter;
    }

}
