package frc.robot.framework; 

import edu.wpi.first.wpilibj.DigitalInput; 
import edu.wpi.first.wpilibj.Encoder; 
import edu.wpi.first.wpilibj.I2C; 
import frc.robot.Robot; 
//import com.kauailabs.navx.frc.AHRS; 
import com.revrobotics.CANEncoder; 
import com.revrobotics.ColorSensorV3; 
import com.ctre.phoenix.sensors.PigeonIMU; 


public class Sensors { 
    //drive base shaft encoders run into Roborio 
    public static Encoder DriveL_E = new Encoder(0, 1); 
    public static Encoder DriveR_E = new Encoder(2, 3); 
    public static Encoder LiftSpinner_E = new Encoder(4, 5); 
    
    //spinner 
    public final static I2C.Port ki2cPort = I2C.Port.kOnboard; 
    public final static ColorSensorV3 Colorsensor = new ColorSensorV3(ki2cPort); 

    //intake 
    public static DigitalInput Bottomconveyor = new DigitalInput(6); 
    public static DigitalInput Topconveyor = new DigitalInput(7); 
    public static CANEncoder IntakeLift_E = new CANEncoder(Motors.IntakeArmLift);
    public static CANEncoder Climber_E = new CANEncoder(Motors.ClimberMotor);
    public static CANEncoder Shooter_E = new CANEncoder(Motors.Shooter);
    public static CANEncoder IntakeArmSweeper_E = new CANEncoder(Motors.IntakeArmSweeper); 
    public static CANEncoder IntakeConveyor_E = new CANEncoder(Motors.IntakeConveyor);
    public static CANEncoder IntakeBottom_E = new CANEncoder(Motors.IntakeBottom); 
    
    //IMU (Gyro) 
    // public static PigeonIMU Gyro = new PigeonIMU(0); 
    public static void init(){ 
        resetDriveEncoders(); 
    } 
    public static void configureEncoders(){ 
        DriveL_E.setDistancePerPulse(Robot.drivetrain_encDistPerPulse);
        DriveR_E.setDistancePerPulse(Robot.drivetrain_encDistPerPulse);
    } 

    public static void resetDriveEncoders(){ 
        DriveL_E.reset(); 
        DriveR_E.reset(); 
    } 
    /** 
    public static void configureGyro(){ 
    Gyro.clearStickyFaults(); 
    Gyro.setYaw(0); 
    } 
    **/ 
}
