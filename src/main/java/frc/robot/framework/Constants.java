package frc.robot.framework; 
/** 
* Add your docs here. 
*/ 
public interface Constants { 
//Basic Control Variables for the bot 
public static boolean rightSideRequiresInversion = true; 
public static boolean fieldOrient = false; 
public static boolean useStraightDriveinAuton = false; 
//Motor Speed and Auton Values 
public static double shooterAutonRPM = 4500; 
public static double shooterManualPWM = 1.0; 
public static double spitBallOutPWM = 0.5; 
public static double colorWheelSpinManualPWM = 0.5; 
public static double intakeArmMoveManualPWM = 0.1; 
public static double conveyorFeedBallToShooterPWM = 0.5; public static double conveyorFeedBallToShooterPulseTime = 1.2; 
//Robot Dimensions 
public static double robotLen = 25; //Set 
public static double robotWidth = 0; // Set 
public static double intakeLenBeyondFrame = 0; //Set 
//Field Dimensions 
//Drivetrain Encoder values 
public static double drivetrain_encCPR = 8192/12.58; 
public static double drivetrain_encCPI = drivetrain_encCPR/Math.PI*4.0; public static double drivetrain_encDistPerPulse = 1/drivetrain_encCPI; public static double drivetrainAccuracy = 10; 
//RobotFunctions Encoder Values 
public static double intakeArmMoveTarget = 10; // Set 
}
