/*------------------------------------------------------------------------ ----*/ 
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. 
*/ 
/* Open Source Software - may be modified and shared by FRC teams. The code */ 
/* must be accompanied by the FIRST BSD license file in the root directory of */ 
/* the project. 
*/ 
/*------------------------------------------------------------------------ ----*/ 
package frc.robot; 

import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.GenericHID.Hand; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auton.*; 
import frc.robot.robotFunctions.*; 
import frc.robot.framework.*; 
/** 
 * The VM is configured to automatically run this class, and to call the  * functions corresponding to each mode, as described in the TimedRobot  * documentation. If you change the name of this class or the package after 
 * creating this project, you must also update the build.gradle file in the 
 * project. 
 */ 
public class Robot extends TimedRobot implements Constants{ 
private double autonomousShootDelay = 0; 
private double timeToExecuteAuton = 0; 
private String autonRoutine = "None Selected"; 
private boolean visionActive = false;
private boolean testRoutineActive = false; 
public static Vision ApolloVision = new Vision(); 
public static Spinner ApolloColorSpinner = new Spinner();
public static Dashboard ApolloDashboard = new Dashboard();
public static Conveyor ApolloConveyor = new Conveyor();
public static Climber ApolloClimber = new Climber(); 
public static DriveBase ApolloDrive = new DriveBase();
public static Shooter ApolloShooter = new Shooter(); 
//Leaving Sensors, Motors, PIDController and AutomatedFunctions Class objects as static objects 
public static Intake ApolloIntake = new Intake(); 
public static Controls CybercatsDriveStation = new Controls(); //Allow to be static since may be used in drivebase 


@Override 
public void robotInit() { 
    ApolloVision.init(); //initiates Camera Streams for Dashboard  ApolloColorSpinner.init(); //clears motor faults, resets encoders, initializes colors 
    ApolloDashboard.init(); //loads smartdashboard with default values  ApolloConveyor.init(); //clears motor faults, resets encoders  ApolloClimber.init(); //clears motor faults, resets encoders  ApolloIntake.init(); //clears motor faults, resets encoders  ApolloDrive.init(); //clears motor faults, resets encoders  ApolloShooter.init(); //clears motor faults, resets encoders  Sensors.init(); //resets drive Encoders, resets the Gyro, Set Distance Per pulse on the encoders 
    PIDControllers.configurePIDControllers(); //Set PID constants for the various PID Controllers 
    ApolloVision.init(); //sets up the limelight data 
    visionActive = false; 
    testRoutineActive = false; 
} //End of robotInit


@Override 
public void robotPeriodic() { 
    UpdateSmartDashboard(); //update dashboard based on latest information 
} 

@Override 
public void autonomousInit() { 
    autonomousShootDelay = ApolloDashboard.getAutonDelay(); //get the delay from the driveTeam 
    autonRoutine = ApolloDashboard.getAutonChoice(); //get the choice of move or shoot and move from the driveTeam 
    autonomousShootDelay = boundAutonomousDelay(autonomousShootDelay, autonRoutine); //binds the value to safeguard from bad input  SmartDashboard.putNumber("Autonomous Delay", autonomousShootDelay); //Overwrite the input so drivers see updated value if bad input  AutomatedFunctions.init(); //resets all variables used in methods  // AutonWeek1.init(); //resets all variables including Step variable  TrenchAuton.init(); 
} 

@Override 
public void autonomousPeriodic() { 
//AutonWeek1.run(autonomousShootDelay, autonRoutine);  TrenchAuton.run(autonomousShootDelay); 
} 

@Override 
public void teleopPeriodic() { 
/*** 
 if (!visionAssistActive){ //Override Driver Control of Robot when secondary is shooting with Vision Assist 
getFlightStickRequests(); 
} 
if (testModeActive){ //Allow for a test mode for now to try out base automated methods 
getXboxTestRequests();
}else{ 
getXboxRequests(); 
} 
**/ 

getXboxTestRequests(); 
/* 
if(!visionActive) { 
getFlightStickRequests(); 
} 
*/ 
}//End of TeleopPeriodic method 

@Override 
public void testPeriodic() { 
} //End of TestPeriodic 
/************************************************************************* **************************** */ 
/* 
*/ 
/* 
*/ 
/* START OF CUSTOM METHODS */ 
/* 
*/ 
/* 
*/ 
/************************************************************************* **************************** */ 

    public double boundAutonomousDelay(double inputDelay, String autonPath){
//Set Variable for timeToExecuteAuton based on AutonPath chosen
    switch (autonPath){ 
        case "Shoot": 
            timeToExecuteAuton = 5; 
            break; 
        case "Move Only": 
            timeToExecuteAuton = 2; 
            break; 
    } 
    //bound if driverstation input is too large 
    return inputDelay >= (15 - timeToExecuteAuton)?(15-timeToExecuteAuton):inputDelay; 
}
//End of boudAutonomousDelay 


public void getFlightStickRequests(){ 
    //Basic Driving with primary joystick 
    //TODO:  Figure out why the driveProcessing and Drive functions can't be found
    // double TeleopLeftSideDrive = ApolloDrive.driveProcessing(Utilities.deaden(CybercatsDriveStation.joystick.getRawAxis(0), Robot.xAxisDeadband),Utilities.deaden(CybercatsDriveStation.joystick.getRawAxis(1), Robot.yAxisDeadband))[0]; 
    // double TeleopRightSideDrive = ApolloDrive.driveProcessing(Utilities.deaden(CybercatsDriveStation.joystick.getRawAxis(0), Robot.xAxisDeadband),Utilities.deaden(CybercatsDriveStation.joystick.getRawAxis(1), Robot.yAxisDeadband))[1]; 
    // ApolloDrive.drive(TeleopLeftSideDrive,TeleopRightSideDrive); 
    //Controls for the Color Wheel Lift Mechanism 
    if (CybercatsDriveStation.joystick.getRawButton(7)){
        ApolloColorSpinner.runColorAsmLiftManual("Lift", 0.5);  
    } else if (CybercatsDriveStation.joystick.getRawButton(8)){
        ApolloColorSpinner.runColorAsmLiftManual("Lower", 0.5);
    } else{ 
        ApolloColorSpinner.stopColorAsmLiftMotor(); 
    } 

    //Controls for the Color Wheel Spin Mechanism 
    if (CybercatsDriveStation.joystick.getRawButton(11)){
        ApolloColorSpinner.runColorSpinMotorManual("CW", Robot.colorWheelSpinManualPWM); 
    } else if (CybercatsDriveStation.joystick.getRawButton(12)){
        ApolloColorSpinner.runColorSpinMotorManual("CCW", Robot.colorWheelSpinManualPWM); 
    } else if (CybercatsDriveStation.joystick.getRawButton(9)){ 
        ApolloColorSpinner.Spin3Times(CybercatsDriveStation.joystick.getRawButtonPressed(9)); 
    } else if (CybercatsDriveStation.joystick.getRawButton(10)){
        ApolloColorSpinner.GoToTarget(); 
    } else { 
        ApolloColorSpinner.stopColorSpinMotor(); 
    } 
} //End of getFlightStickRequests 

public void getXboxRequests(){ 
    //Vision Request 
    if(CybercatsDriveStation.xbox.getAButton()) { 
        AutomatedFunctions.visionAlign(Vision.getXOffset(),Vision.getSpeedCommand( Vision.getXOffset(), 0.0)); 
        visionActive = true; 
    } else { 
        visionActive = false; 
    } 
    /* 
    if (CybercatsDriveStation.xbox.getYButton()){
    if(!ApolloVision.guidedAssist()){ 
    visionAssistActive = true; 
    }else{ 
    visionAssistActive = false; 
    //No neeed to stop drivetrain motors as we check flightstick which will stop motors if no input 
    } 
    }*/ 
    //Climber Pin Retraction Requests 
    if (CybercatsDriveStation.xbox.getBackButton()){ 
        ApolloClimber.retractPin(); 
    } else if (CybercatsDriveStation.xbox.getStartButton()){
        ApolloClimber.engagePin(); 
    } 
    //Climber Requests 
    if (CybercatsDriveStation.xbox.getBumper(Hand.kLeft)){
        ApolloClimber.runClimberManual("Lift", 0.7); 
    } else if (CybercatsDriveStation.xbox.getBumper(Hand.kRight)){
        ApolloClimber.runClimberManual("Lower", 0.3); 
    } else { 
        ApolloClimber.stopClimberMotor(); 
    } 
    //Shooter Requests 
    if (CybercatsDriveStation.xbox.getXButton()){ 
    // ApolloShooter.runShooterMotorManual("Shoot", 1.0);  } else{ 
    // ApolloShooter.stopShooterMotor(); 
    } 
    //Intake and Conveyor Requests 
    if (CybercatsDriveStation.xbox.getBButton()){ //Spit out balls with the B button 
        ApolloConveyor.runConveyorUpperManual("Out", Robot.spitBallOutPWM); 
        ApolloConveyor.runConveyorBottomManual("Out", Robot.spitBallOutPWM);
        if (ApolloIntake.getIntakeArmPosition(Robot.intakeArmMoveTarget) == "Lowered"){
            //if intake arm sweeper down, go ahead and spit out  
            ApolloIntake.runIntakeArmSweeperManual("Out", Robot.spitBallOutPWM); 
        } 
    } else if (CybercatsDriveStation.xbox.getTriggerAxis(Hand.kLeft) > 0.07) {
            //Use Left Trigger to run Upper Conveyor to feed balls to shooter  
            ApolloConveyor.runConveyorUpperManual("In", CybercatsDriveStation.xbox.getTriggerAxis(Hand.kLeft)*0.5);
    } else if (CybercatsDriveStation.xbox.getTriggerAxis(Hand.kRight) > 0.07){ 
        //Use Right Trigger to run Intake Sweeper and Bottom Conveyor to pull in balls 
        ApolloIntake.runIntakeArmSweeperManual("In", CybercatsDriveStation.xbox.getTriggerAxis(Hand.kRight) );  
        ApolloConveyor.runConveyorBottomManual("In", 1.0);  
        ApolloConveyor.stopConveyorUpperMotor(); 
    } else{ 
        ApolloIntake.stopIntakeArmSweeperMotor(); 
        ApolloConveyor.stopConveyorBottomMotor(); 
        ApolloConveyor.stopConveyorUpperMotor(); 
    }

    //Intake Arm Requests 
    if (CybercatsDriveStation.xbox.getPOV() == 0){ 
        ApolloIntake.runIntakeArmLiftManual("Lift", Robot.intakeArmMoveManualPWM); 
    } else if (CybercatsDriveStation.xbox.getPOV() == 180){
        ApolloIntake.runIntakeArmLiftManual("Lower", Robot.intakeArmMoveManualPWM); 
    } else{ 
        ApolloIntake.stopIntakeArmLiftMotor(); 
    } 
} //End of get Xbox Requests 


public void getXboxTestRequests(){
    if (CybercatsDriveStation.xbox.getStartButton()){ //Use Start Button to reset Gyro and drive encoders 
        ApolloDrive.resetEncoders(); 
        //TODO:  Figure out why initialization of Gyro was commented out
        // Sensors.Gyro.setYaw(0); 
    } 

    //TODO:  figure out where getGyroAngle should be stored
    // if(CybercatsDriveStation.xbox.getAButton()) { 
    //     AutomatedFunctions.turnToExactDegree(Sensors.getGyroAngle(), 90.0, "CW"); 
    // } else if(CybercatsDriveStation.xbox.getBButton()) {  
    //     AutomatedFunctions.turnToExactDegree(Sensors.getGyroAngle(), 90.0, "CCW"); 
    // } else { 
    //     ApolloDrive.stopAllDrivetrainMotors(); 
    // } 
    /* 
    if (CybercatsDriveStation.xbox.getYButton()){ //Drive Linear using PID in Roborio and Shaft Encoders as feedback 
    if(!AutomatedFunctions.runLinearDistanceAuton("PID", 0, ApolloDashboard.getTestDriveStraight())){ 
    testRoutineActive = true; 
    }else{ 
    ApolloDrive.stopAllDrivetrainMotors(); 
    } 
    }else if (CybercatsDriveStation.xbox.getXButton()){ //Turn to Degree using the Gyro as feedback 
    if(!ApolloDrive.turnToDegree(ApolloDashboard.getTestTurnToDegree(), "Offset", "Gyro", rightSideRequiresInversion)){ 
    testRoutineActive = true; 
    }else{ 
    ApolloDrive.stopAllDrivetrainMotors(); 
    } 
    }else if (CybercatsDriveStation.xbox.getBButton()){ //Turn using Encoders as feedback 
    if(){
    testRoutineActive = true; 
    }else{ 
    ApolloDrive.stopAllDrivetrainMotors(); 
    } 
    */ 
    /*else if (CybercatsDriveStation.xbox.getAButton()){ //Turn at a constant speed till aligned with target, no feedback 
    if 
    (!ApolloDrive.turnToFindTargetAtSetSpeed(ApolloVision.getVisionTargetHoriz ontalError(), visionTrackingTurnSpeed)){ 
    testRoutineActive = true; 
    }else{ 
    ApolloDrive.stopAllDrivetrainMotors(); 
    } 
    } else { 
    testRoutineActive = false; 
    //Do not stop motors as checking the flightstick should handle this  }*/ 
} //End of getTestXboxRequests 


public void UpdateSmartDashboard(){ 
    //Update the positions of moveable parts 
    SmartDashboard.putString("Climber Pin Status", ApolloClimber.getClimberPinStatus()); 
    SmartDashboard.putBoolean("Climber Pin", ApolloClimber.getClimberPinStatus()=="Engaged"?true:false);  
    SmartDashboard.putString("Intake Arm Position", ApolloIntake.getIntakeArmPosition(Robot.intakeArmMoveTarget));  
    SmartDashboard.putString("Color Arm Position", ApolloColorSpinner.getSpinnerLiftPosition()); 
    
    //Update the encoder counts 
    SmartDashboard.putNumber("Intake Arm Encoder: ", Sensors.IntakeLift_E.getPosition()); 
    SmartDashboard.putNumber("Color Arm Encoder: ", Sensors.LiftSpinner_E.getDistance());
    SmartDashboard.putNumber("Climber Encoder: ", Sensors.Climber_E.getPosition()); 
    SmartDashboard.putNumber("Left Side Encoder: ", Sensors.DriveL_E.getDistance()); 
    SmartDashboard.putNumber("Right Side Encoder: ", Sensors.DriveR_E.getDistance()); 
    //Update the encoder velocity rates 
    SmartDashboard.putNumber("Conveyor Upper RPM: ", Sensors.IntakeConveyor_E.getVelocity()); 
    SmartDashboard.putNumber("Conveyor Bottom RPM", Sensors.IntakeBottom_E.getVelocity()); 
    SmartDashboard.putNumber("Intake Sweeper RPM: ", Sensors.IntakeArmSweeper_E.getVelocity()); 
    //SmartDashboard.putNumber("Shooter RPM: ", Sensors.Shooter_E.getVelocity()); 
    // SmartDashboard.putBoolean("Shooter at Speed", Sensors.Shooter_E.getVelocity()<=0.98*Robot.shooterAutonRPM?false:true);
    SmartDashboard.putNumber("Left Side Speed In/sec", Sensors.DriveL_E.getRate()); 
    SmartDashboard.putNumber("Right Side Speed In/Sec", Sensors.DriveR_E.getRate()); 
    //Update the Color Information 
    SmartDashboard.putNumber("ColorChanges", ApolloColorSpinner.Colorcount); 
    SmartDashboard.putString("Detected Color", ApolloColorSpinner.colorString); 
    SmartDashboard.putString("Game Color Target", ApolloColorSpinner.gameTargetColorString); 
    
    //Update Gyro Information 
    //TODO:  Figure out why getGyroAngle not defined
    // SmartDashboard.putNumber("Gyro Angle", Sensors.getGyroAngle()); 
    //Update Vision Information 
    SmartDashboard.putNumber("Offset", Vision.getXOffset());  // SmartDashboard.putNumber("limelightx", Vision.getLimelightUpdate());  
    /*
    SmartDashboard.putBoolean("Target Found", ApolloVision.getVisionTargetStatus()); 
    SmartDashboard.putNumber("Target Area %", ApolloVision.getVisionTargetArea()); 
    SmartDashboard.putNumber("Target Angle Error H", ApolloVision.getVisionTargetHorizontalError());  
    SmartDashboard.putNumber("Target Angle Error V", ApolloVision.getVisionTargetVerticalError()); */ 

    } //End of updateSmartDashboard method  
} //End of Robot Class
