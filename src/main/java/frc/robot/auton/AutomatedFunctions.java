package frc.robot.auton; 
import com.ctre.phoenix.motorcontrol.ControlMode; import com.revrobotics.ControlType; 
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; import frc.robot.framework.PIDControllers; 
import frc.robot.Robot; 
import frc.robot.framework.Motors; 
import frc.robot.framework.Sensors; 
public class AutomatedFunctions extends Robot { 
    private static boolean linearInit = true; 
    private static boolean linearTimeInit = true;  private static double linearTimeTarget = 0; 
    private static double targetLinearDistanceL = 0;  private static double targetLinearDistanceR = 0;
    private static double currentLinearPosition = 0; 
    private static boolean conveyorInit = true; 
    private static boolean latchConveyor = false;
    private static double conveyorRunTimeTarget = 0;
    private static int ballsAttemptedinAuton = 0;
    private static double turnSpeed; 
    private static boolean isFinished = false; 
    public static void init(){ 
        linearInit = true; 
        linearTimeInit = true; 
        conveyorInit = true; 
        latchConveyor = false; 
        ballsAttemptedinAuton = 0; 
    } 
    public static void visionAlign(double offset, double speed) {
        if(Math.abs(offset) > 0.0) {
            Motors.MotorBL.set(speed); 
            Motors.MotorBR.set(speed); 
            Motors.MotorFR.set(speed); 
            Motors.MotorFL.set(speed); 
        } else{ 
        Motors.MotorBL.set(0.0); 
        Motors.MotorBR.set(0.0); 
        Motors.MotorFR.set(0.0); 
        Motors.MotorFL.set(0.0); 
        } 
    } 


    public static boolean runLinearDistanceWithoutEncoders(String direction, double speed, double time){ 
        boolean isFinished = false; 
        if (linearTimeInit){ 
            linearTimeTarget = Timer.getFPGATimestamp() + time;  linearTimeInit = false; 
        }
        if (Timer.getFPGATimestamp() < linearTimeTarget){
            if (direction == "Backwards"){ 
                Motors.MotorFL.set(-speed); 
                Motors.MotorBL.set(-speed); 
                Motors.MotorFR.set(Robot.rightSideRequiresInversion?speed:-speed); 
                Motors.MotorBR.set(Robot.rightSideRequiresInversion?speed:-speed);
            }else if (direction == "Forward"){ 
                Motors.MotorFL.set(speed); 
                Motors.MotorBL.set(speed); 
                Motors.MotorFR.set(Robot.rightSideRequiresInversion?-speed:speed); 
                Motors.MotorBR.set(Robot.rightSideRequiresInversion?-speed:speed);  
            } 
                isFinished = false;
            }
        else{ 
            ApolloDrive.stopAllDrivetrainMotors(); 
            isFinished = true; 
        } 
    return isFinished; 
    } 


    public static boolean runLinearDistanceAuton(String type, double speed, double distance) { 
        boolean isFinished = false; 
        if (linearInit){ 
            targetLinearDistanceL = distance + 
            (Math.abs(Sensors.DriveL_E.getDistance()) + 
            Math.abs(Sensors.DriveR_E.getDistance()))/2; 
            targetLinearDistanceR = Robot.rightSideRequiresInversion? -distance + (Math.abs(Sensors.DriveL_E.getDistance()) + 
            Math.abs(Sensors.DriveR_E.getDistance()))/2: targetLinearDistanceL;  linearInit = false; 
        }

        currentLinearPosition = (Math.abs(Sensors.DriveL_E.getDistance()) + Math.abs(Sensors.DriveR_E.getDistance()))/2; 
        
        if (type == "PID"){ 
            PIDControllers.PIDDriveL.setReference(targetLinearDistanceL, ControlType.kPosition); 
            PIDControllers.PIDDriveR.setReference(targetLinearDistanceR, ControlType.kPosition); 
            if (Motors.MotorFL.getOutputCurrent() == 0 && Motors.MotorFR.getOutputCurrent() == 0){ //Motors Stopped, we have reached setpoint 
            ApolloDrive.stopAllDrivetrainMotors(); //Just in case... not needed 
            isFinished = true; 
            }else{ 
            isFinished = false; 
            }
        } else if (type == "PWM"){ //Compare (absolute value of difference between target and current) to (absolute value of distance we want to travel - accuracy) 
            boolean leftSideAtTarget = Math.abs((targetLinearDistanceL - currentLinearPosition) - distance) >= (Math.abs(distance) - Robot.drivetrainAccuracy); 
            boolean RightSideAtTarget = Math.abs((targetLinearDistanceR - currentLinearPosition) - distance) >= (Math.abs(distance) - Robot.drivetrainAccuracy); 
            
            if (leftSideAtTarget && RightSideAtTarget) { //Reached Target  Motors.MotorFL.set(0); 
                Motors.MotorBL.set(0); 
                Motors.MotorFR.set(0); 
                Motors.MotorBR.set(0); 
                linearInit = true; 
                isFinished = true; 
            } else{ 
                //drive the motors 
                Motors.MotorFL.set(speed); 
                Motors.MotorBL.set(speed); 
                Motors.MotorFR.set(Robot.rightSideRequiresInversion?-speed:speed); Motors.MotorFR.set(Robot.rightSideRequiresInversion?-speed:speed); 
                /* This is to drive like PID but not using PID controllers, also allows for straight drive compensation  double leftDriveSpeed = DriveBase.autonPIDBehavior(0.9, 0, currentLinearPosition, targetLinearDistanceL); 
                double rightDriveSpeed = Robot.rightSideRequiresInversion? DriveBase.autonPIDBehavior(-0.9, 0, currentLinearPosition, targetLinearDistanceR):DriveBase.autonPIDBehavior(0.9, 0, currentLinearPosition, targetLinearDistanceR); 
                if (Robot.useStraightDriveinAuton){
                leftDriveSpeed = 
                DriveBase.encoderDriveCompensation(leftDriveSpeed, rightDriveSpeed)[0];  rightDriveSpeed = 
                DriveBase.encoderDriveCompensation(leftDriveSpeed, rightDriveSpeed)[1];  } 
                Motors.MotorFL.set(DriveBase.autonPIDBehavior(0.9, 0, currentLinearPosition, targetLinearDistanceL)); 
                Motors.MotorBL.set(DriveBase.autonPIDBehavior(0.9, 0, currentLinearPosition, targetLinearDistanceL)); 
                Motors.MotorFR.set(Robot.rightSideRequiresInversion? DriveBase.autonPIDBehavior(-0.9, 0, currentLinearPosition, targetLinearDistanceR):DriveBase.autonPIDBehavior(0.9, 0, currentLinearPosition, targetLinearDistanceR)); 
                Motors.MotorFR.set(Robot.rightSideRequiresInversion? DriveBase.autonPIDBehavior(-0.9, 0, currentLinearPosition, targetLinearDistanceR):DriveBase.autonPIDBehavior(0.9, 0, currentLinearPosition, targetLinearDistanceR)); 
                */ 
                isFinished = false; 
            } 
        }

        return isFinished; 
    } //End of runLinearDistance method 


    public static boolean shooterManualAuton(double timeout) {  
        Boolean routineComplete = false; 
        Motors.Shooter.set(1.0); 
        if (latchConveyor && !routineComplete){ 
            if (runConveyorAuton(Robot.conveyorFeedBallToShooterPulseTime)){
                ballsAttemptedinAuton += 1; 
                latchConveyor = false; 
            }
        } 
        
        if (ballsAttemptedinAuton ==3 || Timer.getFPGATimestamp() >= timeout){ 
            Motors.Shooter.set(0.0); 
            Motors.IntakeConveyor.set(0); 
            System.out.println("Finished shooting all balls in auton");  Motors.Shooter.set(0.0); 
            routineComplete = true; 
        } 

        return routineComplete; 
    } 

     
    public static boolean runShooterAuton(double timeout) { 
        Boolean shooterUpToSpeed = false; 
        Boolean routineComplete = false; 
        //Start the motors either with max PWM or run to a set speed  ApolloShooter.runShooterMotorManual("Shoot", 1.0);  
        //Shooter.runShooterMotorAuto(4200); 
        SmartDashboard.putNumber("Current Auton Step", 25);  
        SmartDashboard.putNumber("Balls Shot in Auton", ballsAttemptedinAuton); 
        //Check to see if the shooter motor has reached appropriate speed  
        if( Sensors.Shooter_E.getVelocity() < Robot.shooterAutonRPM) { //Revving up the shoot motor 
            shooterUpToSpeed = false; 
        } else if( Sensors.Shooter_E.getVelocity() >= Robot.shooterAutonRPM){ 
            shooterUpToSpeed = true; 
            latchConveyor = true; //Don't want control loop to drop moving conveyor if speed dips during timer 
        }
        //Run the conveyor to move the balls into shooter when shooter is at speed 
        if (latchConveyor && !routineComplete){ 
            if (runConveyorAuton(Robot.conveyorFeedBallToShooterPulseTime)){  ballsAttemptedinAuton += 1; 
                latchConveyor = false; 
            } 
        } 

        if (ballsAttemptedinAuton ==3 || Timer.getFPGATimestamp() >= timeout){ 
            Motors.Shooter.set(0.0);  
            Motors.IntakeConveyor.set(0); 
            System.out.println("Finished shooting all balls in auton");  routineComplete = true; 
        } 

        return routineComplete; 
    } //End of runShooterAuton method 
    
    public static boolean runConveyorAuton(double delay){
        boolean isFinished = false; 
        if (conveyorInit){ 
        conveyorRunTimeTarget = Timer.getFPGATimestamp() + delay;  System.out.println("Current Time = " + Timer.getFPGATimestamp()); 
        System.out.println("End Time for Conveyor Pulse" + conveyorRunTimeTarget); 
        conveyorInit = false; 
        } 
        if (Timer.getFPGATimestamp() - conveyorRunTimeTarget <= 0){  ApolloConveyor.runConveyorUpperManual("In", Robot.conveyorFeedBallToShooterPWM); 
            isFinished = false; 
        } else{
            ApolloConveyor.stopConveyorUpperMotor();  conveyorInit = true; 
            isFinished = true; 
        } 
    return isFinished; 
    } //End of runConveyorAuton 


    public static boolean turnToDegree(double gyroFeedback, double targetDegree, String direction) { 
        Double currentError; 
        System.out.println("in turnToDegree()"); 
            if(direction == "CW") { 
                currentError = gyroFeedback - targetDegree;  System.out.println("getting error"); 
                if(Math.abs(currentError) > 5.0) { 
                    turnSpeed = .1; 
                    isFinished = false; 
                } else { 
                    turnSpeed = 0.0; 
                    isFinished = true; 
                } 
            } else if(direction == "CCW") { 
                currentError = -gyroFeedback - targetDegree;  System.out.println("getting error"); 
                if(Math.abs(currentError) > 5.0) { 
                    turnSpeed = -.1; 
                    isFinished = false; 
                } else { 
                    turnSpeed = 0.0; 
                    isFinished = true; 
                } 
            } 
        Motors.MotorBL.set(turnSpeed);
        Motors.MotorBR.set(turnSpeed); 
        Motors.MotorFR.set(turnSpeed); 
        Motors.MotorFL.set(turnSpeed); 
        return isFinished; 
    }
    
    
    public static boolean turnToExactDegree(double gyroFeedback, double targetDegree, String direction) { 
        Double currentError = targetDegree - gyroFeedback;  Double currentErrorProportion = (targetDegree - gyroFeedback)/targetDegree; 
        if (direction == "CW") { 
            if(currentError < targetDegree) { 
                turnSpeed = currentErrorProportion; 
            } else { 
                turnSpeed = 0.0; 
                isFinished = true; 
            } 
        } else if(direction == "CCW") { 
            if(currentError < targetDegree) { 
                turnSpeed = -currentErrorProportion; 
            } else { 
                turnSpeed = 0.0; 
                isFinished = true; 
            } 
        } 
        return isFinished; 
    } 
}
