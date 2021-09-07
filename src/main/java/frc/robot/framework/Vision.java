/*------------------------------------------------------------------------ ----*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. 
*/ 
/* Open Source Software - may be modified and shared by FRC teams. The code */ 
/* must be accompanied by the FIRST BSD license file in the root directory of */ 
/* the project. 
*/ 
/*------------------------------------------------------------------------ ----*/ 
package frc.robot.framework; 

import edu.wpi.cscore.UsbCamera; 
import edu.wpi.first.cameraserver.CameraServer; 
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance; import frc.robot.Robot; 
public class Vision extends Robot { 
    public static UsbCamera Camera1; 
    public static UsbCamera Camera2; 
    /* 
    public NetworkTable visionTable; 
    public NetworkTableEntry txLocal; 
    public NetworkTableEntry tyLocal; 
    public NetworkTableEntry taLocal; 
    public NetworkTableEntry tvLocal; 
    public NetworkTableEntry tsLocal; 
    public double targetOffsetAngle_Horizontal = 0; 
    public double targetOffsetAngle_Vertical = 0; 
    public double targetArea = 0; 
    public double targetSkew = 0;
    public boolean targetFound = false; 
    */ 
    public static NetworkTable tableLimelight; 
    public static NetworkTableEntry tx; 
    public static NetworkTableEntry ty; 
    public static NetworkTableEntry ta; 
    public static double horizontalError = 0; 
    public static double verticalError = 0; 
    public static double area = 0; 
    public static double xOffset; 
    public static Double aim = -.1; 
    public static Double distance = -.1; 
    public static Double min_aim = -.1; 
    public static double visionSpeed; 
    public void init(){ 
        Camera1 = CameraServer.getInstance().startAutomaticCapture(0);  Camera2 = CameraServer.getInstance().startAutomaticCapture(1);  tableLimelight = 
            NetworkTableInstance.getDefault().getTable("limelight");  tx= tableLimelight.getEntry("tx"); 
        ty = tableLimelight.getEntry("ty"); 
        ta = tableLimelight.getEntry("ta"); 
        //visionTable = 
        NetworkTableInstance.getDefault().getTable("limelight"); 
        /* 
        txLocal = visionTable.getEntry("tx"); 
        tyLocal = visionTable.getEntry("ty"); 
        taLocal = visionTable.getEntry("ta"); 
        tvLocal = visionTable.getEntry("tv"); 
        tsLocal = visionTable.getEntry("ts"); 
        */ 
    } 


    public static void updateLimelight() { 
        horizontalError = tx.getDouble(0.0); 
        verticalError = ty.getDouble(0.0); 
        area = ta.getDouble(0.0);
    } 


    public static double getLimelightTx() { 
        System.out.println("updating limelight"); 
        return horizontalError = tx.getDouble(0.0); 
    } 
    /* 
    public boolean getVisionTargetStatus(){ 
    return tvLocal.getBoolean(false); 
    } 
    public double getVisionTargetHorizontalError(){ 
    return txLocal.getDouble(0); 
    } 
    public double getVisionTargetArea(){ 
    return taLocal.getDouble(0); 
    } 
    public double getVisionTargetVerticalError(){ 
    return tyLocal.getDouble(0); 
    } 
    public double getVisionTargetSkew(){ 
    return tsLocal.getDouble(0); 
    } 
    */ 
    /* 
    public boolean guidedAssist(){ 
    boolean isTargeting = true; 
    targetFound = getVisionTargetStatus(); //Whether the limelight has any valid targets (0 or 1) 
    if (targetFound){ // 
    targetOffsetAngle_Horizontal = txLocal.getDouble(0); //Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    targetOffsetAngle_Vertical = tyLocal.getDouble(0); //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees) 
    targetArea = taLocal.getDouble(0); //Target Area (0% of image to 100% of image) 
    targetSkew = tsLocal.getDouble(0); //Skew or rotation (-90 degrees to 0 degrees) 
    isTargeting = 
    !(ApolloDrive.turnToDegre[e(targetOffsetAngle_Horizontal, "offset", "gyro", rightSideRequiresInversion)); 
    }else{ 
    //No target is found 
    isTargeting = false; 
    } 
    return isTargeting; //since we are using in main code to block driver controls we must provide true when running not when complete  } //End of guidedAssist 
    */ 


    public static double getXOffset() { 
        if(horizontalError > -2.0 && horizontalError < 2.0) {  horizontalError = 0; 
        } 
        return horizontalError; 
    } 


    public static double getSpeedCommand(double offset, double target) {  target = 0.0; 
        Double currentValue = offset - target; 
        if(Math.abs(currentValue) != 0.0) { 
            if(currentValue > 0.0) { 
                visionSpeed = .05; 
            } else if(currentValue < 0.0) { 
                visionSpeed = -.05;
            } else { 
                visionSpeed = 0.0; 
            } 
        }
        return visionSpeed; 
    } 


    public static double getDistanceFromTarget(double theta) { 
        return 1.0; //ONLY AS A PLACE HOLDER 
    } 
    public static double getShooterSpeedLimelight() { 
        return 1.0;//ONLY AS A PLACE HOLDER 
    } 
    public static void shootAtDistance(double distanceFromTarget, double shooterSpeed) { 
        //process distance from target 
        //increase shooter speed with distance 
        //find shooterspeed 
    } 
}
