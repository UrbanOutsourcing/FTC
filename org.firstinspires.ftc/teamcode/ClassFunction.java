/* Copyright (c) 2017 FIRST. All rights reserved.
 
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.EVE.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 
 */
//@Autonomous(name="ClassFunction Java group="Linear Opmode")

public class ClassFunction {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftDriveRear = null;
    private DcMotor RightDriveRear = null;
    //private EVEHardware robot   = new EVEHardware();   // Use a Pushbot's hardware
 
 
  public ClassFunction(DcMotor pLeftDrive,DcMotor pRightDrive){
            LeftDriveRear  = pLeftDrive;
            RightDriveRear = pRightDrive;
    }
    
     public void ClassDrive(double Left, double Right,double timeoutS) {
            
            
            runtime.reset();
            //robot.init(hardwareMap); 
            while (runtime.seconds() < timeoutS) {
              
             LeftDriveRear.setPower(Left);
             RightDriveRear.setPower(Right);
           
               }
               // Set Power to Zero
                LeftDriveRear.setPower(0);
                RightDriveRear.setPower(0);
               
        
        }
           
}
