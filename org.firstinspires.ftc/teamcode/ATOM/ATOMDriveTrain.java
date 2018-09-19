/* Copyright (c) 2017 FIRST. All rights reserved.
 
 */

package org.firstinspires.ftc.teamcode.ATOM;

import org.firstinspires.ftc.teamcode.ATOM.*;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

public class ATOMDriveTrain {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftDriveRear = null;
    private DcMotor RightDriveRear = null;
    private ATOMHardware robot   = null;   // Use ATOM and EVE hardware
 
 
    public ATOMDriveTrain(ATOMHardware pRobot){
            robot = pRobot;
            
    }
    
    
     public void PDrive(double Left, double Right,double timeoutS) {
            
            
            //runtime.reset();
            //robot.init(hardwareMap); 
            //while (runtime.seconds() < timeoutS) {
              
             robot.LeftDriveRear.setPower(Left);
             robot.RightDriveRear.setPower(Right);
             robot.LeftDriveFront.setPower(Left);
             robot.RightDriveFront.setPower(Right);
             
             //  }
               // Set Power to Zero
                //robot.LeftDriveRear.setPower(0);
                //robot.RightDriveRear.setPower(0);
               
        
        }
        
        public void EDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, int degrees) {
            
            //runtime.reset();
            InitializeEncoders(leftInches, rightInches,degrees); 
            robot.LeftDriveRear.setPower(Math.abs(speed));
            robot.RightDriveRear.setPower(Math.abs(speed));
            robot.LeftDriveFront.setPower(Math.abs(speed));
            robot.RightDriveFront.setPower(Math.abs(speed));
           
               
             while ((robot.LeftDriveRear.isBusy() & robot.RightDriveRear.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                //                            robot.LeftDriveRear.getCurrent

                //                            robot.RightDriveRear.getCurrentPosition());
                //telemetry.update();
            }  
               // Set Power to Zero
            robot.LeftDriveRear.setPower(0);
            robot.RightDriveRear.setPower(0);
            robot.LeftDriveFront.setPower(0);
            robot.RightDriveFront.setPower(0);
         
        }   
        
         public void InitializeEncoders(double leftInches, double rightInches, int degrees) {
             
            int newLeftTarget;
            int newRightTarget;
            
            // Enable Encoder Mode
            robot.LeftDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            CalculateTarget(leftInches,rightInches,degrees);
            
            // Turn On RUN_TO_POSITION
            robot.LeftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LeftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }   
        
        public void CalculateTarget(double leftInches, double rightInches, int degrees) {
             
            int newLeftTarget;
            int newRightTarget;
            
            switch (degrees) {
             case 90 : 
              // Determine new target position, and pass to motor controller
              newLeftTarget = robot.LeftDriveRear.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
              newRightTarget = robot.RightDriveRear.getCurrentPosition() - (int)(rightInches * robot.COUNTS_PER_INCH);
              robot.LeftDriveRear.setTargetPosition(newLeftTarget);
              robot.RightDriveRear.setTargetPosition(newRightTarget);
            
              newLeftTarget = robot.LeftDriveFront.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
              newRightTarget = robot.RightDriveFront.getCurrentPosition() - (int)(rightInches * robot.COUNTS_PER_INCH);
              robot.LeftDriveFront.setTargetPosition(newLeftTarget);
              robot.RightDriveFront.setTargetPosition(newRightTarget);
             break;
             default : 
            // Determine new target position, and pass to motor controller
              newLeftTarget = robot.LeftDriveRear.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
              newRightTarget = robot.RightDriveRear.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH);
              robot.LeftDriveRear.setTargetPosition(newLeftTarget);
              robot.RightDriveRear.setTargetPosition(newRightTarget);
            
              newLeftTarget = robot.LeftDriveFront.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
              newRightTarget = robot.RightDriveFront.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH);
              robot.LeftDriveFront.setTargetPosition(newLeftTarget);
              robot.RightDriveFront.setTargetPosition(newRightTarget);
                            }
            }    
}
