/* Copyright (c) 2017 FIRST. All rights reserved.
 
 */

package org.firstinspires.ftc.teamcode.ATOM;

import org.firstinspires.ftc.teamcode.ATOM.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 *
 */
@Autonomous(name="ATOMTeleOp Java", group="Linear Opmode")

public class ATOMTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private Servo liftArm = null;
    private ATOMHardware robot   = new ATOMHardware();
    private float clawPosition;
    private double leftPower;
    private double rightPower;
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        InitializeMotors();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            
             telemetry.addData("Calling", "DriveTrain");
             telemetry.update();
             ATOMDriveTrain DriveTrain = new ATOMDriveTrain(robot) ;
             
             
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            
             // Tank Mode uses one stick to control each wheel.
             // - This requires no math, but it is hard to drive forward slowly and keep straight.
              leftPower  = -gamepad1.left_stick_y ;
              rightPower = -gamepad1.right_stick_y ;
                  
              DriveTrain.PDrive(leftPower,rightPower,0.0); // Drive Robot
              OperateExtensions(); // Operate Robot Extensions
                  
            // Show the elapsed game time and wheel power.
                 telemetry.addData("Status", "Run Time: " + runtime.toString());
              telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
              telemetry.update();
        }
        
          
    }
    
     public void InitializeMotors() {
        
        robot.init(hardwareMap);
        robot.pivot.setPosition(.5);
        robot.liftArm.setPosition(.5);
         }
     
     public void OperateExtensions() {
        
                  clawPosition = gamepad2.left_trigger;  // Operate Claw using Servos
                  robot.leftClaw.setPosition(clawPosition);
                  robot.rightClaw.setPosition(clawPosition);
                 //robot.liftArm.setPosition(.5);
                  //robot.liftArm.setPosition(gamepad2.right_trigger);
                  
                  
                 // if( gamepad2.right_bumper) {
                 //  robot.liftArm.setPosition(0);   //Raise liftArm using DcMotors
                 // }
                 // else {
                 //   robot.liftArm.setPosition(1);    //Lower liftArm using DcMotors
                //  }
                 // if( gamepad2.left_bumper) {
                 //   robot.takeArm.setPower(-gamepad2.left_stick_y);   //Outtake takeArm  using DcMotors
                //  }else {
                //    robot.takeArm.setPower(gamepad2.left_stick_y);    //Intake  takeArm  using DcMotors
                //  }
                  robot.liftArm.setPosition(.5);
                  if( gamepad2.dpad_up)  { robot.liftArm.setPosition(0);  }; //Pivot Arm Initial Position
                  if( gamepad2.dpad_down) { robot.liftArm.setPosition(1);  }; //Pivot Arm Full  Position
                  
                  robot.pivot.setPosition(.5);
                  if( gamepad2.dpad_left)  { robot.pivot.setPosition(0);  }; //Pivot Arm Initial Position
                  if( gamepad2.dpad_right) { robot.pivot.setPosition(1);  }; //Pivot Arm Full  Position
         }    
}
