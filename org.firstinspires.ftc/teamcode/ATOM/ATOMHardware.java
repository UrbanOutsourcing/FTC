/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 
 */

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for the robot.
 * 
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "LeftDriveRear","LeftDriveFront"
 * Motor channel:  Right drive motor:        "RightDriveRear","RightDriveFront"
 * Motor channel:  Manipulator drive motor:  "liftArm"
 * Servo channel:  Servo to open left claw:  "leftClaw"
 * Servo channel:  Servo to open right claw: "rightClaw"
 */

package org.firstinspires.ftc.teamcode.ATOM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ATOMHardware
{
    /* Public OpMode members. */
    public DcMotor  LeftDriveRear   = null;
    public DcMotor  RightDriveRear  = null;
    public DcMotor  LeftDriveFront   = null;
    public DcMotor  RightDriveFront  = null;
    public DcMotor  liftArm     = null;
    public DcMotor  takeArm     = null;
    public Servo  leftClaw    = null;
    public Servo  rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    
    public  double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Neverest Motor Encoder
    public  double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    public  double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public  double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    public  double     DRIVE_SPEED             = 0.6;
    public  double     TURN_SPEED              = 0.5;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ATOMHardware(){

    }
   
   public ATOMHardware(double cpr, double gearReduction,double wheelDiameter){
       this.COUNTS_PER_MOTOR_REV = cpr;
       this.DRIVE_GEAR_REDUCTION = gearReduction;
       this.WHEEL_DIAMETER_INCHES = wheelDiameter;
       
       this.COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    }
   
   
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftDriveRear  = hwMap.get(DcMotor.class, "LeftDriveRear");
        RightDriveRear = hwMap.get(DcMotor.class, "RightDriveRear");
        LeftDriveFront  = hwMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hwMap.get(DcMotor.class, "RightDriveFront");
        liftArm    =      hwMap.get(DcMotor.class, "liftArm");
        takeArm    =      hwMap.get(DcMotor.class, "takeArm");
        LeftDriveRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RightDriveRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        LeftDriveRear.setPower(0);
        RightDriveRear.setPower(0);
        LeftDriveFront.setPower(0);
        RightDriveFront.setPower(0);
        liftArm.setPower(0);
        takeArm.setPower(0);
        

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        takeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        

        // Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
    }
 }

