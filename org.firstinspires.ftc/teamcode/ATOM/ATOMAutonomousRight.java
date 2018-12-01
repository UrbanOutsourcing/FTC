/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ATOM;

//import org.firstinspires.ftc.teamcode.ATOM.*;

import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="ATOMAutonomousRight Java", group="Linear Opmode")

public class ATOMAutonomousRight extends LinearOpMode {

    // Declare OpMode members.
      private ElapsedTime runtime = new ElapsedTime();
    
      private  double leftDistance  = 30;  // Hardcoded Distance when not using Vuforia
      private  double rightDistance = 30;
      private double drivePower = .5;      //Default Motor Power
      
      
      private ATOMHardware robot   = new ATOMHardware(); //Create Robot Instance
      private ATOMDriveTrain DriveTrain = new ATOMDriveTrain(robot); //Create DriveTrain Instance passing Robot
      
      
      private ColorSensor sensorColor;
      private DistanceSensor sensorDistance;
      
      //Location Variables in Inches and Degrees
      private double distanceToDepot = 48;
      private double distanceToCrater = -110;
      private double distanceToMineral = 40;
      private double distanceToTarget = 43;
      private double degreesToTarget = -45;
      private double degreesToDepot = -90;
      private double degreesToMineral = 30;
      private double detectAttempts = 10;
      private double initialAngle;
      private String goldPosition = "None";
      
      //private BNO055IMU imu;
      //private BNO055IMU.Parameters imuParameters;
      //private Orientation angles;
      // Acceleration gravity;
      
      private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
      private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
      private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
      
      private List<VuforiaTrackable> allTrackables;
      private static final String VUFORIA_KEY = "AU63SW3/////AAABmUFRPq/ohU8CpLVdaXLTNlg+DNBh3enzFq8SxmRlvTxY6QYiRY2ich0trSJ1UKpoAspqzCjBBiFPERZ5yhrSU8nLyN+mWAB8AU/oKCMl4IyVVqGcYVM4wu/fWzDp8ox/IiX5RzfvhvH0F0KpS3y9VGYIDwOactEIJEMzJYKmAAvGIJCME1YqYAIeoFr38DDpEkqf7gTA8qxQI8Y/AsUOD+v85DHAoGvQbj2JYneG7dZFWQNTk4TtQXW0WAO6bk5cBM9sS3PwokzGQ/N8GsWzGEe6MnpHB589KgY6b97xIhGV6ji1Dhkw+Jj5pg/FrmGOieaCxmiVTNcBIctv6hCJrN6jQOa13CbKkSDwKDTU1qli";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
      private static final float mmPerInch        = 25.4f;
      private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
      private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
      private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

      private OpenGLMatrix lastLocation = null;
      private boolean targetVisible = false;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
     
      private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        
          
        
        InitializeRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Heading", initialAngle);
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();

        

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        //if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
        //    if (tfod != null) {
        //        tfod.activate();
        //    }
        //}
        /** Start tracking the data sets we care about. */
        //targetsRoverRuckus.activate();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
             
            //LowerRobot(); //Lower Robot
            TouchMineral();  //Detect and Touch Mineral
            DriveToDepot();  //Drive to Depot
            DropMarker();    //Drop Marker and Park in Crater
             
            drivePower  = 0;
            break;
            
        }
        if (tfod != null) {
            tfod.shutdown();
        }
          
    }
    
     public void InitializeRobot() {
        
        //Initialize Robot Hardware
        robot.init(hardwareMap);
        robot.liftArm.setPosition(.5);
        robot.pivot.setPosition(.5);
        initialAngle = robot.GetHeading();
        initVuforia();
        
       //Initialize TensorFlow 
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
     }
     
     public void LowerRobot() {
        
        telemetry.addData("Lower Robot","");
        telemetry.update();
        
        robot.leftClaw.setPosition(1);
        robot.liftArm.setPosition(1);
        sleep(7200);
             
        robot.liftArm.setPosition(.5);
        robot.pivot.setPosition(0);
        robot.pivot.setPosition(1); 
             
        DriveTrain.EDrive(drivePower,0,0,0,-45);// unlatch from hook
        DriveTrain.EDrive(drivePower,2,2,0,0);  // Inch forward
        DriveTrain.EDrive(drivePower,0,0,0,45); // Face Minerals
     }
     
     public void DriveToDepot() {
             
        //DriveTrain.EDrive(drivePower,0,0,0,degreesToTarget); // Turn 45 degrees to align with image
        telemetry.addData("Drive to Depot","");
        telemetry.addData("Heading", robot.GetHeading() - initialAngle);
        telemetry.update();
        DriveTrain.EDrive(drivePower,distanceToTarget,distanceToTarget,0,0); // Drive to Target
        DriveTrain.EDrive(drivePower,0,0,0,degreesToDepot); // Right Turn
        //robot.liftArm.setPosition(0);
        DriveTrain.EDrive(drivePower,distanceToDepot,distanceToDepot,0,0); // Drive to Depot
        
     }
     
     public void DropMarker() {
         
      // Drop Marker
             
        telemetry.addData("Drop Marker","");
        telemetry.addData("Heading", robot.GetHeading() - initialAngle);
        telemetry.update();
             
        robot.pivot.setPosition(.5);
        robot.pivot.setPosition(0);              //Pivot Arm Initial Position
        sleep(500);
        robot.pivot.setPosition(.5);
        robot.leftClaw.setPosition(0);
        sleep(500);
        robot.leftClaw.setPosition(1);
            
             
     //Reverse to Crater
             
        telemetry.addData("Reverse to Crater","");
        telemetry.addData("Heading", robot.GetHeading() - initialAngle);
        telemetry.update();
             
        DriveTrain.EDrive(drivePower+.25,distanceToCrater,distanceToCrater,0,0);    
             
     }
     public void TouchMineral() {
            
            telemetry.addData("Touch Mineral","");
            telemetry.addData("Heading", robot.GetHeading() - initialAngle);
            telemetry.update();
             
             goldPosition = DetectMineral();
             
             
             switch(goldPosition) {
             
             case "Center":
             DriveTrain.EDrive(drivePower,distanceToMineral,distanceToMineral,0,0); // Drive to Mineral 
             DriveTrain.EDrive(drivePower,-distanceToMineral,-distanceToMineral,0,0); // Reverse from Mineral 
             break;
             case "Left":
             DriveTrain.EDrive(drivePower,0,0,0,-degreesToMineral); // Turn to Mineral 
             DriveTrain.EDrive(drivePower,distanceToMineral,distanceToMineral,0,0); // Drive to Mineral 
             DriveTrain.EDrive(drivePower,-distanceToMineral,-distanceToMineral,0,0); // Reverse from Mineral 
             DriveTrain.EDrive(drivePower,0,0,0,(degreesToTarget + degreesToMineral)); // Face Target 
             break;
             case "Right":
             DriveTrain.EDrive(drivePower,0,0,0,degreesToMineral); // Turn to Right Mineral 
             DriveTrain.EDrive(drivePower,distanceToMineral,distanceToMineral,0,0); // Drive to Mineral  
             DriveTrain.EDrive(drivePower,-distanceToMineral,-distanceToMineral,0,0); // Reverse from Mineral 
             DriveTrain.EDrive(drivePower,0,0,0,(degreesToTarget + degreesToMineral)); // Face Target
             break;
             default:
                 
             }
              
             telemetry.addData("Gold Position",goldPosition);
             telemetry.update();
     }
     
     
     private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.25;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    
     
     
     
     public String DetectMineral () {
      
      telemetry.addData("Detecting Mineral", "");
      telemetry.update();
      
      String GoldPosition= "None";
      /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
             double i = 0;
       while (i < detectAttempts) {     
         if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        i = detectAttempts;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            GoldPosition = "Left";
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            GoldPosition = "Right";
                          } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            GoldPosition = "Center";
                          }
                        }
                      }
                      telemetry.update();
                      i++;
                      //sleep(500);
                    }
                }
       }
                
                return GoldPosition;
     }

        
}
