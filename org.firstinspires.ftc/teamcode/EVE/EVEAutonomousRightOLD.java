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

package org.firstinspires.ftc.teamcode.EVE;
//
//import org.firstinspires.ftc.teamcode.EVE.*;

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

@Autonomous(name="EVEAutonomousRight Java", group="Linear Opmode")

public class EVEAutonomousRightOLD extends LinearOpMode {

    // Declare OpMode members.
      private ElapsedTime runtime = new ElapsedTime();
    
      private  double leftDistance  = 30;  // Hardcoded Distance when not using Vuforia
      private  double rightDistance = 30;
      private double drivePower = .5;      //Default Motor Power
      
      
      private EVEHardware robot   = new EVEHardware(); //Create Robot Instance
      private EVEDriveTrain DriveTrain = new EVEDriveTrain(robot); //Create DriveTrain Instance passing Robot
      
      
      private ColorSensor sensorColor;
      private DistanceSensor sensorDistance;
      
      //Location Variables in Inches and Degrees
      private double distanceToDepot = 44;
      private double distanceToCrater = -80;
      private double distanceToMineral = 18;
      private double distanceToTarget = 35;
      private double degreesToTarget = 22.5;
      private double degreesToDepot = 40;
      private double degreesToMineral = 30;
      private String goldPosition = "None";
      
      private BNO055IMU imu;
      private BNO055IMU.Parameters imuParameters;
      private Orientation angles;
      private Acceleration gravity;
      
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
    VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
     
      private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        
          
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;
        
      
        //Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
       
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);
        
        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 75;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 100;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        
        
        InitializeRobot();
        telemetry.addData("Status", "Initialized");
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();

        

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }
        /** Start tracking the data sets we care about. */
        //targetsRoverRuckus.activate();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
             
            sleep(1000);
             
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
        robot.pivot.setPosition(.5);
        
        
        //Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");  
       // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
       // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
       // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       // Disable logging.
        imuParameters.loggingEnabled = false;
       // Initialize IMU.
        imu.initialize(imuParameters);
        
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
        robot.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArm.setTargetPosition(6000);
        robot.liftArm.setPower(.5);
        while (robot.liftArm.isBusy()) {

                // Loop until the robot reaches the designated distance
                
        }  
        sleep(7200);
             
        robot.liftArm.setPower(0);
        //EVEDriveTrain DriveTrain = new EVEDriveTrain(robot) ;
             
        robot.pivot.setPosition(0);
        robot.pivot.setPosition(1); 
        
        DriveTrain.EDrive(drivePower,0,0,0,-22.5);// unlatch from hook
        DriveTrain.EDrive(drivePower,2,2,0,0);  // Inch forward
        DriveTrain.EDrive(drivePower,0,0,0,22.5); // Face Minerals
     }
     
     public void DriveToDepot() {
             
        telemetry.addData("Drive to Depot","");
        telemetry.update();
             
        DriveTrain.EDrive(drivePower,0,0,0,degreesToTarget); // Turn 45 degrees to align with image
        DriveTrain.EDrive(drivePower,distanceToTarget,distanceToTarget,0,0); // Drive to Target
        DriveTrain.EDrive(drivePower,0,0,0,degreesToDepot); // Right Turn
        //robot.liftArm.setPosition(0);
        DriveTrain.EDrive(drivePower,distanceToDepot,distanceToDepot,0,0); // Drive to Depot
        
     }
     
     public void DropMarker() {
         
      // Drop Marker
             
        telemetry.addData("Drop Marker","");
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
        telemetry.update();
             
        DriveTrain.EDrive(drivePower,distanceToCrater,distanceToCrater,0,0);    
             
     }
     public void TouchMineral() {
        
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
             DriveTrain.EDrive(drivePower,0,0,0,degreesToMineral); // Face Minerals 
             break;
             case "Right":
             DriveTrain.EDrive(drivePower,0,0,0,degreesToMineral); // Turn to Right Mineral 
             DriveTrain.EDrive(drivePower,distanceToMineral,distanceToMineral,0,0); // Drive to Mineral  
             DriveTrain.EDrive(drivePower,-distanceToMineral,-distanceToMineral,0,0); // Reverse from Mineral 
             DriveTrain.EDrive(drivePower,0,0,0,-degreesToMineral); // Face Minerals
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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    
     public double GetPosition() {
        
      // Get absolute orientation
      // Get acceleration due to force of gravity.
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      gravity = imu.getGravity();
      // Display orientation info.
      telemetry.addData("rot about Z", angles.firstAngle);
      telemetry.addData("rot about Y", angles.secondAngle);
      telemetry.addData("rot about X", angles.thirdAngle);
      // Display gravitational acceleration.
      telemetry.addData("gravity (Z)", gravity.zAccel);
      telemetry.addData("gravity (Y)", gravity.yAccel);
      telemetry.addData("gravity (X)", gravity.xAccel);
      telemetry.update();
      return  angles.firstAngle;
     }
     
     
     public String DetectMineral () {
      
      telemetry.addData("Detecting Mineral", "");
      telemetry.update();
      
      String GoldPosition= "None";
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
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            telemetry.addData("Gold Mineral Detected","");
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
                    }
                }
                return GoldPosition;
     }

        
}
