/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import android.util.Size;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Far", group = "Concept")

public class CamTestHard2 extends LinearOpMode {
    private DcMotor         LeftFrontDrive   = null;
    private DcMotor         RightBackDrive  = null;
    private DcMotor         RightFrontDrive   = null;
    private DcMotor         LeftBackDrive  = null;
    private Servo Dropper = null;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "model_unquant.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedBlue.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "blue","red"
    };
    private ElapsedTime rat = new ElapsedTime();
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        int color = 1;

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        int state = 0;
        LeftFrontDrive  = hardwareMap.get(DcMotor.class, "LFD");
        LeftBackDrive = hardwareMap.get(DcMotor.class, "LBD");
        RightFrontDrive  = hardwareMap.get(DcMotor.class, "RFD");
        RightBackDrive = hardwareMap.get(DcMotor.class, "RBD");
        Dropper = hardwareMap.get(Servo.class, "ser3");
        if(gamepad1.dpad_up){
            color = 1;
        }else{
            color = 2;
        }
        waitForStart();
        

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        LeftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        Dropper.setDirection(Servo.Direction.REVERSE);

        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
      
        LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bobot bot = new Bobot(LeftFrontDrive,RightFrontDrive,LeftBackDrive,RightBackDrive);
        int spot = 0;
        
        String Dcolor = null;
        boolean Mflag = false;

        if (opModeIsActive()) {
            rat.reset();
            while (opModeIsActive()) {

                //telemetryTfod();

                // Push telemetry to the Driver Station.
                //telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                if(rat.seconds() < 4){
                    double x = 0.0;
                List<Recognition> Curent = tfod.getRecognitions();
                int i = 0;
                if(Curent.size()>0){
                for (Recognition recognition : Curent) {
                    if(recognition.getRight() - recognition.getLeft() < 150){
                        spot = i;
                    }
                    i++;
                }
                }
                
                
                    if(Curent.size()>0){
                    x = (Curent.get(spot).getLeft() + Curent.get(spot).getRight()) / 2 ;
                    if(x > 200){
                    state = 1;
                    //telemetry.addData("1", x);
                    //telemetry.update();
                    }else{
                    state = 2;
                    //telemetry.addData("2", x);
                    //telemetry.update();
                }

                
                if(Curent.get(spot).getRight() - Curent.get(spot).getLeft() > 150){
                    state = 3;
                }
                    }else{
                        state = 3;
                    }
                // Share the CPU.
                telemetry.addData("STATE", state);
                telemetry.addData("POS", x);
                telemetry.addData("TIME", rat.seconds());
                telemetry.addData("object", spot);
                telemetry.addData("color", color);
                telemetryTfod();
                telemetry.update();
                sleep(20);
                }else{
                    if(color == 1){
                    if(state == 2){
                    //driveForward(2000);
                    driveForward(1200);
                    sleep(1200);
                    Dropper.setPosition(0.75);
                    sleep(300);
                    driveForward(400);
                    sleep(500);
                    driveBack(400);
                    sleep(500);
                    Dropper.setPosition(0.5);
                    driveLeft(900);
                    sleep(800);
                    driveForward(1300);
                    sleep(800);
                    driveRight(5500);
                    sleep(30000);
                    }
                    if(state == 1){
                    driveForward(1000);
                    sleep(300);
                        turnRight(950);
                        sleep(500);
                        driveBack(350);
                        sleep(150);
                        Dropper.setPosition(0.75);
                        sleep(400);
                        driveForward(600);
                        sleep(200);
                        Dropper.setPosition(0.5);
                        driveBack(700);
                        sleep(300);
                        driveLeft(1700);
                        sleep(300);
                        driveForward(3600);
                        sleep(30000);
                    }
                    if(state == 3){
                        driveForward(1000);
                    sleep(300);
                    driveLeft(800);
                    sleep(400);
                    Dropper.setPosition(0.75);
                    sleep(200);
                    driveForward(200);
                    sleep(200);
                    Dropper.setPosition(0.5);
                    sleep(300);
                    driveBack(500);
                    sleep(200);
                    driveRight(600);
                    sleep(300);
                    driveForward(1500);
                    sleep(300);
                    driveRight(5000);
                    sleep(30000);
                    }
                    }if(color == 2){
                        if(state == 2){
                    //driveForward(2000);
                    driveForward(1200);
                    sleep(1200);
                    Dropper.setPosition(0.75);
                    sleep(300);
                    driveForward(400);
                    sleep(500);
                    driveBack(400);
                    sleep(500);
                    Dropper.setPosition(0.5);
                    driveRight(900);
                    sleep(800);
                    driveForward(1300);
                    sleep(800);sleep(30000);
                    driveLeft(5500);
                    
                    }
                    if(state == 1){
                    driveForward(1000);
                    sleep(300);
                    driveRight(800);
                    sleep(400);
                    Dropper.setPosition(0.75);
                    sleep(200);
                    driveForward(200);
                    sleep(200);
                    Dropper.setPosition(0.5);
                    sleep(300);
                    driveBack(500);
                    sleep(200);
                    driveLeft(600);
                    sleep(300);
                    driveForward(1500);
                    sleep(300);sleep(30000);
                    driveLeft(5000);
                    
                    }
                    if(state == 3){
                    driveForward(1000);
                    sleep(300);
                        turnLeft(950);
                        sleep(500);
                        driveBack(350);
                        sleep(150);
                        Dropper.setPosition(0.75);
                        sleep(400);
                        driveForward(600);
                        sleep(200);
                        Dropper.setPosition(0.5);
                        driveBack(700);
                        sleep(300);
                        driveRight(1700);
                        sleep(300);sleep(30000);
                        driveForward(3600);
                        
                    }
                    
                    }
                }
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }
    public void driveForward(int driveTime) {
        
        
        LeftFrontDrive.setPower(0.5);
        LeftBackDrive.setPower(0.5);
        RightFrontDrive.setPower(0.5);
        RightBackDrive.setPower(0.5);
        sleep(driveTime);
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);

    }
    public void driveBack(int driveTime) {

        LeftFrontDrive.setPower(-0.5);
        LeftBackDrive.setPower(-0.5);
        RightFrontDrive.setPower(-0.5);
        RightBackDrive.setPower(-0.5);
        sleep(driveTime);
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);

    }

    public void driveRight(int driveTime) {

        LeftFrontDrive.setPower(0.5);
        LeftBackDrive.setPower(-0.5);
        RightFrontDrive.setPower(-0.5);
        RightBackDrive.setPower(0.5);
        sleep(driveTime);
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);

    }

    public void driveLeft(int driveTime) {

        LeftFrontDrive.setPower(-0.5);
        LeftBackDrive.setPower(0.5);
        RightFrontDrive.setPower(0.5);
        RightBackDrive.setPower(-0.5);
        sleep(driveTime);
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);

    }
    public void turnRight(int driveTime) {

        LeftFrontDrive.setPower(0.5);
        LeftBackDrive.setPower(0.5);
        RightFrontDrive.setPower(-0.5);
        RightBackDrive.setPower(-0.5);
        sleep(driveTime);
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);

    }
    public void turnLeft(int driveTime) {

        LeftFrontDrive.setPower(-0.5);
        LeftBackDrive.setPower(-0.5);
        RightFrontDrive.setPower(0.5);
        RightBackDrive.setPower(0.5);
        sleep(driveTime);
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);

    }
    // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
//setCamera 
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(320, 240));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.80f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
    
    
    // end method telemetryTfod()

}
// end class
