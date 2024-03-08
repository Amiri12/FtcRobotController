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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="FarBlue_smart", group="Robot")


public class LeftFar extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LeftFrontDrive   = null;
    private DcMotor         RightBackDrive  = null;
    private DcMotor         RightFrontDrive   = null;
    private DcMotor         LeftBackDrive  = null;
    private Servo DropperL = null;
    private ColorRangeSensor senL = null;
    private Servo DropperR = null;
    private Servo DropperM = null;
    private ColorRangeSensor senR = null;
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     THRESH = 20;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftFrontDrive  = hardwareMap.get(DcMotor.class, "LFD");
        LeftBackDrive = hardwareMap.get(DcMotor.class, "LBD");
        RightFrontDrive  = hardwareMap.get(DcMotor.class, "RFD");
        RightBackDrive = hardwareMap.get(DcMotor.class, "RBD");
        DropperR = hardwareMap.get(Servo.class, "DR");
        DropperL = hardwareMap.get(Servo.class, "DR1");
        DropperM = hardwareMap.get(Servo.class, "ser3");
        senR = hardwareMap.get(ColorRangeSensor.class, "sen");
        senL = hardwareMap.get(ColorRangeSensor.class, "sen1");
        armMotor = hardwareMap.get(DcMotor.class, "AM");
        wristMotor = hardwareMap.get(DcMotor.class, "WM");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        LeftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        DropperR.setDirection(Servo.Direction.REVERSE);



        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        wristMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bobot bot = new Bobot(LeftFrontDrive,RightFrontDrive,LeftBackDrive,RightBackDrive);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                LeftFrontDrive.getCurrentPosition(),
                RightBackDrive.getCurrentPosition());
                telemetry.addData("dist", senL.getDistance(DistanceUnit.CM));
        telemetry.update();
        DropperR.setPosition(0.5);
        DropperL.setPosition(0.5);
        DropperM.setPosition(0.5);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        bot.forward(800, 0.2);
        int sqr = 1000;
        sleep(500);
        boolean CRF = true;
        boolean CLF = true;
        boolean DTF = true;
        boolean DRF = true;
        boolean DLF = true;
        /*
        driveForward(750);
        sleep (500);
        driveRight(sqr);
        sleep (500);
        driveBack(750);
        sleep (500);
        driveLeft(sqr);
        */

        int tar = LeftFrontDrive.getCurrentPosition() + 1100;
        int pos = LeftFrontDrive.getCurrentPosition();
        int star = LeftFrontDrive.getCurrentPosition();
        int state = 0;
        telemetry.addData("Path", "Complete");
        telemetry.addData("pos", LeftFrontDrive.getCurrentPosition());
        telemetry.addData("CLF",CLF);
        telemetry.addData("DTF",DTF);
        telemetry.addData("CRF",CRF);
        telemetry.addData("DLF",DLF);
        telemetry.addData("DRF",DRF);
        telemetry.addData("state", state);
        telemetry.addData("senR", Double.isNaN(senR.getDistance(DistanceUnit.CM)));

        telemetry.update();
        sleep(1000);
        while(DLF && DTF && CRF && DRF && CLF){
            bot.forward(0.3);
            pos = LeftFrontDrive.getCurrentPosition();
            telemetry.addData("redR", senR.red());
            telemetry.addData("redL", senL.red());
            telemetry.addData("green", senL.green());
            telemetry.addData("pos", pos);
            telemetry.addData("distR",senR.getDistance(DistanceUnit.CM));
            telemetry.addData("distL",senL.getDistance(DistanceUnit.CM));
            telemetry.update();
            //CLF = !(senL.red() > senL.green() && senL.red() > 20);
            //CRF = !(senR.red() > senR.green() && senL.red() > 20);
            DLF = !isClose(senL.getDistance(DistanceUnit.CM), THRESH);
            DRF = !isClose(senR.getDistance(DistanceUnit.CM), THRESH);
            DTF = pos < tar;
        }

        bot.stop();
            telemetry.addData("red", senR.red());
            telemetry.addData("blue", senL.red());
            telemetry.addData("green", senL.green());
            telemetry.addData("pos", pos);
            telemetry.addData("distR",senR.getDistance(DistanceUnit.CM));
            telemetry.addData("distL",senL.getDistance(DistanceUnit.CM));
            telemetry.update();
        //DropperR.setPosition(0.75);
        //DropperL.setPosition(0.75);
        sleep(1000);
        //bot.backwards(pos - star, 0.3);
        if(!CRF){
            state = 1;
        }
        if(!DTF){
            state = 2;
        }
        if(!CLF){
            state = 3;
        }
        if(!DRF){
            state = 1;
        }
        if(!DLF){
            state = 3;
        }


        int Power = 1;
      switch(state){
          case 1:
              bot.right(1100, 0.3);
              DropperR.setPosition(0.75);
              DropperL.setPosition(0.75);
              sleep(500);
              bot.right(500, 0.3);
              bot.forward(500, 0.3);
              bot.left(3900, 0.3);
              bot.rightTurn(1100, 0.3);
              bot.right(700, 0.3);
              armMotor.setTargetPosition(-3700);
              armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armMotor.setPower(1.0);
              sleep(1500);
              bot.backwards(190, 0.15);
              DropperM.setPosition(1.0);
              sleep(700);
               armMotor.setTargetPosition(-100);
              armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armMotor.setPower(.5);
              sleep(500);
              bot.left(700,0.2);
              bot.backwards(900, 0.15);
              break;
          case 2:
              bot.right(350, 0.3);
              //bot.forward(300, 0.3);
              DropperR.setPosition(0.75);
              DropperL.setPosition(0.75);
              sleep(500);
              bot.right(500, 0.3);
              bot.forward(500, 0.3);
              bot.left(2000, 0.3);
              bot.rightTurn(1100, 0.3);
              bot.right(700, 0.3);
              armMotor.setTargetPosition(-3000);
              armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armMotor.setPower(1.0);
              sleep(1500);
              bot.right(190, 0.15);
              DropperM.setPosition(1.0);
              sleep(700);
              armMotor.setTargetPosition(-100);
              armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armMotor.setPower(.5);
              break;
          case 3:
          bot.left(100, 0.3);
          DropperR.setPosition(0.75);
          DropperL.setPosition(0.75);
          sleep(500);
          bot.right(300, 0.3);
          bot.forward(1400, 0.3);
          bot.left(4000,0.3);
          bot.rightTurn(1100, 0.3);
          bot.right(2400, 0.3);
          armMotor.setTargetPosition(-3700);
          armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armMotor.setPower(1.0);
          sleep(1500);
          bot.left(190, 0.15);
          DropperM.setPosition(1.0);
          sleep(700);
           armMotor.setTargetPosition(-100);
          armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armMotor.setPower(.5);
          sleep(1500);
          bot.left(1450,0.2);
          bot.backwards(900, 0.15);
          break;
          default:
          bot.backwards(790, 0.3);
          driveForward(2000);
          sleep (500);
          driveLeft(4600);
          sleep(500);
      }

        double mats = 2;
        double dis = 23.5 * mats;
        //encoderDrive(Power, dis, dis, 50000);





        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.addData("pos", LeftFrontDrive.getCurrentPosition());
        telemetry.addData("CLF",CLF);
        telemetry.addData("DTF",DTF);
        telemetry.addData("CRF",CRF);
        telemetry.addData("DLF",DLF);
        telemetry.addData("DRF",DRF);
        telemetry.addData("state", state);
        telemetry.addData("senR", Double.isNaN(senR.getDistance(DistanceUnit.CM)));

        telemetry.update();
        sleep(30000);
// pause to display final telemetry message.
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
    public boolean isClose(double dist, double thresh){
        if(Double.isNaN(dist)){
            return false;
        }else{
            return dist < thresh;
        }

    }




    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LeftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LeftBackDrive.setTargetPosition(newLeftTarget);
            RightFrontDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBackDrive.setPower(Math.abs(speed));
            RightFrontDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftBackDrive.isBusy() && RightFrontDrive.isBusy())) {

                if (LeftBackDrive.isBusy() && RightFrontDrive.isBusy()){

            LeftFrontDrive.setPower(1.0);
            RightBackDrive.setPower(1.0);

        }
        else {
            LeftFrontDrive.setPower(0);
            RightBackDrive.setPower(0);
        }


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        LeftBackDrive.getCurrentPosition(), RightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftBackDrive.setPower(0);
            RightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   // optional pause after each move.
        }
    }
}
