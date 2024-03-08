/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;






import com.qualcomm.robotcore.hardware.DcMotor;








import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="2 person", group="Linear Opmode")

public class person2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;
    private DcMotor SUCC = null;
    private DcMotorEx lift = null;
    private DigitalChannel lim = null;
    private CRServo claw = null;
    private CRServo claw1 = null;
    private CRServo claw3 = null;
    private Servo claw4 = null;
    private Servo claw2 = null;
    private ColorRangeSensor sen = null;
    private DistanceSensor dist = null;
    private Gamepad.RumbleEffect rumble = null;
    private Gamepad.RumbleEffect.Builder steps = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LBD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBD");
        armMotor = hardwareMap.get(DcMotor.class, "AM");
        wristMotor = hardwareMap.get(DcMotor.class, "WM");
        SUCC = hardwareMap.get(DcMotor.class, "SU");
        sen = hardwareMap.get(ColorRangeSensor.class, "sen1");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lim = hardwareMap.get(DigitalChannel.class, "Lim");
        claw = hardwareMap.get(CRServo.class, "ser1");
        claw1 = hardwareMap.get(CRServo.class, "ser2");
        claw2 = hardwareMap.get(Servo.class, "ser3");
        claw3 = hardwareMap.get(CRServo.class, "ser4");
        claw4 = hardwareMap.get(Servo.class, "ser5");
        dist = hardwareMap.get(DistanceSensor.class, "dist");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        wristMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //claw1.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(CRServo.Direction.REVERSE);
        //armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //lift.setMode(DcMotor.RunMode.RESET_ENCODERS);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lim.setMode(DigitalChannel.Mode.INPUT);
        steps = new Gamepad.RumbleEffect.Builder();
        steps.addStep(0.3,0.3,100);
        rumble = steps.build();

        // Wait for the game to start (driver presses PLAY)
         telemetry.addData("Status", "Initialized");
        telemetry.update();
        int pos = -400;
        boolean flag = true;
        boolean lock = false;
        boolean scoreNeg = true;
        double Lval = 0;
        double Cval = 0;
        int tick = 1;
        boolean planeFlag = true;
        boolean planeLock = false;
        boolean hangFlag = false;
        double flagTime = 0.0;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;
            boolean Abut = gamepad2.a;
            boolean Bbut = gamepad1.right_bumper;
            boolean Sbut = gamepad2.square;
            boolean Cbut = gamepad2.cross;
            boolean Lbut = gamepad1.dpad_up;
            boolean Rbut = gamepad1.dpad_down;
            double Rtrig = gamepad2.right_trigger;
            double Ltrig = gamepad2.left_trigger;
            boolean modF = gamepad1.left_bumper;
            boolean scorePos = gamepad1.dpad_left;
            boolean wall = lim.getState();
            double mod = 1.0;
            double mod2 = 1.0;

            if (scorePos && scoreNeg) {
                tick = -tick;
            }

            scoreNeg = !scorePos;
            // Decrease mod2 in proportion to gamepad1.right_trigger
            mod2 -= gamepad1.right_trigger * 0.4; // You can adjust the multiplier as needed

            // Ensure mod2 does not go below 0
            mod2 = Math.max(mod2, 0.0);

            //gamepad2.left_bumper;
            boolean pick = gamepad2.right_bumper;
            double gate = gamepad2.right_trigger + 0.5;


            if (flag) {
                lock = gamepad2.left_bumper;
            }
            if(planeFlag){
                planeLock = gamepad2.dpad_up;
            }
            if (modF) {
                mod = 0.25;

            } else {
                mod = 1.0;
            }

            if (Lbut && runtime.seconds() > 10) {
                lift.setTargetPosition(-9100);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1.0);
            }

            if (Rbut && runtime.seconds() > 10) {
                lift.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1.0);
            }


            if(wall && pos < -2000){
                axial = Math.max(axial, 0);
                gamepad1.runRumbleEffect(rumble);
            }


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = -axial + lateral + yaw;
            double rightFrontPower = -axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            double armPower = gamepad2.left_stick_y * 0.5;
            double wristPower = gamepad2.right_stick_y * 0.4;
            double liftPower = Cval + Lval;
            //double pos = 0.0;
            double Apos = 0.0;
            boolean home = false;
            //pos = 0.2 * Rtrig;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels

            if (Abut) {
                //claw1.setPosition(0.5);
                //claw2.setPosition(0.5);
            } else {
                //claw1.setPosition(0.0);
                //claw2.setPosition(0.0);
            }
            if (Bbut) {
                claw.setPower(1.0);
                claw1.setPower(1.0);
                claw3.setPower(1.0);
                //claw4.setPower(1.0);
                SUCC.setPower(1.0);

            } else {
                if (gamepad1.b && !Bbut) {
                    SUCC.setPower(-1.0);
                    claw.setPower(0.0);
                    claw1.setPower(0.0);
                    claw3.setPower(0.0);
                    //claw4.setPower(0.0);
                } else {
                    claw.setPower(0.0);
                    claw1.setPower(0.0);
                    claw3.setPower(0.0);
                    //claw4.setPower(0.0);
                    SUCC.setPower(0.0);
                }
            }
            if (lock) {
                lock = true;
                flag = false;
                if (tick > 0) {
                    armMotor.setTargetPosition(-3700);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1.0);
                    wristMotor.setTargetPosition(150);
                    wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wristMotor.setPower(0.5);
                    if (armMotor.getCurrentPosition() < -3600) {
                        wristMotor.setTargetPosition(94);
                        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wristMotor.setPower(0.6);
                        lock = false;
                        flag = true;
                        pos = armMotor.getCurrentPosition();
                    }
                }else {
                        armMotor.setTargetPosition(-3200);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(1.0);
                        wristMotor.setTargetPosition(150);
                        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wristMotor.setPower(0.5);
                        if (armMotor.getCurrentPosition() < -3000) {
                            wristMotor.setTargetPosition(94);
                            wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            wristMotor.setPower(0.6);
                            lock = false;
                            flag = true;
                            pos = armMotor.getCurrentPosition();
                        }
                    }



            }
            if (Sbut) {
                armMotor.setTargetPosition(50);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1.0);
                wristMotor.setTargetPosition(500);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.3);
                pos = armMotor.getCurrentPosition();
                if (armMotor.getCurrentPosition() < 100) {
                    wristMotor.setTargetPosition(300);
                    wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wristMotor.setPower(0.4);
                }
            }


            if (Abut) {
                armMotor.setTargetPosition(-400);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1.0);
                wristMotor.setTargetPosition(200);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.5);
                pos = armMotor.getCurrentPosition();

            }

            if (planeLock && !lock && runtime.seconds() > 10) {
                planeLock = true;
                planeFlag = false;
                armMotor.setTargetPosition(-1300);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1.0);
                lift.setTargetPosition(-9100);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1.0);
                pos = armMotor.getCurrentPosition();
                if (armMotor.getCurrentPosition() < -1200 ) {
                    planeLock = false;
                    planeFlag = true;
                    pos = armMotor.getCurrentPosition();
                }


            }

            if(runtime.seconds() > 5){
                claw4.setPosition(gamepad2.left_trigger);
                if( claw4.getPosition() > 0.90 && !hangFlag){
                    hangFlag = true;
                    flagTime = runtime.seconds();
                }
            }

            if(hangFlag && runtime.seconds() - flagTime >= 2){
                armMotor.setTargetPosition(-300);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.6);
                pos = armMotor.getCurrentPosition();
            }


            if (pick) {

                pos = armMotor.getCurrentPosition();

            }
            if (!pick && !lock && !Sbut && !Abut && flag && !planeLock && planeFlag && !hangFlag) {
                armMotor.setTargetPosition(pos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.45);

            }

            if (!armMotor.isBusy() && pick) {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor.setPower(armPower);
            }
            if (!wristMotor.isBusy() && !lock) {
                wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wristMotor.setPower(wristPower);
            }


            if (dist.getDistance(DistanceUnit.CM) < 40) {
                // Decrease mod in proportion to dist
                mod = dist.getDistance(DistanceUnit.CM) * 0.03; // You can adjust the multiplier as needed


            }
            if (armMotor.getCurrentPosition() < -2000){
                mod2 = 0.35;
            }



            leftFrontDrive.setPower(leftFrontPower * mod * mod2);
            rightFrontDrive.setPower(rightFrontPower * mod * mod2);
            leftBackDrive.setPower(leftBackPower * mod * mod2);
            rightBackDrive.setPower(rightBackPower * mod * mod2);

            //lift.setPower(liftPower);

            claw2.setPosition(gate);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * mod * mod2, rightFrontPower * mod * mod2);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * mod * mod2, rightBackPower * mod * mod2);
            telemetry.addData("tick", tick);
            telemetry.addData("wristMotor pos", wristMotor.getCurrentPosition());
            telemetry.addData("armMotor tar", armMotor.getTargetPosition());
            telemetry.addData("lift pos", lift.getMode());
            telemetry.addData("lift power", liftPower);
            telemetry.addData("mod", mod);
            telemetry.addData("mod2", mod2);
            telemetry.addData("armDist", dist.getDistance(DistanceUnit.CM));
            telemetry.addData("dist2", sen.getDistance(DistanceUnit.CM));
            telemetry.addData("lim", wall);
            telemetry.addData("lock", lock);
            telemetry.addData("planeLock", planeLock);

            telemetry.update();

        }
    }

}