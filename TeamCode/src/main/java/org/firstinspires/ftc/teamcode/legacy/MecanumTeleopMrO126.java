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

package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumTelopMrO126")
//@Disabled
public class MecanumTeleopMrO126 extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum1   robot           = new Mecanum1();              //
    double          armPosition     = Mecanum1.ARM_HOME;                   // Servo safe position
    double          clawPosition    = Mecanum1.CLAW_HOME;                  // Servo safe position
    double          handPosition    = Mecanum1.HAND_HOME;                  // Servo safe position

    final double    CLAW_SPEED      = 0.05 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.05  ;
    final double    HAND_SPEED      = 0.1; // sets rate to move servo
    double          lift            = 0.2;
    //private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double LFspeed;
        double RFspeed;
        double LRspeed;
        double RRspeed;
        double liftup;
        double liftdown;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQRju9v/////AAAAGYVVWUuAKEAPph69Ouwpq+8CMlbZi/zomEEP" +
                "MAzyzRvCU9xj4/W+fiPSxFB1xv8BdlL55c6vD9wbMdHCPpMKKIqrNIJpXw06LgCF8xDeBXJEOEo" +
                "oeyamPY8gLwHvkHDA0EEP52F+b7J1IDRbmJedlK6GdOIYFLiSBVISCf0+vdiWJvJiWiPTgggSiC" +
                "RsV8IsK/OYwOqv5VoPv/m7no+VACFqPTcsKaAv5F49zqYXIncFbNKv9onHg5CEkZ4aMf7D/zcAp" +
                "hCO5Gb3BN+DtyUmrHM4oALhoMFgqRw59plwzxfD45Uzfyu6Jn2k7LPTCqs94SpprMfOqOotLtBHx" +
                "T19Rkl5toHI5buxqLlQDOX8y/jQ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use back camera
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();

        relicTrackables.activate();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            LFspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            LRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
            RFspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            RRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;

            LFspeed = Range.clip(LFspeed, -1, 1);
            LRspeed = Range.clip(LRspeed, -1, 1);
            RFspeed = Range.clip(RFspeed, -1, 1);
            RRspeed = Range.clip(RRspeed, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            LFspeed = (float)scaleInput(LFspeed);
            LRspeed =  (float)scaleInput(LRspeed);
            RFspeed = (float)scaleInput(RFspeed);
            RRspeed =  (float)scaleInput(RRspeed);

            robot.LFMotor.setPower(LFspeed);
            robot.RFMotor.setPower(RFspeed);
            robot.LRMotor.setPower(LRspeed);
            robot.RRMotor.setPower(RRspeed);

            liftup = gamepad2.left_trigger;
            liftdown = gamepad2.right_trigger;

//            Use gamepad left & right trigger raise and lower the lift motor.
            // az
            robot.tiltMotor.setPower(liftup-liftdown);



            // Use gamepad x & b move hands in and out
            if (gamepad2.x)
                handPosition += HAND_SPEED;
            else if (gamepad2.b)
                handPosition -= HAND_SPEED;

            handPosition = Range.clip(handPosition, Mecanum1.HAND_MIN_RANGE, Mecanum1.HAND_MAX_RANGE);
            robot.LHand.setPosition(handPosition);
            robot.RHand.setPosition(handPosition);

            // MrO code Use gamepad right bump & left bump to raise and lower the arm
            if (gamepad2.right_bumper)
                 armPosition += ARM_SPEED;
            else if (gamepad2.left_bumper)
                armPosition -= ARM_SPEED;

            armPosition  = Range.clip(armPosition, Mecanum1.ARM_MIN_RANGE, Mecanum1.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);

            // Use gamepad a & y move claw up and down
            if (gamepad2.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad2.y)
                clawPosition -= CLAW_SPEED;

            clawPosition = Range.clip(clawPosition, Mecanum1.CLAW_MIN_RANGE, Mecanum1.CLAW_MAX_RANGE);
            robot.RClaw.setPosition(clawPosition);
            robot.LClaw.setPosition(clawPosition);


            if (gamepad2.dpad_down)
                robot.liftMotor.setPower(-0.5);
            else if (gamepad2.dpad_up)
                robot.liftMotor.setPower(0.5);
            else
                robot.liftMotor.setPower(0.0);

            // Move both servos to new position. MrO Added Hand  code




            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("RClaw",  "%.2f", clawPosition);
            telemetry.addData("LClaw",  "%.2f", clawPosition);
            telemetry.addData("Rhand",  "%.2f", handPosition);
            telemetry.addData("Lhand",  "%.2f", handPosition);

            telemetry.addData("LFMotor",   "%03d", robot.LFMotor.getCurrentPosition());
            telemetry.addData("LRMotor",  "%03d", robot.LRMotor.getCurrentPosition());
            telemetry.addData("RFMotor",  "%03d", robot.RFMotor.getCurrentPosition());
            telemetry.addData("RRMotor",  "%03d", robot.RRMotor.getCurrentPosition());
//            telemetry.addData("leftClawDistance",  "%.2f", robot.leftClawDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("leftClawBlue",  "%03d", robot.leftClawColorSensor.blue());
//            telemetry.addData("rightClawRed",  "%03d", robot.rightClawColorSensor.red());
//            telemetry.addData("rightClawDistance",  "%.2f", robot.rightClawDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("rightClawBlue",  "%03d", robot.rightClawColorSensor.blue());

            telemetry.update();
            // Pause for 20 mS each cycle = update 50 times a second.
            sleep(20);

        }
    }

    /*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
