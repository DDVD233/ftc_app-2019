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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Mecanum Auto Blue")
@Disabled
public class MecanumAutoBlue extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum1 robot   = new Mecanum1();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.arm.setPosition(0.0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


//        robot.LClaw.setPosition(0.5);
//        robot.RClaw.setPosition(0.5);
//        robot.LHand.setPosition(0.9);
//        robot.RHand.setPosition(0.9);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.30)) {
//            telemetry.addData("Path", "Move hand down: %2.5f S Elapsed", runtime.seconds());
//            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
//            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
//            telemetry.update();
//        }
//
//        robot.LHand.setPosition(0.8);
//        robot.RHand.setPosition(0.8);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.30)) {
//            telemetry.addData("Path", "Move Claw in: %2.5f S Elapsed", runtime.seconds());
//            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
//            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
//            telemetry.update();
//        }
//
//        robot.tiltMotor.setPower(-1.0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            telemetry.addData("Path", "Move LiftMotor up: %2.5f S Elapsed", runtime.seconds());
//            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
//            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
//            telemetry.update();
//        }

        robot.tiltMotor.setPower(0.0);
        robot.LFMotor.setPower(0.3);
        robot.LRMotor.setPower(-0.3);
        robot.RFMotor.setPower(-0.3);
        robot.RRMotor.setPower(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.20)) {
            telemetry.addData("Path", "Move to the right: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            telemetry.update();
        }

        robot.arm.setPosition(0.95);
        robot.LFMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.RRMotor.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Move arm: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            telemetry.update();
        }

        boolean isRed = (robot.armColorSensor.red() > robot.armColorSensor.blue());
        if (isRed) {
            // Move forward
            robot.LRMotor.setPower(1.0);
            robot.RRMotor.setPower(1.0);
            robot.LFMotor.setPower(1.0);
            robot.RFMotor.setPower(1.0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Kick Ball: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
                telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
                telemetry.update();
            }
        } else {
            // Move back
            robot.LRMotor.setPower(-1.0);
            robot.RRMotor.setPower(-1.0);
            robot.LFMotor.setPower(-1.0);
            robot.RFMotor.setPower(-1.0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Kick Ball: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
                telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
                telemetry.update();
            }
        }

        robot.arm.setPosition(0.0);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "arm up: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            telemetry.update();
        }

        robot.LRMotor.setPower(-1.0);
        robot.RRMotor.setPower(-1.0);
        robot.LFMotor.setPower(-1.0);
        robot.RFMotor.setPower(-1.0);
        runtime.reset();
        double extraTime = isRed ? 0.5 : -0.5;
        while (opModeIsActive() && (runtime.seconds() < (1.0 + extraTime))) {
            telemetry.addData("Path", "Kick Ball: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            telemetry.update();
        }

        robot.LFMotor.setPower(-0.8);
        robot.LRMotor.setPower(0.8);
        robot.RFMotor.setPower(0.8);
        robot.RRMotor.setPower(-0.8);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Move left: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.arm.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
