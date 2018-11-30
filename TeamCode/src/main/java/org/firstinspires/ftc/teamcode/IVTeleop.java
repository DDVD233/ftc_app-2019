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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="IV Telop")
//@Disabled
public class IVTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    IVcapstonehardward   robot           = new IVcapstonehardward();              //
    double          LPosition     = robot.L_HOME;                   // Servo safe position
    double       RPosition    = robot.R_HOME;                  // Servo safe position
    double          BPosition     = robot.B_HOME;                   // Servo safe position
    double       TPosition    = robot.T_HOME;                  // Servo safe position
    final double    L_SPEED      = 0.05 ;                            // sets rate to move servo
    final double    R_SPEED      = 0.05 ;                            // sets rate to move servo
    final double    B_SPEED      = 0.05 ;                            // sets rate to move servo
    final double    T_SPEED      = 0.05 ;                            // sets rate to move servo

    //private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;

           // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

            robot.frontleft.setPower(left);
            robot.frontright.setPower(right);
            robot.backleft.setPower(left);
            robot.backright.setPower(right);



           // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                LPosition += L_SPEED;
            else if (gamepad1.y)
                LPosition -= L_SPEED;

        LPosition  = Range.clip(LPosition, robot.L_MIN_RANGE, robot.L_MAX_RANGE);
        robot.Lever.setPosition(LPosition);

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                RPosition += R_SPEED;
            else if (gamepad1.b)
                RPosition -= R_SPEED;

            RPosition  = Range.clip(RPosition, robot.R_MIN_RANGE, robot.R_MAX_RANGE);
            robot.RoArm.setPosition(RPosition);

            if (gamepad1.dpad_up){
                BPosition += B_SPEED;}
            if (gamepad1.dpad_down){
                BPosition -= B_SPEED;}

            BPosition  = Range.clip(BPosition, robot.B_MIN_RANGE, robot.B_MAX_RANGE);
            robot.Bottom.setPosition(BPosition);

            if (gamepad1.dpad_right)
                TPosition += T_SPEED;
            else if (gamepad1.dpad_left)
                TPosition -= T_SPEED;

            TPosition  = Range.clip(TPosition, robot.T_MIN_RANGE, robot.T_MAX_RANGE);
            robot.Top.setPosition(TPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("Lever",   "%.2f", LPosition);
            telemetry.addData("RoArm",  "%.2f", RPosition);
            telemetry.addData("Bottom",  "%.2f", BPosition);
            telemetry.addData("Top",  "%.2f", TPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
