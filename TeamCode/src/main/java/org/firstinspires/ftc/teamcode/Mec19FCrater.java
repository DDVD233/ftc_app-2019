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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.legacy.Direction;


/**
 * This is the autonomous mode for the robot Mecanum19.
 * Run this program if the robot is facing the crater at the start of the match.
 * It is still under work.
 */

@Autonomous(name="Mec19FCrater")
public class Mec19FCrater extends LinearOpMode {

    /* Declare OpMode members. */
    //private ElapsedTime runtime = new ElapsedTime();

    // OpenGLMatrix lastLocation = null;
    // int a

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        Mecanum19Drive mecanumDrive = new Mecanum19Drive(this);

        mecanumDrive.setWheelPower(0,0,0,0);
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        mecanumDrive.initDetector();

        waitForStart();

        if (mecanumDrive.robot.digitalSwitch.getState()) {

            mecanumDrive.robot.liftM.setPower(1);
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            mecanumDrive.robot.liftM.setPower(0);
            telemetry.addData("Digital Touch", "Is Pressed");
        }
      //  mecanumDrive.robot.liftM.setPower(1);

        //mecanumDrive.waitFor(12.8, "Releasing robot");

        mecanumDrive.robot.liftM.setPower(0);

        mecanumDrive.encoderDriveMove(1, Direction.RIGHT, 10, 3);
        mecanumDrive.encoderDriveMove(1, Direction.FORWARD, 5, 3);
        mecanumDrive.encoderDriveMove(1, Direction.LEFT, 10, 3);

        mecanumDrive.kickGoldCube();

        mecanumDrive.encoderDriveMove(1.0, Direction.FORWARD, 20, 5); // Move forward and kick the ball
    }
}
