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


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class IVcapstonehardward {
    /* Public OpMode members. */
    public DcMotor  frontleft   = null;
    public DcMotor  frontright  = null;
    public DcMotor  backleft = null;
    public DcMotor  backright  = null;
    public Servo    Lever        = null;
    public Servo    RoArm      = null;
    public Servo    Bottom       = null;
    public Servo    Top     = null;

    public final static double L_HOME = 0.5; // 0 -> up; 0.7 -> Right
    public final static double R_HOME = 0.5;
    public final static double B_HOME = 0.5; // 0 -> up; 0.7 -> Right
    public final static double T_HOME = 0.5;


    public final static double L_MIN_RANGE  = 0.00;
    public final static double L_MAX_RANGE  = 0.70;
    public final static double R_MIN_RANGE  = 0.30;
    public final static double R_MAX_RANGE  = 0.7;
    public final static double B_MIN_RANGE  = 0.00;
    public final static double B_MAX_RANGE  = 0.70;
    public final static double T_MIN_RANGE  = 0.30;
    public final static double T_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public IVcapstonehardward() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontleft  = hwMap.get(DcMotor.class, "Fleft_drive");
        frontright = hwMap.get(DcMotor.class, "Fright_drive");
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft  = hwMap.get(DcMotor.class, "Bleft_drive");
        backright = hwMap.get(DcMotor.class, "Bright_drive");
        backleft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        Lever  = hwMap.get(Servo.class, "Lever");
        RoArm = hwMap.get(Servo.class, "RoArm");
        Bottom = hwMap.get(Servo.class, "Bottom");
        Top = hwMap.get(Servo.class, "Top");

        Lever.setPosition(L_HOME);
        RoArm.setPosition(R_HOME);
        Bottom.setPosition(B_HOME);
        Top.setPosition(T_HOME);
    }
}
