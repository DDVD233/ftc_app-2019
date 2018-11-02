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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
public class Mecanum1 {
    /* Public OpMode members. */
    public DcMotor LFMotor    = null;
    public DcMotor RFMotor    = null;
    public DcMotor LRMotor    = null;
    public DcMotor RRMotor    = null;
    public DcMotor tiltMotor  = null;
    public DcMotor liftMotor  = null;
    public Servo arm          = null;
    public Servo LClaw        = null;
    public Servo RClaw        = null;
    public Servo LHand        = null;
    public Servo RHand        = null;
    //public Servo radarArm     = null;
    public ColorSensor armColorSensor = null;
    public DistanceSensor armDistanceSensor = null;
    //public DistanceSensor radarDistanceSensor = null;
//    public ColorSensor leftClawColorSensor = null;
//    public ColorSensor rightClawColorSensor = null;
//    public DistanceSensor leftClawDistanceSensor = null;
//    public DistanceSensor rightClawDistanceSensor = null;
    public BNO055IMU gyro = null;


    final static double ARM_HOME = 0.0; // 0 -> up; 0.7 -> Right
    final static double CLAW_HOME = 0.1;
    final static double HAND_HOME = 0.8;
    final static double ARM_MIN_RANGE  = 0.00;
    final static double ARM_MAX_RANGE  = 1.0;
    final static double CLAW_MIN_RANGE  = 0.0; // 0-> I; 0.5-> ---
    final static double CLAW_MAX_RANGE  = 0.5;
    final static double HAND_MIN_RANGE  = 0.8; // 0.8->\/; 0.9-> /\;
    final static double HAND_MAX_RANGE  = 1.0;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Mecanum1() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors

        LFMotor = hwMap.get(DcMotor.class, "LFMotor");
        RFMotor = hwMap.get(DcMotor.class, "RFMotor");
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LRMotor = hwMap.get(DcMotor.class, "LRMotor");
        RRMotor = hwMap.get(DcMotor.class, "RRMotor");
        LRMotor.setDirection(DcMotor.Direction.REVERSE);
        tiltMotor = hwMap.get(DcMotor.class, "liftMotor");
        liftMotor = hwMap.get(DcMotor.class, "tiltMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        //  liftM  = hwMap.get(DcMotor.class, "liftM");
        // Set all motors to zero power
        LFMotor.setPower(0);
        RFMotor.setPower(0);
        LRMotor.setPower(0);
        RRMotor.setPower(0);
        liftMotor.setPower(0);
        tiltMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        arm  = hwMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.REVERSE);
        LClaw = hwMap.get(Servo.class, "LClaw");
        RClaw = hwMap.get(Servo.class, "RClaw");
        LHand = hwMap.get(Servo.class, "LHand");
        RHand = hwMap.get(Servo.class, "RHand");
        //radarArm = hwMap.get(Servo.class, "radarArm");

        gyro = hwMap.get(BNO055IMU.class, "imu");

        armColorSensor = hwMap.get(ColorSensor.class, "acs");
//        leftClawColorSensor = hwMap.get(ColorSensor.class, "LCS");
//        rightClawColorSensor = hwMap.get(ColorSensor.class, "RCS");
//        leftClawDistanceSensor = hwMap.get(DistanceSensor.class, "LCS");
//        rightClawDistanceSensor = hwMap.get(DistanceSensor.class, "RCS");
        armDistanceSensor = hwMap.get(DistanceSensor.class, "acs");

        //radarDistanceSensor = hwMap.get(DistanceSensor.class, "rds");


        LHand.setDirection(Servo.Direction.REVERSE);
        LClaw.setDirection(Servo.Direction.REVERSE);
        LHand.setPosition(HAND_HOME);
        RHand.setPosition(HAND_HOME);
        LClaw.setPosition(CLAW_HOME);
        RClaw.setPosition(CLAW_HOME);
    }
}

