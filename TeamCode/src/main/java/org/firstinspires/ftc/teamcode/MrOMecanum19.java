package org.firstinspires.ftc.teamcode;

/**
 * Created by David Dai on 10/22/18.
 */

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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 *
 * Motor  channel:  Left rear motor:                          "LRMotor"
 * Motor  channel:  Right rear motor:                         "RRMotor"
 * Motor  channel:  Left front motor:                         "LFMotor"
 * Motor  channel:  Right front motor:                        "RFMotor"
 * Motor  channel:  Lift motor:                               "liftM"
 * Motor  channel:  Sweeper motor:                            "sweeper"
 * Motor  channel:  Major arm at the center:                  "mainArm"
 * Motor  channel: 
 * Gyro   channel:  Robot gyro of type "AdaFruit IMU":        "imu"
 * Sensor channel:  Left color sensor detecting the ball:     "leftColor"
 * Sensor channel:  Right color sensor detecting the ball:    "rightColor"
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class MrOMecanum19 {
    /* Public OpMode members. */
    public DcMotor LFMotor = null;
    public DcMotor RFMotor = null;
    public DcMotor LRMotor = null;
    public DcMotor RRMotor = null;
    public DcMotor liftM   = null;
    //public DcMotor sweeper = null;
    public CRServo sweeper = null;
    public Servo   sweeperARM = null;
    public DcMotor mainArm = null;
    public BNO055IMU gyro = null;
    // public Servo    arm         = null;
//    public Servo    Lclaw        = null;
//    public Servo    Rclaw        = null;

    public final static double ARM_HOME = 1.0; // 0 -> up; 0.7 -> Right
   // public final static double CLAW_HOME = 0.3;
    public final static double ARM_MIN_RANGE  = 1.00;
    public final static double ARM_MAX_RANGE  = 0.50;
    //public final static double CLAW_MIN_RANGE  = 0.30;
    ///public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MrOMecanum19() {
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
        liftM  = hwMap.get(DcMotor.class, "liftM");
        mainArm = hwMap.get(DcMotor.class, "mainArm");
        mainArm.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        LFMotor.setPower(0);
        RFMotor.setPower(0);
        LRMotor.setPower(0);
        RRMotor.setPower(0);
     //   sweeper.setPower(0);
        liftM.setPower(0);
       // sweeper.setPower(0);
        mainArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        sweeperARM  = hwMap.get(Servo.class, "sweeperARM");
        sweeper = hwMap.get(CRServo.class, "sweeper");
//        Rclaw = hwMap.get(Servo.class, "Rclaw");
        sweeperARM.setPosition(ARM_HOME);
//        Lclaw.setDirection(Servo.Direction.REVERSE);
//        Lclaw.setPosition(CLAW_HOME);
//        Rclaw.setPosition(CLAW_HOME);
    }
}


