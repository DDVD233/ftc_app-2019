package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mecanum19;

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

@TeleOp(name="Mecanum Telop")
//@Disabled
public class Mecanum19Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum19 robot           = new Mecanum19();                         //
    // double          armPosition     = robot.ARM_HOME;                 // Servo safe position
    private double          clawPosition    = Mecanum19.CLAW_HOME;       // Servo safe position
    private final double    CLAW_SPEED      = 0.05 ;                     // sets rate to move servo
    final double    ARM_SPEED       = 0.05  ;                            // sets rate to move servo
    double          lift            = 0.2;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double LFspeed;
        double RFspeed;
        double LRspeed;
        double RRspeed;
        //  double liftup;
        //  double liftdown;

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

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            LFspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            LRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
            RFspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            RRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;

            LFspeed = Range.clip(LFspeed, -1, 1);
            LRspeed = Range.clip(LRspeed, -1, 1);
            RFspeed = Range.clip(RFspeed, -1, 1);
            RRspeed = Range.clip(RRspeed, -1, 1);


            robot.LFMotor.setPower(LFspeed);
            robot.RFMotor.setPower(RFspeed);
            robot.LRMotor.setPower(LRspeed);
            robot.RRMotor.setPower(RRspeed);

            robot.liftM.setPower(gamepad1.left_trigger-gamepad1.right_trigger);



            setLiftMotorPower();
            setSweeperPower();
            telemetry.update();

            telemetry.addData("LFMotor",   "%.2f", robot.LFMotor.getPower());
            telemetry.addData("LRMotor",  "%.2f", LRspeed);
            telemetry.addData("RFMotor",  "%.2f", RFspeed);
            telemetry.addData("RFMotor",  "%.2f", RRspeed) ;
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }




    /**
     * Use x and a button to control the lift motor.
     * If the lift power is not 0, pressing any of the buttons will stop the motor.
     * Otherwise the x button will move the lift up while b button will lower the lift.
     */
    private void setLiftMotorPower() {
        if (gamepad1.x) {
            double liftPower = (robot.liftM.getPower()==0.0)?1.0:0.0;
            robot.liftM.setPower(liftPower);
        } else if (gamepad1.a) {
            double liftPower = (robot.liftM.getPower()==0.0)?-1.0:0.0;
            robot.liftM.setPower(liftPower);
        }
    }

    /**
     * Use left bumper to control the sweeper.
     * The bumper act as a switch, turning the sweeper on and off.
     */
    private void setSweeperPower() {
        if (gamepad1.left_bumper) {
            double sweeperPower = (robot.sweeper.getPower()==0.0)?1.0:0.0;
            robot.sweeper.setPower(sweeperPower);
        }
    }
}

