package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.legacy.Direction;

/**
 * Created by David Dai on 11/2/18.
 * This class is compatitble with the robot Mecanum19.
 * It contains all the convenient functions for the autonomous mode.
 * Some you may want to use:
 *
 */

public class Mecanum19Drive {
    Mecanum19 robot = null;
    private LinearOpMode opMode = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 1718 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();

    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.06;     // Larger is more responsive, but also less stable

    private double[] distanceMap = new double[181];
    //private double[] slopeMap = new double[181];

    /** Constructor
     * @param opModeInstance The opMode instance. Most of time it's the class itself.
     *                       If that's the case, just use {@code this} for this parameter.
     */
    Mecanum19Drive(LinearOpMode opModeInstance) {
        robot = new Mecanum19();
        opMode = opModeInstance;
        robot.init(opModeInstance.hardwareMap);
    }

    private void resetEncoders() {
        // Send telemetry message to signify robot waiting
        opMode.telemetry.addData("Status", "Resetting Encoders");    //
        opMode.telemetry.update();

        robot.LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setWheelPower(double LFPower,
                       double RFPower,
                       double LRPower,
                       double RRPower) {
        robot.LFMotor.setPower(LFPower);
        robot.RFMotor.setPower(RFPower);
        robot.LRMotor.setPower(LRPower);
        robot.RRMotor.setPower(RRPower);
    }

    // From Mecanum 1. Move the robot using encoders.
    void encoderDriveMove(double speed,
                          Direction Direction,
                          double distanceInInch,
                          double timeoutS) {
        int newLeftRearTarget = 0;
        int newRightRearTarget = 0;
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;


        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            switch (Direction) {
                case LEFT:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.RFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
                case RIGHT:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.RFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
                case BACKWARD:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.RFMotor.getCurrentPosition() - (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
                case FORWARD:
                    newLeftRearTarget = robot.LRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightRearTarget = robot.RRMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newLeftFrontTarget = robot.LFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.RFMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
                    break;
            }
            // Determine new target position, and pass to motor controller

            robot.LRMotor.setTargetPosition(newLeftRearTarget);
            robot.RRMotor.setTargetPosition(newRightRearTarget);
            robot.LFMotor.setTargetPosition(newLeftFrontTarget);
            robot.RFMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LRMotor.setPower(Math.abs(speed));
            robot.LFMotor.setPower(Math.abs(speed));
            robot.RRMotor.setPower(Math.abs(speed));
            robot.RFMotor.setPower(Math.abs(speed));


            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    robot.LRMotor.isBusy() && robot.LFMotor.isBusy() && robot.RRMotor.isBusy()) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newRightFrontTarget,  newLeftFrontTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.RFMotor.getCurrentPosition(),
                        robot.LFMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            robot.LFMotor.setPower(0);
            robot.LRMotor.setPower(0);
            robot.RFMotor.setPower(0);
            robot.RRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }



//    void liftMotorDrive(double speed,
//                        double distanceInInch,
//                        double timeoutS) {
//        int newLiftTarget;
//
//
//        // Ensure that the opmode is still active
//        if (opMode.opModeIsActive()) {
//            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
//            // Determine new target position, and pass to motor controller
//
//            robot.liftMotor.setTargetPosition(newLiftTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.liftMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opMode.opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    robot.liftMotor.isBusy()) {
//
//                // Display it for the driver.
//                opMode.telemetry.addData("Path1",  "Lift mptor running to %7d", newLiftTarget);
//                opMode.telemetry.addData("Path2",  "Running at %7d",
//                        robot.liftMotor.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.liftMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }

    void gyroInit() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        robot.gyro.initialize(parameters);
    }

    /**
     *  Method to spin on central axis to point in a new Direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.

        double currentAngle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double targetAngle = currentAngle + angle;
        while (opMode.opModeIsActive() && !onHeading(speed, targetAngle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
        }
    }

    // Used by the internal class gyroTurn. Don't use it directly.
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.LRMotor.setPower(leftSpeed);
        robot.LFMotor.setPower(leftSpeed);
        robot.RRMotor.setPower(rightSpeed);
        robot.RFMotor.setPower(rightSpeed);

        // Display it for the driver.
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return desired steering force
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    void move(Direction direction, double power) {
        switch (direction) {
            case LEFT:
                robot.LRMotor.setPower(power);
                robot.RRMotor.setPower(-power);
                robot.LFMotor.setPower(-power);
                robot.RFMotor.setPower(power);

                break;
            case RIGHT:
                robot.LRMotor.setPower(-power);
                robot.RRMotor.setPower(power);
                robot.LFMotor.setPower(power);
                robot.RFMotor.setPower(-power);
                break;
            case BACKWARD:
                robot.LRMotor.setPower(-power);
                robot.RRMotor.setPower(-power);
                robot.LFMotor.setPower(-power);
                robot.RFMotor.setPower(-power);
                break;
            case FORWARD:
                robot.LRMotor.setPower(power);
                robot.RRMotor.setPower(power);
                robot.LFMotor.setPower(power);
                robot.RFMotor.setPower(power);
                break;
        }
    }

    void waitFor(double seconds, String message) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < seconds)) {
            opMode.telemetry.addData("Path", message + ": %2.5f S Elapsed", runtime.seconds());
//            telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            opMode.telemetry.update();
        }
    }
}
