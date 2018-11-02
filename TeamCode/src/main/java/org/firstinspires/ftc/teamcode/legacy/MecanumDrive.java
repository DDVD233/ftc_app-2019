package org.firstinspires.ftc.teamcode.legacy;

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

/**
 * This is a class for robot Mecanum-1 that uses across different autonomous modes.
 * Before you use these functions, you must initialize this class using the constructor {@link #MecanumDrive(Mecanum1, LinearOpMode) constructor} below.
 */

class MecanumDrive {
    Mecanum1 robot = null;
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
     * @param robotInstance The robot instance initialized using method robot.init(hardwareMap)
     * @param opModeInstance The opMode instance. Most of time it's the class itself.
     *                       If that's the case, just use {@code this} for this parameter.
     */
    MecanumDrive(Mecanum1 robotInstance, LinearOpMode opModeInstance) {
        robot = robotInstance;
        opMode = opModeInstance;
    }

    private void resetEncoders() {
        // Send telemetry message to signify robot waiting
        opMode.telemetry.addData("Status", "Resetting Encoders");    //
        opMode.telemetry.update();

        robot.LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
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



    void liftMotorDrive(double speed,
                        double distanceInInch,
                        double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int)(distanceInInch * COUNTS_PER_INCH);
            // Determine new target position, and pass to motor controller

            robot.liftMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    robot.liftMotor.isBusy()) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Lift mptor running to %7d", newLiftTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d",
                        robot.liftMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

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

    void autonomous90(Team team) {
        resetEncoders();
        gyroInit();

        // Send telemetry message to indicate successful Encoder reset
        opMode.telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.LRMotor.getCurrentPosition(),
                robot.RRMotor.getCurrentPosition());
        opMode.telemetry.update();

        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.arm.setPosition(0.0);
        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        robot.LClaw.setPosition(0.0); //arm up
        robot.RClaw.setPosition(0.0);

        // Send telemetry message to signify robot waiting;
        opMode.telemetry.addData("Status", "Ready to run");
        opMode.telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        opMode.waitForStart();

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
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

        opMode.telemetry.addData(">", "Press Play to start");
        opMode.telemetry.update();

        opMode.waitForStart();
//>>>>>>>>>>>>>>>>>>>>>>>>>>START>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        relicTrackables.activate();

        RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

        runtime.reset();
        robot.LClaw.setPosition(0.5);
        robot.RClaw.setPosition(0.5);
        while (opMode.opModeIsActive() && runtime.seconds() < 2.5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                opMode.telemetry.addData("VuMark", "%s visible", vuMark);
                column = vuMark;

                break; // Break the while loop after the vuMark is found
            } else {
                opMode.telemetry.addData("VuMark", "not visible");
            }

            opMode.telemetry.update();
        }

        robot.RHand.setPosition(1.0);
        robot.LHand.setPosition(1.0);

        robot.arm.setPosition(0.65);
        liftMotorDrive(1.0, 2, 2); // Lift the block up 2 inches
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
            opMode.telemetry.addData("Path", "Move sensor arm: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            opMode.telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            switch (column) {
                case RIGHT:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Right";
                        }
                    });
                    break;
                case LEFT:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Left";
                        }
                    });
                    break;
                case CENTER:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Center";
                        }
                    });
                    break;
                case UNKNOWN:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Unknown";
                        }
                    });
                    break;
            }
            opMode.telemetry.update();
        }

        boolean isRed = (robot.armColorSensor.red() > robot.armColorSensor.blue());
        double backwardInch = 28.2; // The distance to move forward/backward afterward
        switch (column) {
            case RIGHT:
                backwardInch = 24;
                break;
            case LEFT:
                backwardInch = 37;
                break;
            case CENTER:
                backwardInch = 28.6;
                break;
            case UNKNOWN:
                break;
        }


        if (isRed) {
            // Move back
            if (team == Team.BLUE) {
                encoderDriveMove(0.3, Direction.BACKWARD, 3, 1);
                backwardInch -= 3;
            } else {
                encoderDriveMove(0.3, Direction.FORWARD, 3, 1);
                backwardInch += 3;
            }
        } else {
            // Move forward
            if (team == Team.RED) {
                encoderDriveMove(0.3, Direction.BACKWARD, 3, 1);
                backwardInch -= 3;
            } else {
                encoderDriveMove(0.3, Direction.FORWARD, 3, 1);
                backwardInch += 3;
            }
        }

        robot.arm.setPosition(0.0);

        switch (team) {
            case RED:
                encoderDriveMove(0.7, Direction.BACKWARD, backwardInch, 5);
                break;
            case BLUE:
                encoderDriveMove(0.7, Direction.FORWARD, backwardInch, 5);
                break;
        }

        gyroTurn(0.8, 82);

        encoderDriveMove(0.7, Direction.FORWARD, 10, 3);

        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.3)) {
            opMode.telemetry.addData("Path", "Move arm apart: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }

        encoderDriveMove(0.3, Direction.BACKWARD, 6, 2);

        encoderDriveMove(0.3, Direction.FORWARD, 6, 2);

        encoderDriveMove(0.3, Direction.BACKWARD, 6, 2);

        opMode.telemetry.addData("Path", "Complete");
        opMode.telemetry.update();
    }

    void autonomous180(Team team) {
        resetEncoders();
        gyroInit();

        // Send telemetry message to indicate successful Encoder reset
        opMode.telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.LRMotor.getCurrentPosition(),
                robot.RRMotor.getCurrentPosition());
        opMode.telemetry.update();

        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.arm.setPosition(0.0);
        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        robot.LClaw.setPosition(0.0); //arm up
        robot.RClaw.setPosition(0.0);

        // Send telemetry message to signify robot waiting;
        opMode.telemetry.addData("Status", "Ready to run");
        opMode.telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        opMode.waitForStart();

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
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

        opMode.telemetry.addData(">", "Press Play to start");

        opMode.telemetry.update();

        opMode.waitForStart();
//>>>>>>>>>>>>>>>>>>>>>>>>>>START>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        relicTrackables.activate();

        RelicRecoveryVuMark column = RelicRecoveryVuMark.CENTER;

        runtime.reset();
        robot.LClaw.setPosition(0.5);
        robot.RClaw.setPosition(0.5);
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                opMode.telemetry.addData("VuMark", "%s visible", vuMark);
                column = vuMark;

                break; // Break the while loop after the vuMark is found
            } else {
                opMode.telemetry.addData("VuMark", "not visible");
            }

            opMode.telemetry.update();
        }

        robot.RHand.setPosition(1.0);
        robot.LHand.setPosition(1.0);

        robot.arm.setPosition(0.7);
        liftMotorDrive(1.0, 2, 1.5); // Lift the block up 4 inches
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 2.5)) {
            opMode.telemetry.addData("Path", "Move sensor arm: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            opMode.telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            switch (column) {
                case RIGHT:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Right";
                        }
                    });
                    break;
                case LEFT:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Left";
                        }
                    });
                    break;
                case CENTER:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Center";
                        }
                    });
                    break;
                case UNKNOWN:
                    opMode.telemetry.addData("Position",new Func<String>() {
                        @Override public String value() {
                            return "Unknown";
                        }
                    });
                    break;
            }
            opMode.telemetry.update();
        }

        boolean isRed = (robot.armColorSensor.red() > robot.armColorSensor.blue());
        double backwardInch = 27.5; // The distance to move forward afterward
        if (isRed) {
            // Move back
            if (team == Team.RED) {
                encoderDriveMove(0.3, Direction.FORWARD, 2, 1);
                backwardInch -= 3;
            } else {
                encoderDriveMove(0.3, Direction.BACKWARD, 2, 1);
                backwardInch -= 3;
            }
        } else {
            // Move forward
            if (team == Team.BLUE) {
                encoderDriveMove(0.3, Direction.FORWARD, 2, 1);
                backwardInch -= 3;
            } else {
                encoderDriveMove(0.3, Direction.BACKWARD, 2, 1);
                backwardInch -= 3;
            }
        }

        robot.arm.setPosition(0.0);

        switch (team) {
            case BLUE:
                encoderDriveMove(0.8, Direction.FORWARD, backwardInch, 5);
                break;
            case RED:
                encoderDriveMove(0.8, Direction.BACKWARD, backwardInch, 5);
                break;
        }

        double distanceToTheRight = 8;
        switch (team) {
            case RED:
                switch (column) {
                    case RIGHT:
                        distanceToTheRight = 1.5;
                        break;
                    case LEFT:
                        distanceToTheRight = 18.5;
                        break;
                    case CENTER:
                        distanceToTheRight = 8.5;
                        break;
                    case UNKNOWN:
                        break;
                }
                break;
            case BLUE:
                switch (column) {
                    case RIGHT:
                        distanceToTheRight = 9.5;
                        break;
                    case LEFT:
                        distanceToTheRight = 0.5;
                        break;
                    case CENTER:
                        distanceToTheRight = 5;
                        break;
                    case UNKNOWN:
                        break;
                }
                break;
        }

        encoderDriveMove(0.5, Direction.RIGHT, distanceToTheRight, 4);

        if (team == Team.RED) {
            gyroTurn(0.8, 180);
        }

        encoderDriveMove(0.7, Direction.FORWARD, 9.5, 3);

        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.3)) {
            opMode.telemetry.addData("Path", "Move arm apart: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }

        encoderDriveMove(0.3, Direction.BACKWARD, 6, 1);

        encoderDriveMove(0.3, Direction.FORWARD, 6, 1);

        encoderDriveMove(0.3, Direction.BACKWARD, 3, 1);

        opMode.telemetry.addData("Path", "Complete");
        opMode.telemetry.update();
    }

    void autonomous180II(Team team) {
        resetEncoders();
        gyroInit();

        // Send telemetry message to indicate successful Encoder reset
        opMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.LRMotor.getCurrentPosition(),
                robot.RRMotor.getCurrentPosition());
        opMode.telemetry.update();

        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.LRMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.arm.setPosition(0.0);
        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        robot.LClaw.setPosition(0.0); //arm up
        robot.RClaw.setPosition(0.0);

        // Send telemetry message to signify robot waiting;
        opMode.telemetry.addData("Status", "Ready to run");
        opMode.telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        opMode.waitForStart();

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
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

        opMode.telemetry.addData(">", "Press Play to start");

        opMode.telemetry.update();

        opMode.waitForStart();
//>>>>>>>>>>>>>>>>>>>>>>>>>>START>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        relicTrackables.activate();

        RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

        runtime.reset();
        robot.LClaw.setPosition(0.5);
        robot.RClaw.setPosition(0.5);
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                opMode.telemetry.addData("VuMark", "%s visible", vuMark);
                column = vuMark;

                break; // Break the while loop after the vuMark is found
            } else {
                opMode.telemetry.addData("VuMark", "not visible");
            }

            opMode.telemetry.update();
        }

        robot.RHand.setPosition(1.0);
        robot.LHand.setPosition(1.0);

        robot.arm.setPosition(0.7);
        liftMotorDrive(1.0, 2, 2); // Lift the block up 4 inches
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 1.0)) {
            opMode.telemetry.addData("Path", "Move sensor arm: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.addData("Red", String.valueOf(robot.armColorSensor.red()));
            opMode.telemetry.addData("Blue", String.valueOf(robot.armColorSensor.blue()));
            switch (column) {
                case RIGHT:
                    opMode.telemetry.addData("Position", new Func<String>() {
                        @Override
                        public String value() {
                            return "Right";
                        }
                    });
                    break;
                case LEFT:
                    opMode.telemetry.addData("Position", new Func<String>() {
                        @Override
                        public String value() {
                            return "Left";
                        }
                    });
                    break;
                case CENTER:
                    opMode.telemetry.addData("Position", new Func<String>() {
                        @Override
                        public String value() {
                            return "Center";
                        }
                    });
                    break;
                case UNKNOWN:
                    opMode.telemetry.addData("Position", new Func<String>() {
                        @Override
                        public String value() {
                            return "Unknown";
                        }
                    });
                    break;
            }
            opMode.telemetry.update();
        }

        boolean isRed = (robot.armColorSensor.red() > robot.armColorSensor.blue());
        double backwardInch = 25; // The distance to move forward afterward
        if (isRed) {
            // Move back
            if (team == Team.RED) {
                encoderDriveMove(0.3, Direction.FORWARD, 2, 1);
                backwardInch -= 3;
            } else {
                encoderDriveMove(0.3, Direction.BACKWARD, 2, 1);
                backwardInch -= 3;
            }
        } else {
            // Move forward
            if (team == Team.BLUE) {
                encoderDriveMove(0.3, Direction.FORWARD, 2, 1);
                backwardInch -= 3;
            } else {
                encoderDriveMove(0.3, Direction.BACKWARD, 2, 1);
                backwardInch -= 3;
            }
        }

        robot.arm.setPosition(0.0);

        switch (team) {
            case BLUE:
                encoderDriveMove(1.0, Direction.FORWARD, backwardInch, 5);
                break;
            case RED:
                encoderDriveMove(1.0, Direction.BACKWARD, backwardInch, 5);
                break;
        }

//        encoderDriveMove(0.5, Direction.RIGHT, 6, 4);
//
//        for (int angle=0; angle<180; angle++) {
//            robot.radarArm.setPosition(angle/180);
//            opMode.sleep(20);
//            distanceMap[angle] = robot.radarDistanceSensor.getDistance(DistanceUnit.INCH);
//        }

        if (team == Team.RED) {
            gyroTurn(0.8, 180);
        }

        int moveCount = 2;
        switch (column) {
            case LEFT:
                moveCount = 3;
                break;
            case CENTER:
                moveCount = 2;
                break;
            case RIGHT:
                moveCount = 1;
                break;
            case UNKNOWN:
                moveCount = 2;
                break;
        }

//        for (int i=1; i==moveCount; i++) {
//            moveUntilSeesRibs(Direction.LEFT);
//        }

        encoderDriveMove(0.7, Direction.FORWARD, 9.5, 3);

        robot.RHand.setPosition(0.8); //arm \ /
        robot.LHand.setPosition(0.8);
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.3)) {
            opMode.telemetry.addData("Path", "Move arm apart: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }

        encoderDriveMove(0.3, Direction.BACKWARD, 6, 1);

        encoderDriveMove(0.3, Direction.FORWARD, 6, 1);

        encoderDriveMove(0.3, Direction.BACKWARD, 3, 1);

        opMode.telemetry.addData("Path", "Complete");
        opMode.telemetry.update();
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

    private void stopWheelMotors() {
        robot.LRMotor.setPower(0);
        robot.RRMotor.setPower(0);
        robot.LFMotor.setPower(0);
        robot.RFMotor.setPower(0);
    }

//    private void moveUntilSeesRibs(Direction direction) {
//        move(direction, 0.3);
//        while (isSeeingRibs() && opMode.opModeIsActive()) {
//            opMode.telemetry.addData("Path", "Moving sideways until doesn't see ribs");
//            opMode.telemetry.update();
//        }
//
//        while (!isSeeingRibs() && opMode.opModeIsActive()) {
//            opMode.telemetry.addData("Path", "Moving sideways until sees ribs");
//            opMode.telemetry.update();
//        }
//
//        stopWheelMotors();
//        opMode.sleep(100);
//    }
//
//    private boolean isSeeingRibs() {
//        return ((Math.abs(robot.rightClawColorSensor.blue() - robot.rightClawColorSensor.red()) > 55)
//                && (Math.abs(robot.leftClawColorSensor.blue() - robot.leftClawColorSensor.red()) > 55));
//    }
}
