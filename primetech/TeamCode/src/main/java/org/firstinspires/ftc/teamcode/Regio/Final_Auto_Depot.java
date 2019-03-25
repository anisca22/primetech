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

package org.firstinspires.ftc.teamcode.Regio;

import android.drm.DrmInfoRequest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Regio.HardwareFinal.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.Regio.HardwareFinal.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Regio.HardwareFinal.LATCH_LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.Regio.HardwareFinal.LATCH_LOCK_OPEN;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Depot", group="Pushbot")
@Disabled
public class Final_Auto_Depot extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareFinal robot = new HardwareFinal();   // Use a Pushbot's hardware
    Orientation angles;
    BNO055IMU imu;
    double gyroDirection = 0;
    double lockPosition = LATCH_LOCK_CLOSED;

    int goldPosition = 0;

    private ElapsedTime     runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZdEGAb/////AAABmZ4AF6SZ9UY4tTSsTIZJ44oEW7p8rxW7ag25tT3UZ/fuIsUS/lDsxQlWoG+guqvqmGpsx6Uut/kOGSXDvbaZ92ttTmKtyFS9V1V6LIscYnPO73m6a5FK9WOq3/4xNyyEU1fLmRjmPlO1A2Yg9GtgWPmt8xqcblGsOgjg3rO6/BfXbPXtPLmHgePVwp/FCwEpGb7czfi5AIYcaazuOLKl6DTFzXdq1vlhr/1SZQlfhYOR74ECNclPfQXuLk+AOEoUyWvVemG/gdh1StRGIdUy0upbeM+bKqhOUqUvKC4MasSoQ4REok9uLmsZXxbpEnWiJ1ZbGACRgTb7SdYOY9mFLlkOoV5mPJ6DcGssF1Pkbwzj";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {



        telemetry.addData("HI!", " DON'T TOUCH ANYTHING");
        telemetry.update();

        /***    INIT STARTS HERE                           ***/
        /***                INIT STARTS HERE               ***/
        /***                            INIT STARTS HERE   ***/

        /***INIT***/
        robot.init(hardwareMap);

        /***VUFORIA + TENSORFLOW***/
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        /***ENCODERS***/
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());
        telemetry.update();


        /**     GYRO        **/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroDirection = noformatAngle(angles.angleUnit, angles.firstAngle);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (tfod != null) {
            tfod.activate();
        }

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Status:", "Wait for start");
            telemetry.update();
        }

        runtime.reset();

        /***    AUTONOMUS STARTS HERE                           ***/
        /***                AUTONOMUS STARTS HERE               ***/
        /***                            AUTONOMUS STARTS HERE   ***/

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double start = noformatAngle(angles.angleUnit, angles.firstAngle);

        goldPosition = checkTensorFlow(1000);
        lowerRobot();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);

        telemetry.addData("DIFFERENCE", heading -start);
        telemetry.update();
        //sleep(5000);

        driveBackward(30, DRIVE_SPEED / 5);
        printHeading();

        strafeLeft(70, DRIVE_SPEED);
        printHeading();

        encoderArm(DRIVE_SPEED, 800, -1, 15);

        driveForward(50, DRIVE_SPEED);
        printHeading();

        if (goldPosition == 1)
        {
            /** RIGHT **/
            strafeRight(120, DRIVE_SPEED);
            rotateRight(10);
            printHeading();

            encoderMainDiagonal(DRIVE_SPEED, -350, 15);
            rotateLeft(30);
            printHeading();

            driveForward(60, DRIVE_SPEED);
            rotateLeft(180);
            printHeading();
            driveBackward(100, DRIVE_SPEED);
            encoderArm(DRIVE_SPEED, 1300, 1, 15);
        }
        else if (goldPosition == -1)
        {
            /** LEFT **/
            encoderSecondaryDiagonal(DRIVE_SPEED, -150, 15);
            strafeRight(40, DRIVE_SPEED);
            encoderSecondaryDiagonal(DRIVE_SPEED, -250, 15);
            rotateRight(30);
            printHeading();

            driveForward(60, DRIVE_SPEED);
            rotateLeft(180);
            printHeading();
            encoderArm(DRIVE_SPEED, 1300, 1, 15);
        }
        else
        {
            /** CENTER **/
            strafeRight(110, DRIVE_SPEED);
            driveForward(50, DRIVE_SPEED);
            strafeRight(50, DRIVE_SPEED);
            encoderSecondaryDiagonal(DRIVE_SPEED, -120, 15);
            driveForward(200, DRIVE_SPEED);
            rotateLeft(180);
            printHeading();

            driveBackward(60, DRIVE_SPEED);

            encoderArm(DRIVE_SPEED, 1300, 1, 15);

        }
    }


    public void printHeading()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("HEADING:", heading);
        telemetry.update();
    }

    public int checkTensorFlow(int T) {
        int goldie = 0;
        while(T>0 && opModeIsActive())
        {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldie = -1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldie = 1;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldie = 0;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
            sleep(1);
            T--;
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        return goldie;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void rotation(double target) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
        target = heading + target;
        double distance = Math.abs(heading - target);
        double direction = 0;

        while (distance > 5 && opModeIsActive() && !isStopRequested())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            telemetry.addData("Heading", heading);
            telemetry.update();
            if (Math.abs(heading - target) < 180)
            {
                if (target > heading)
                {
                    distance = target - heading;
                    direction = 1; ///DREAPTA
                }
                else
                {
                    distance = heading - target;
                    direction = -1; ///STANGA
                }
            }
            else
            {
                if (target > heading)
                {
                    distance = heading - target + 360;
                    direction = -1; ///STANGA
                }
                else
                {
                    distance = target - heading + 360;
                    direction = 1; ///DREAPTA
                }
            }

            robot.backLeftMotor.setPower(HardwareFinal.TURN_SPEED * direction);
            robot.frontLeftMotor.setPower(HardwareFinal.TURN_SPEED * direction);
            robot.backRightMotor.setPower(HardwareFinal.TURN_SPEED * direction);
            robot.frontRightMotor.setPower(HardwareFinal.TURN_SPEED * direction);

        }

        /*** CLOSER ***/

        while (distance > 1 && opModeIsActive() && !isStopRequested())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            telemetry.addData("Heading", heading);
            telemetry.update();
            if (Math.abs(heading - target) < 180)
            {
                if (target > heading)
                {
                    distance = target - heading;
                    direction = 1; ///DREAPTA
                }
                else
                {
                    distance = heading - target;
                    direction = -1; ///STANGA
                }
            }
            else
            {
                if (target > heading)
                {
                    distance = heading - target + 360;
                    direction = -1; ///STANGA
                }
                else
                {
                    distance = target - heading + 360;
                    direction = 1; ///DREAPTA
                }
            }

            robot.backLeftMotor.setPower(HardwareFinal.TURN_SPEED / 2 * direction);
            robot.frontLeftMotor.setPower(HardwareFinal.TURN_SPEED / 2  * direction);
            robot.backRightMotor.setPower(HardwareFinal.TURN_SPEED / 2 * direction);
            robot.frontRightMotor.setPower(HardwareFinal.TURN_SPEED / 2 * direction);

        }

        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

    }

    private void rotateLeft (double angle) {
        rotation(+ angle);
    }

    private void rotateRight (double angle){
        rotation(- angle);
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double noformatAngle(AngleUnit angleUnit, double angle) {
        return noFormatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    double noFormatDegrees(double degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }

    public void encoderArm(double speed, double distance, double direction, double timeoutS) {
        int armTarget;

        if (direction == -1)
            robot.armMotor.setDirection(DcMotor.Direction.REVERSE);
        else if (direction == 1)
            robot.armMotor.setDirection(DcMotor.Direction.FORWARD);
        // Ensure that the opmode is still active
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive() && !isStopRequested() && !(gamepad1.dpad_up && gamepad1.b)) {

            // Determine new target position, and pass to motor controller
            armTarget = robot.armMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);

            robot.armMotor.setTargetPosition(armTarget);

            // Turn On RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.armMotor.isBusy())
                    && !isStopRequested() && !(gamepad1.a && gamepad1.b))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to :%7d", armTarget);
                telemetry.addData("Path2",  "Running at :%7d", robot.armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.armMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armMotor.setDirection(DcMotor.Direction.FORWARD);

            //sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM);

            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.backLeftMotor.setPower(speed);
            robot.backRightMotor.setPower(-speed);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(-speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() &&
                                    robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()
                    ))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget
                );
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition()
                        ,
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }

    public void encoderStrafe(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);

            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.backLeftMotor.setPower(-speed);
            robot.backRightMotor.setPower(-speed);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() &&
                                    robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()
                    ))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget
                );
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition()
                        ,
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }

    public void encoderMainDiagonal(double speed, double distance,  double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double frontLeftDistance =  + distance * COUNTS_PER_MM;
            double backRightDistance =  - distance * COUNTS_PER_MM;

            telemetry.addData("fld", frontLeftDistance);
            telemetry.addData("brd", backRightDistance);

            telemetry.update();

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(frontLeftDistance);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(backRightDistance);

            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double frontLeftSpeed = + speed;
            double backRightSpeed = - speed;

            runtime.reset();
            robot.frontLeftMotor.setPower(frontLeftSpeed);
            robot.backRightMotor.setPower(backRightSpeed);

            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftMotor.isBusy() && robot.backRightMotor.isBusy()) )
            {
                telemetry.addData("FLM has", newFrontLeftTarget - robot.frontLeftMotor.getCurrentPosition());
                telemetry.addData("BRM has", newBackRightTarget - robot.backRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }

    public void encoderSecondaryDiagonal(double speed, double distance,  double timeoutS) {
        int newBackLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double frontRightDistance =  - distance * COUNTS_PER_MM;
            double backLeftDistance   =  + distance * COUNTS_PER_MM;

            telemetry.addData("fld", frontRightDistance);
            telemetry.addData("brd", backLeftDistance);

            telemetry.update();

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(frontRightDistance);
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(backLeftDistance);

            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);
            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double frontRightSpeed = - speed;
            double backLeftSpeed   = + speed;

            runtime.reset();
            robot.frontRightMotor.setPower(frontRightSpeed);
            robot.backLeftMotor.setPower(backLeftSpeed);

            robot.frontLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy()) )
            {
                telemetry.addData("FLM has", newFrontRightTarget - robot.frontRightMotor.getCurrentPosition());
                telemetry.addData("BRM has", newBackLeftTarget - robot.backLeftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }

    public void encoderDiagonal(double speed, double distance, double angle,  double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        double drive = Math.sin(angle);
        double strafe = Math.cos(angle);

        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.update();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double backLeftDistance = (- drive - strafe) * 200 * distance * COUNTS_PER_MM;
            double backRightDistance = (+ drive - strafe) * 200 * distance * COUNTS_PER_MM;
            double frontLeftDistance = (- drive + strafe) * 200 * distance * COUNTS_PER_MM;
            double frontRightDistance = (+ drive + strafe) * 200 * distance * COUNTS_PER_MM;

            telemetry.addData("bld", backLeftDistance);
            telemetry.addData("brd", backRightDistance);
            telemetry.addData("fld", frontLeftDistance);
            telemetry.addData("frd", frontRightDistance);
            telemetry.update();

            sleep(5000);

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(backLeftDistance);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(backRightDistance);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(frontLeftDistance);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(frontRightDistance);

            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.

            //Range.clip((+ drive + strafe) * speed, -1.0, 1.0);

            double backLeftSpeed = Range.clip((- drive - strafe) * speed, -1.0, 1.0);
            double backRightSpeed = Range.clip((+ drive - strafe) * speed, -1.0, 1.0);;
            double frontLeftSpeed = Range.clip((- drive + strafe) * speed, -1.0, 1.0);
            double frontRightSpeed = Range.clip((+ drive + strafe) * speed, -1.0, 1.0);

            runtime.reset();
            robot.backLeftMotor.setPower(backLeftSpeed);
            robot.backRightMotor.setPower(backRightSpeed);
            robot.frontLeftMotor.setPower(frontLeftSpeed);
            robot.frontRightMotor.setPower(frontRightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            int cnt = 0;

            if (robot.backLeftMotor.isBusy())
                cnt++;
            if (robot.backRightMotor.isBusy())
                cnt++;
            if (robot.frontLeftMotor.isBusy())
                cnt++;
            if (robot.frontRightMotor.isBusy())
                cnt++;

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (cnt > 1))
            {
                cnt = 0;
                if (robot.backLeftMotor.isBusy())
                    cnt++;
                if (robot.backRightMotor.isBusy())
                    cnt++;
                if (robot.frontLeftMotor.isBusy())
                    cnt++;
                if (robot.frontRightMotor.isBusy())
                    cnt++;

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget
                );
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition()
                        ,
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    private void driveForward (double distance, double speed)
    {
        encoderDrive(speed, -distance,15);
    }

    private void driveBackward (double distance, double speed) {
        encoderDrive(-speed, distance,15);
    }

    private void strafeRight(double distance, double speed) {
        encoderStrafe(speed, -distance, 15);
    }

    private void strafeLeft (double distance, double speed) {
        encoderStrafe(-speed, distance, 15);
    }

    private void moveForwardRight(double distance, double speed) {
        encoderSecondaryDiagonal(distance, speed, 15);
    }

    private void moveForwardLeft(double distance, double speed) {
        encoderMainDiagonal(distance, speed, 15);
    }

    private void moveBackwardRight(double distance, double speed) {
        encoderMainDiagonal(-distance, -speed, 15);
    }

    private void moveBackwardLeft(double distance, double speed) {
        encoderSecondaryDiagonal(-distance, -speed, 15);
    }

    private void lowerRobot() {
        lockPosition = LATCH_LOCK_OPEN;
        robot.latchServo.setPosition(LATCH_LOCK_OPEN);
        idle();
        encoderArm(1, 150, -1, 15);///CLOSE
        encoderArm( 1, 2800, 1, 15);///UNLATCH
        /**
        robot.armMotor.setPower(0);
        sleep(3000);
        telemetry.addData("DONE", 1);
        telemetry.update();
        encoderArm(1, 150, -1, 15);-*/
    }
}

