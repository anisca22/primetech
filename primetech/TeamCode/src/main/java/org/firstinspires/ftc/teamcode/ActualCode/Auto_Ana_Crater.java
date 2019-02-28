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

package org.firstinspires.ftc.teamcode.ActualCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ActualCode.Old.Main;
import org.firstinspires.ftc.teamcode.HardwareDemoCluj;

import java.util.List;
import java.util.Locale;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.random;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.BIG_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.LOCK_OPEN;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.MARKER_START;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.TURN_SPEED;

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

@Autonomous(name="Auto_Crater", group="Pushbot")
@Disabled
public class Auto_Ana_Crater extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDemoCluj robot = new HardwareDemoCluj();   // Use a Pushbot's hardware
    Orientation angles;
    BNO055IMU imu;
    double gyroDirection = 0;
    double markerPosition = MARKER_START;
    double lockPosition = LOCK_CLOSED;

    private ElapsedTime     runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AddjBXL/////AAAAmcN61ZHW80IvtUwvfesWZa5JrV9AQn+mphNUco4vRSptOi8UXRpia2gnoLyZrCakLsIEUTD6Z84YWrKm3hjsUcsq8XuiTCxroeAOz4ExDes3eBcnsXsEWud++ymX1jCUgGt4sBHuRh7J0BZ+mj4ATIsXcBHf/SlWjmkKavc0vSqfwR6owMJPBzs0tv49k//jc6JJh2pKREB6YGUBUjlTsroX1qGvxxLHLTTHog1tmBe7cvsa+jQAGtn7kItK/quRF9DQqDGo9dc3UlPUbhwX5O9V4cdOt0r45C62g6Buj47mxVzzz5XurgeGYF1dMhLyl4toN5mCi03wUb+L1/X1pBGPNWwD3guQzUy7pGPjQlYw";
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

        /***VUFORIA***/
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /***FLASH***/
        //CameraDevice.getInstance().setFlashTorchMode(true); NU AVEM VOIE CU CAMERA
        //CameraDevice.getInstance().setFlashTorchMode(true);



        /***ENCODERS***/
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());
        telemetry.update();


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroDirection = noformatAngle(angles.angleUnit, angles.firstAngle);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();

        runtime.reset();

        /***    AUTONOMUS STARTS HERE                           ***/
        /***                AUTONOMUS STARTS HERE               ***/
        /***                            AUTONOMUS STARTS HERE   ***/

        lowerRobot();

        telemetry.addData("Direction", gyroDirection);
        telemetry.addData("LOWERED", 1);
        telemetry.update();

        rotateLeft(35, gyroDirection);
        encoderArm(1, 40, 1, 15);

        telemetry.addData("Direction", gyroDirection);
        telemetry.update();
        sleep(1000);

        robot.armMotor.setPower(0);
        telemetry.addData("Direction", gyroDirection);
        telemetry.update();

        if (checkTensorFlow(1000) == true)
        {
            telemetry.addData("MINERAL:", "left");
            telemetry.update();
            driveBackward(21, DRIVE_SPEED);
            sleep(200);
            rotateRight(30, gyroDirection);
            driveBackward(5, DRIVE_SPEED);
            rotateLeft(180,gyroDirection);
            /*
            driveForward(5, DRIVE_SPEED);
            rotateLeft(65,gyroDirection);

            driveBackward(16, 0.5);
            rotateLeft(42, gyroDirection);
            driveBackward(50, 0.5);
            rotateLeft(45,gyroDirection);
            sleep(200);
            robot.markerServo.setPosition(LOCK_OPEN);
            idle();
            */

        }
        else {
            rotateRight(30, gyroDirection);
            driveBackward(5, 0.5);
            if (checkTensorFlow(1000) == true)
            {
                telemetry.addData("MINERAL:", "center");
                telemetry.update();
                driveBackward(20, 0.5);
                sleep(200);
                driveBackward(5,0.5);
                rotateLeft(180,gyroDirection);
                /*
                driveForward(10, DRIVE_SPEED);
                rotateLeft(115,gyroDirection);
                */
            }
            else {
                rotateRight(45, gyroDirection);
                telemetry.addData("MINERAL:", "right");
                telemetry.update();
                driveBackward(20, 0.5);
                sleep(200);
                rotateLeft(30,gyroDirection);
                driveBackward(5,0.5);
                rotateLeft(180,gyroDirection);

                /*
                driveForward(10, DRIVE_SPEED);
                rotateLeft(150,gyroDirection);
                */
            }
        }
        driveForward(5,0.5);
        encoderArm(1, 80, -1, 15);

        /***    AUTONOMUS ENDS HERE                           ***/
        /***                AUTONOMUS ENDS HERE               ***/
        /***                            AUTONOMUS ENDS HERE   ***/
    }


    public boolean checkTensorFlow(int T) {
        boolean isGold = false;
        while(T>0)
        {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int goldie = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                            goldie = (int) recognition.getLeft();
                    }

                    if(goldie != -1)
                        isGold = true;

                    telemetry.update();
                }
            }
            T--;
            sleep(1);
        }
        if (isGold == true)
            telemetry.addData("Found Gold","YES");
        else
            telemetry.addData("Found Gold","NO");
        telemetry.update();
        return isGold;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

        if (target > 0)
        {
            while (heading < 0 && opModeIsActive())
                heading += 360;
            while (heading > 360 && opModeIsActive())
                heading -= 360;
        }
        else //if (target < 0)
        {
            while (heading > 0 && opModeIsActive())
                heading -= 360;
            while (heading < -360 && opModeIsActive())
                heading += 360;
        }
        while (Math.abs(heading - target) > 5 && opModeIsActive())
        {
            if (target > 0)
            {
                if (heading > target)
                {
                    robot.backLeftMotor.setPower(-TURN_SPEED);
                    robot.frontLeftMotor.setPower(-TURN_SPEED);
                    robot.backRightMotor.setPower(TURN_SPEED);
                    robot.frontRightMotor.setPower(TURN_SPEED);
                }
                else if (heading < target)
                {
                    robot.backLeftMotor.setPower(TURN_SPEED);
                    robot.frontLeftMotor.setPower(TURN_SPEED);
                    robot.backRightMotor.setPower(-TURN_SPEED);
                    robot.frontRightMotor.setPower(-TURN_SPEED);
                }
            }
            else
            {
                if (heading < target)
                {
                    robot.backLeftMotor.setPower(-TURN_SPEED);
                    robot.frontLeftMotor.setPower(-TURN_SPEED);
                    robot.backRightMotor.setPower(TURN_SPEED);
                    robot.frontRightMotor.setPower(TURN_SPEED);
                }
                else if (heading > target)
                {
                    robot.backLeftMotor.setPower(TURN_SPEED);
                    robot.frontLeftMotor.setPower(TURN_SPEED);
                    robot.backRightMotor.setPower(-TURN_SPEED);
                    robot.frontRightMotor.setPower(-TURN_SPEED);
                }

            }

            telemetry.addData("STEP", 1);
            telemetry.addData("Difference", Math.abs(heading - target));
            telemetry.addData("Heading", heading);
            telemetry.addData("Target", target);
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            if (target > 0)
            {
                while (heading < 0 && opModeIsActive())
                    heading += 360;
                while (heading > 360 && opModeIsActive())
                    heading -= 360;
            }
            else //if (target < 0)
            {
                while (heading > 0 && opModeIsActive())
                    heading -= 360;
                while (heading < -360 && opModeIsActive())
                    heading += 360;
            }
        }

        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

        if (target > 0)
        {
            while (heading < 0 && opModeIsActive())
                heading += 360;
            while (heading > 360 && opModeIsActive())
                heading -= 360;
        }
        else //if (target < 0)
        {
            while (heading > 0 && opModeIsActive())
                heading -= 360;
            while (heading < -360 && opModeIsActive())
                heading += 360;
        }

        while ((Math.abs(heading - target) > 1) && opModeIsActive())
        {
            if (target > 0)
            {
                if (heading > target)
                {
                    robot.backLeftMotor.setPower(-TURN_SPEED);
                    robot.frontLeftMotor.setPower(-TURN_SPEED);
                    robot.backRightMotor.setPower(TURN_SPEED);
                    robot.frontRightMotor.setPower(TURN_SPEED);
                }
                else if (heading < target)
                {
                    robot.backLeftMotor.setPower(TURN_SPEED);
                    robot.frontLeftMotor.setPower(TURN_SPEED);
                    robot.backRightMotor.setPower(-TURN_SPEED);
                    robot.frontRightMotor.setPower(-TURN_SPEED);
                }
            }
            else
            {
                if (heading < target)
                {
                    robot.backLeftMotor.setPower(-TURN_SPEED);
                    robot.frontLeftMotor.setPower(-TURN_SPEED);
                    robot.backRightMotor.setPower(TURN_SPEED);
                    robot.frontRightMotor.setPower(TURN_SPEED);
                }
                else if (heading > target)
                {
                    robot.backLeftMotor.setPower(TURN_SPEED);
                    robot.frontLeftMotor.setPower(TURN_SPEED);
                    robot.backRightMotor.setPower(-TURN_SPEED);
                    robot.frontRightMotor.setPower(-TURN_SPEED);
                }

            }

            telemetry.addData("STEP", 2);
            telemetry.addData("Difference", Math.abs(heading - target));
            telemetry.addData("Heading", heading);
            telemetry.addData("Target", target);
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            if (target > 0)
            {
                while (heading < 0 && opModeIsActive())
                    heading += 360;
                while (heading > 360 && opModeIsActive())
                    heading -= 360;
            }
            else //if (target < 0)
            {
                while (heading > 0 && opModeIsActive())
                    heading -= 360;
                while (heading < -360 && opModeIsActive())
                    heading += 360;
            }
        }

        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
    }

    private void rotateLeft (double angle, double heading) {
        telemetry.addData("Heading", heading);
        telemetry.addData("NewDir", heading - angle);
        telemetry.update();
        //sleep(1000);
        while (heading < 0 && opModeIsActive())
            heading += 360;
        while (heading > 360 && opModeIsActive())
            heading -= 360;
        while (angle < 0 && opModeIsActive())
            angle += 360;
        while (angle > 360 && opModeIsActive())
            angle -= 360;
        double diff = heading + angle;
        rotation(diff);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroDirection = noformatAngle(angles.angleUnit, angles.firstAngle);
    }

    private void rotateRight (double angle, double heading){
        telemetry.addData("Heading", heading);
        telemetry.addData("NewDir", heading - angle);
        telemetry.update();
        //sleep(1000);
        while (heading < 0 && opModeIsActive())
            heading += 360;
        while (heading > 360 && opModeIsActive())
            heading -= 360;
        while (angle < 0 && opModeIsActive())
            angle += 360;
        while (angle > 360 && opModeIsActive())
            angle -= 360;
        double diff = heading - angle;
        rotation(diff);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroDirection = noformatAngle(angles.angleUnit, angles.firstAngle);
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

            sleep(250);   // optional pause after each move
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
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);
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
            robot.backLeftMotor.setPower(Math.abs(speed));
            robot.backRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

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

            sleep(250);   // optional pause after each move
        }
    }

    private void driveForward (double distance, double speed)
    {
        encoderDrive(speed, distance,15);
    }

    private void driveBackward (double distance, double speed) {
        encoderDrive(speed, -distance,15);
    }

    private void lowerRobot() {
        lockPosition = LOCK_OPEN;
        robot.lockServo.setPosition(LOCK_OPEN);
        idle();
        encoderArm(0.8, 15, 1, 15);
        robot.armMotor.setPower(0);
        /**
        encoderArm(0.8, 15, 1, 15);
        lockPosition = LOCK_OPEN;
        robot.lockServo.setPosition(LOCK_OPEN);
        robot.armMotor.setPower(0.8);
        idle();
        sleep(300);
        robot.armMotor.setPower(0);*/
        sleep(3000);
        telemetry.addData("DONE", 1);
        telemetry.update();
        encoderArm(1, 15, -1, 15);
    }
}

