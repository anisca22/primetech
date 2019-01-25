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

package org.firstinspires.ftc.teamcode.ActualCode.Old;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareTurda;

import java.util.List;

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

@Autonomous(name="AutoCrater", group="Pushbot")
@Disabled
public class AutoTest2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTurda robot = new HardwareTurda();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro   gyro    = null;
    ColorSensor sensorRGB;

    private ElapsedTime     runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AddjBXL/////AAAAmcN61ZHW80IvtUwvfesWZa5JrV9AQn+mphNUco4vRSptOi8UXRpia2gnoLyZrCakLsIEUTD6Z84YWrKm3hjsUcsq8XuiTCxroeAOz4ExDes3eBcnsXsEWud++ymX1jCUgGt4sBHuRh7J0BZ+mj4ATIsXcBHf/SlWjmkKavc0vSqfwR6owMJPBzs0tv49k//jc6JJh2pKREB6YGUBUjlTsroX1qGvxxLHLTTHog1tmBe7cvsa+jQAGtn7kItK/quRF9DQqDGo9dc3UlPUbhwX5O9V4cdOt0r45C62g6Buj47mxVzzz5XurgeGYF1dMhLyl4toN5mCi03wUb+L1/X1pBGPNWwD3guQzUy7pGPjQlYw";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    static final double     HEADING_THRESHOLD       = 5 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder 1440 tetrix
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM   = 4.0 * 2.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.2;


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
        //CameraDevice.getInstance().setFlashTorchMode(true);

        /***GYRO***/
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        /***COLOR SENSOR***/
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        /***ENCODERS***/
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());
        telemetry.update();


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        gyro.resetZAxisIntegrator();

        waitForStart();

        if (tfod != null) {
            tfod.activate();
        }
        runtime.reset();

        /***    AUTONOMUS STARTS HERE                           ***/
        /***                AUTONOMUS STARTS HERE               ***/
        /***                            AUTONOMUS STARTS HERE   ***/



        //lowerRobot(50);
        //sleep(1000);

        ///COBORARE ROBOT

        encoderArm(0.2, 170, 30);

        /*
        rotateLeft(90);
        rotateRight(90);

        encoderArm(1, 100, 20);
        sleep(1000);
        driveBackward(2);

        //ROTIRE PT DAT JOS DE PE LANDER
        rotateLeft(60);
        sleep(1000);
        driveForward(10);
        rotateRight(90);
        driveForward(10);
        rotateLeft(30);
        ///AICI II ALINIAT ROBOTUL

        ///SE UITA DREAPTA
        rotateRight(45);
        if(checkTensorFlow(2000) == true)
        {
            sleep(1000);
            driveForward(54);
            driveBackward(54);
        }
        else
        {
            sleep(1000);
            ///SE UITA FATA
            rotateLeft(45);
            if(checkTensorFlow(2000) == true)
            {
                sleep(1000);
                driveForward(43);
                driveBackward(43);
            }
            else
            {
                sleep(1000);
                ///SE UITA STANGA
                rotateLeft(45);
                if(checkTensorFlow(2000) == true)
                {
                    driveForward(54);
                    driveBackward(54);
                }
            }
        }
        sleep(1000);

        rotateLeft(90);
        driveForward(104);
        rotateLeft(45);
        driveForward(127);
        ///PUNE TEAM MARKER-UL AICI
        
        /**
        if (isGold == true)
            telemetry.addData("Found Gold","YES");
        else
            telemetry.addData("Found Gold","NO");
        telemetry.update();
         */
        sleep(1000);


        /**
        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.addData("Path", "Complete");
        telemetry.update();
         */


        /***    AUTONOMUS ENDS HERE                           ***/
        /***                AUTONOMUS ENDS HERE               ***/
        /***                            AUTONOMUS ENDS HERE   ***/
    }


    public boolean checkTensorFlow(int T)
    {
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

                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                                ///isGold = true;
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                    }
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


    public void encoderArm(double speed, double distance, double timeoutS) {
        int armTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            armTarget = robot.armMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            robot.armMotor.setTargetPosition(armTarget);

            // Turn On RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.armMotor.isBusy()))
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

            sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

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

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        //int     newLeftTarget;
        //int     newRightTarget;

        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     newFrontLeftTarget;
        int     newFrontRightTarget;

        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);

            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;

            //newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            //newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Set Target and Turn On RUN_TO_POSITION
            ///robot.leftDrive.setTargetPosition(newLeftTarget);
            ///robot.rightDrive.setTargetPosition(newRightTarget);


            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ///robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ///robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.backLeftMotor.setPower(Math.abs(speed));
            robot.backRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (
                            robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() &&
                                    robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()
                    )) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.backLeftMotor.setPower(leftSpeed);
                robot.backRightMotor.setPower(rightSpeed);
                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);

                //robot.leftDrive.setPower(leftSpeed);
                //robot.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",
                        newFrontLeftTarget,  newFrontRightTarget,
                        newBackLeftTarget, newBackRightTarget
                );
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition()
                        ,
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            telemetry.addData("UUUU", gyro.getIntegratedZValue());
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        telemetry.addData("UNGHI", angle);
        telemetry.update();
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
        robot.backLeftMotor.setPower(leftSpeed);
        robot.backRightMotor.setPower(rightSpeed);
        robot.frontLeftMotor.setPower(leftSpeed);
        robot.frontRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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

    private void rotateLeft(double angle) {
        double actualAngle = gyro.getIntegratedZValue();
        gyroTurn(TURN_SPEED, actualAngle + (360 - angle));
    }

    private void rotateRight(double angle) {
        double actualAngle = gyro.getIntegratedZValue();
        gyroTurn(-TURN_SPEED, actualAngle + angle);
    }

    private void driveBackward(double distance) {
        double actualAngle = gyro.getIntegratedZValue();
        gyroDrive(DRIVE_SPEED, -distance, actualAngle);
    }

    private void driveForward(double distance) {
        double actualAngle = gyro.getIntegratedZValue();
        gyroDrive(DRIVE_SPEED, distance, actualAngle);
    }

    private void lowerRobot(double distance) {
        double actualAngle = gyro.getIntegratedZValue();
        gyroDrive(DRIVE_SPEED, distance, actualAngle);
    }

}

