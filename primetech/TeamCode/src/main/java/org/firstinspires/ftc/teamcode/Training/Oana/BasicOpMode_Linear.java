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

package org.firstinspires.ftc.teamcode.Training.Oana;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_Oana", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor LandingMotor = null;
    private Servo MarkerServo;
    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440 tetrix
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.2;
    static final double HEADING_THRESHOLD = 5;      // As tight as we can make it with an integer gyro



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** AICI BAGI INITIALIZARILE*/
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();
        gyro.resetZAxisIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        LandingMotor = hardwareMap.get(DcMotor.class, "LandingMotor");
        LandingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        LandingMotor.setDirection(DcMotor.Direction.REVERSE);

        //Initializing servos
        MarkerServo = hardwareMap.servo.get(String.valueOf(MarkerServo));

        //setting servo position
        MarkerServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        //******AUTONOMOUS PERIOD ******


        encoderLanding(1, 100, 20);
        sleep(1000);
        driveForward(10);
        Sampling(15);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }


    public void encoderLanding(double speed, double distance, double timeoutS) {
        int LandingTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LandingTarget = LandingMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            LandingMotor.setTargetPosition(LandingTarget);

            // Turn On RUN_TO_POSITION
            LandingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LandingMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (LandingMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to :%7d", LandingTarget);
                telemetry.addData("Path2", "Running at :%7d", LandingMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LandingMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LandingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void Sampling (int timp) {
        int mineralGold = -1;
        TFObjectDetector tfod = null;
        while (timp > 0) {
            if (tfod != null) {
                tfod.activate();
            }
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
                            mineralGold = -1;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            mineralGold = 1;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            mineralGold = 0;
                        }
                    }
                }
                telemetry.update();
            }

            if (mineralGold == -1) {
                rotateLeft(45);
                driveForward(50);
                driveBackward(25);
                rotateRight(45);


            } else if (mineralGold == 1) {
                    rotateLeft(135);
                    driveForward(50);
                    driveBackward(25);
                    rotateRight(135);


            }else if (mineralGold==0) {
                driveForward(50);
                driveBackward(25);
            }


            }

        }


        public void gyroDrive ( double speed,
        double distance,
        double angle){

            //int     newLeftTarget;
            //int     newRightTarget;

            int newBackLeftTarget;
            int newBackRightTarget;
            int newFrontLeftTarget;
            int newFrontRightTarget;

            int moveCounts;
            double max;
            double error;
            double steer;
            double leftSpeed;
            double rightSpeed;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);

                newBackLeftTarget = BackLeftDrive.getCurrentPosition() + moveCounts;
                newBackRightTarget = BackRightDrive.getCurrentPosition() + moveCounts;
                newFrontLeftTarget = FrontLeftDrive.getCurrentPosition() + moveCounts;
                newFrontRightTarget = FrontRightDrive.getCurrentPosition() + moveCounts;

                //newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
                //newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

                BackLeftDrive.setTargetPosition(newBackLeftTarget);
                BackRightDrive.setTargetPosition(newBackRightTarget);
                FrontLeftDrive.setTargetPosition(newFrontLeftTarget);
                FrontRightDrive.setTargetPosition(newFrontRightTarget);

                // Set Target and Turn On RUN_TO_POSITION
                ///robot.leftDrive.setTargetPosition(newLeftTarget);
                ///robot.rightDrive.setTargetPosition(newRightTarget);


                BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                ///robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ///robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                BackLeftDrive.setPower(Math.abs(speed));
                BackRightDrive.setPower(Math.abs(speed));
                FrontLeftDrive.setPower(Math.abs(speed));
                FrontRightDrive.setPower(Math.abs(speed));

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (
                                BackLeftDrive.isBusy() && BackRightDrive.isBusy() &&
                                        FrontLeftDrive.isBusy() && FrontRightDrive.isBusy()
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
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    BackLeftDrive.setPower(leftSpeed);
                    BackRightDrive.setPower(rightSpeed);
                    FrontLeftDrive.setPower(leftSpeed);
                    FrontRightDrive.setPower(rightSpeed);

                    //robot.leftDrive.setPower(leftSpeed);
                    //robot.rightDrive.setPower(rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d:%7d:%7d",
                            newFrontLeftTarget, newFrontRightTarget,
                            newBackLeftTarget, newBackRightTarget
                    );
                    telemetry.addData("Actual", "%7d:%7d:%7d:%7d",
                            FrontLeftDrive.getCurrentPosition(),
                            FrontRightDrive.getCurrentPosition()
                            ,
                            BackLeftDrive.getCurrentPosition(),
                            BackRightDrive.getCurrentPosition()
                    );
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }

                // Stop all motion;
                BackLeftDrive.setPower(0);
                BackRightDrive.setPower(0);
                FrontLeftDrive.setPower(0);
                FrontRightDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        private void driveForward ( double distance){
            double actualAngle = gyro.getIntegratedZValue();
            gyroDrive(DRIVE_SPEED, distance, actualAngle);
        }

        private void rotateLeft ( double angle){
            double actualAngle = gyro.getIntegratedZValue();
            gyroTurn(TURN_SPEED, actualAngle + (360 - angle));
        }
        private void rotateRight ( double angle){
            double actualAngle = gyro.getIntegratedZValue();
            gyroTurn(-TURN_SPEED, actualAngle + angle);
        }
        public void gyroTurn ( double speed, double angle){

            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
                telemetry.addData("UUUU", gyro.getIntegratedZValue());
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
            }
        }
        public double getError ( double targetAngle){

            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - gyro.getIntegratedZValue();
            while (robotError > 180) robotError -= 360;
            while (robotError <= -180) robotError += 360;
            return robotError;
        }

        public double getSteer ( double error, double PCoeff){
            return Range.clip(error * PCoeff, -1, 1);
        }
        boolean onHeading ( double speed, double angle, double PCoeff){
            double error;
            double steer;
            boolean onTarget = false;
            double leftSpeed;
            double rightSpeed;

            // determine turn power based on +/- error
            error = getError(angle);

            telemetry.addData("UNGHI", angle);
            telemetry.update();
            if (Math.abs(error) <= HEADING_THRESHOLD) {
                steer = 0.0;
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            } else {
                steer = getSteer(error, PCoeff);
                rightSpeed = speed * steer;
                leftSpeed = -rightSpeed;
            }


            // Send desired speeds to motors.
            BackLeftDrive.setPower(leftSpeed);
            BackRightDrive.setPower(rightSpeed);
            FrontLeftDrive.setPower(leftSpeed);
            FrontRightDrive.setPower(rightSpeed);

            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

            return onTarget;
        }
        private void driveBackward ( double distance){
            double actualAngle = gyro.getIntegratedZValue();
            gyroDrive(DRIVE_SPEED, -distance, actualAngle);
        }
    }














