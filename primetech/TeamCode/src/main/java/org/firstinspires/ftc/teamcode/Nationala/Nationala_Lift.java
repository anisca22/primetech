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

package org.firstinspires.ftc.teamcode.Nationala;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Nationala.HardwareNationalTest.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.Nationala.HardwareNationalTest.diagonalValue;
import static org.firstinspires.ftc.teamcode.Nationala.HardwareNationalTest.driveValue;
import static org.firstinspires.ftc.teamcode.Nationala.HardwareNationalTest.liftValue;
import static org.firstinspires.ftc.teamcode.Nationala.HardwareNationalTest.strafeValue;
import static org.firstinspires.ftc.teamcode.Nationala.HardwareNationalTest.turnValue;
import static org.firstinspires.ftc.teamcode.Regio.HardwareFinal.COUNTS_PER_MM;

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

@Autonomous(name="Lift ", group="Pushbot")
///@Disabled
public class Nationala_Lift extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareNationalTest robot = new HardwareNationalTest();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("HI!", " DON'T TOUCH ANYTHING");
        telemetry.update();

        /***    INIT STARTS HERE                           ***/
        /***                INIT STARTS HERE               ***/
        /***                            INIT STARTS HERE   ***/

        /***INIT***/
        robot.init(hardwareMap);


        /***ENCODERS***/
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Status:", "Wait for start");
            telemetry.update();
        }

        runtime.reset();

        /***    AUTONOMUS STARTS HERE                           ***/
        /***                AUTONOMUS STARTS HERE               ***/
        /***                            AUTONOMUS STARTS HERE   ***/

        encoderLift(1, 15, 1, 15);

        stopAllMotion();

        /***  AUTONOMUS ENDS HERE                             ***/
        /***                AUTONOMUS ENDS HERE               ***/
        /***                            AUTONOMUS ENDS HERE   ***/
    }

    public void encoderLift(double speed, double distance, double direction, double timeoutS) {
        int liftTarget;

        if (direction == -1)
            robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);
        else if (direction == 1)
            robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
        // Ensure that the opmode is still active
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive() && !isStopRequested() && !(gamepad1.dpad_up && gamepad1.b)) {

            // Determine new target position, and pass to motor controller
            liftTarget = robot.liftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * liftValue);

            robot.liftMotor.setTargetPosition(liftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.liftMotor.isBusy())
                    && !isStopRequested())
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to :%7d", liftTarget);
                telemetry.addData("Path2",  "Running at :%7d", robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);

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
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * driveValue);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * driveValue);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * driveValue);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * driveValue);

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

    public void encoderTurn(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);

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
            robot.backRightMotor.setPower(speed);
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

    public void encoderStrafe(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * strafeValue);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * strafeValue);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * strafeValue);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * strafeValue);

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

            double frontLeftDistance =  + distance * COUNTS_PER_MM * diagonalValue;
            double backRightDistance =  - distance * COUNTS_PER_MM * diagonalValue;

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

            double frontRightDistance =  - distance * COUNTS_PER_MM * diagonalValue;
            double backLeftDistance   =  + distance * COUNTS_PER_MM * diagonalValue;

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

    public void rotateRight(double angle)
    {
        encoderTurn(TURN_SPEED, angle, 15);
    }

    public void stopAllMotion() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.stringMotor.setPower(0);
        robot.liftMotor.setPower(0);
        robot.armMotor.setPower(0);
    }

}

