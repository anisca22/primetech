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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareDemoCluj;

import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.LOCK_OPEN;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.MARKER_RELEASED;
import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.MARKER_START;


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

@TeleOp(name="Cluj_Linear", group="Linear Opmode")
@Disabled
public class Main_Demo_Cluj_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double armPower = 0;
    double markerPosition = MARKER_START;
    double lockPosition = LOCK_CLOSED;
    double leftPower = 0;
    double rightPower = 0;
    double drive = 0;
    double turn = 0;
    HardwareDemoCluj robot = new HardwareDemoCluj();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**GAMEPAD 1**/


            //DRIVE
            if (gamepad1.left_stick_y != 0)
                drive = -gamepad1.left_stick_y/2; //FATA SPATE
            else if (gamepad2.left_stick_y != 0)
                drive = -gamepad2.left_stick_y/2; //FATA SPATE
            else
                drive = 0;

            if (gamepad1.right_stick_x != 0)
                turn  =  -gamepad1.right_stick_x/2;  //STANGA DREAPTA
            else if (gamepad2.right_stick_x != 0)
                turn  =  -gamepad2.right_stick_x/2;  //STANGA DREAPTA
            else turn = 0;


            //LOCK SERVO
            if (gamepad1.x)
                lockPosition = LOCK_OPEN;
            else if (gamepad1.y)
                lockPosition = LOCK_CLOSED;

/*
            //RIDICARE BRAT AUTOMAT
            if (gamepad1.dpad_up && gamepad1.a)
            {
                stopItDude();
                lockPosition = LOCK_OPEN;
                robot.lockServo.setPosition(LOCK_OPEN);
                encoderArm(1, 120, -1, 15);
            }

            if (gamepad1.dpad_down && gamepad1.a)
            {
                stopItDude();
                lockPosition = LOCK_OPEN;
                robot.lockServo.setPosition(LOCK_OPEN);
                encoderArm(1, 140, 1, 15);
                lockPosition = LOCK_CLOSED;
                robot.lockServo.setPosition(LOCK_CLOSED);
            }
*/
            /*
            if (gamepad1.b)
            {
                stopItDude();
                encoderArm(0.8, 15, 1, 15);
                lockPosition = LOCK_OPEN;
                robot.lockServo.setPosition(LOCK_OPEN);
                //sleep(1000);
                //encoderArm(1, 15, 1, 15);
                sleep(3000);
                telemetry.addData("DONE", 1);
                telemetry.update();
                //sleep(2000);
                //lockPosition = LOCK_CLOSED;
                //robot.lockServo.setPosition(LOCK_CLOSED);
            }
            */
            /**GAMEPAD 2**/

            //MARKER SERVO
            if (gamepad1.a)
                markerPosition = MARKER_RELEASED;
            else if (gamepad1.b)
                markerPosition = MARKER_START;

            //RIDICARE BRAT MANUAL
            if (gamepad1.dpad_up)
                armPower = 1;
            else if (gamepad1.dpad_down)
                armPower = -1;
            else if (!(gamepad1.dpad_up ||gamepad1.dpad_down))
                armPower = 0;

            /*
            else if (gamepad2.dpad_up)
                armPower = 0.6;
            else if (gamepad2.dpad_down)
                armPower = -0.6;
            else armPower = 0;
            */
            robot.armMotor.setPower(armPower);

            //CALCULARE PUTERE
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // PUTERE LA MOTOARE DE DEPLASARE
            robot.backLeftMotor.setPower(leftPower);
            robot.frontLeftMotor.setPower(leftPower);
            robot.backRightMotor.setPower(rightPower);
            robot.frontRightMotor.setPower(rightPower);

            //POZITIONARE SERVOURI
            robot.markerServo.setPosition(markerPosition);
            robot.lockServo.setPosition(lockPosition);

        }

        stopItDude();

    }

    public void stopItDude(){
        //OPRIT MOTOARE DE DEPLASARE
        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

        //OPRIT MOTORUL DE RIDICARE
        robot.armMotor.setPower(0);

        //SETARE POZITIE SERVOURI DE SFARSIT
        robot.lockServo.setPosition(LOCK_CLOSED);
        robot.markerServo.setPosition(MARKER_START);
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
        }
    }
}
