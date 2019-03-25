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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareTest;

import static org.firstinspires.ftc.teamcode.HardwareTest.LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.HardwareTest.LOCK_OPEN;
import static org.firstinspires.ftc.teamcode.HardwareTest.MARKER_DOWN;
import static org.firstinspires.ftc.teamcode.HardwareTest.MARKER_START;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class Teleop_Oana_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double armPower = 0;
    double markerPosition = MARKER_START;
    double lockPosition = LOCK_OPEN;
    HardwareTest robot = new HardwareTest();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        //DECLARARE DE VARIABILE
        double leftPower;
        double rightPower;


        //CITIRE VARIABILE
        double drive = -gamepad1.left_stick_y; //FATA SPATE
        double turn  =  -gamepad1.right_stick_x;  //STANGA DREAPTA

        //CALCULARE PUTERE
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        //MARKER SERVO
        if (gamepad1.dpad_down)
        {
            markerPosition = MARKER_DOWN;
        }

        if (gamepad1.dpad_up)
        {
            markerPosition = MARKER_START;
        }

        //LOCK SERVO
        if (gamepad2.x)
        {
            lockPosition = LOCK_OPEN;
        }

        if (gamepad2.y)
        {
            lockPosition = LOCK_CLOSED;
        }

        //RIDICARE BRAT
        if (gamepad2.dpad_up)
        {
            armPower = 0.8;
        }
        else if (gamepad2.dpad_down)
        {
            armPower = -0.8;
        }
        else armPower = 0;



        // PUTERE LA MOTOARE DE DEPLASARE
        robot.backLeftMotor.setPower(leftPower);
        robot.frontLeftMotor.setPower(leftPower);
        robot.backRightMotor.setPower(rightPower);
        robot.frontRightMotor.setPower(rightPower);

        // PUTERE LA MOTORUL PENTRU BRAT
        robot.armMotor.setPower(armPower);

        //POZITIONARE SERVOURI
        robot.markerServo.setPosition(markerPosition);
        robot.lockServo.setPosition(lockPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //OPRIT MOTOARE DE DEPLASARE
        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

        //OPRIT MOTORUL DE RIDICARE
        robot.armMotor.setPower(0);

        //SETARE POZITIE SERVOURI DE SFARSIT
        robot.lockServo.setPosition(LOCK_OPEN);
        robot.markerServo.setPosition(MARKER_START);

    }

}
