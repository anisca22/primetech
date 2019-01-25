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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareRobot;

import static org.firstinspires.ftc.teamcode.HardwareRobot.ARM_POWER;
import static org.firstinspires.ftc.teamcode.HardwareRobot.COLECTOR_POWER;
import static org.firstinspires.ftc.teamcode.HardwareRobot.INIT_CAPAC;
import static org.firstinspires.ftc.teamcode.HardwareRobot.MAX_CAPAC;
import static org.firstinspires.ftc.teamcode.HardwareRobot.MAX_PUTERE;
import static org.firstinspires.ftc.teamcode.HardwareRobot.MIN_CAPAC;
import static org.firstinspires.ftc.teamcode.HardwareRobot.MIN_PUTERE;
import static org.firstinspires.ftc.teamcode.HardwareRobot.ROTATOR_POWER;
import static org.firstinspires.ftc.teamcode.HardwareRobot.SLIDER_POWER;

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

@TeleOp(name="Main TeleOP", group="Iterative Opmode")
@Disabled
public class Main extends OpMode
{
    // Declare OpMode members.
    double armPower = 0;
    double rotatorPower = 0;
    double sliderPower = 0;
    double colectorPower = 0;
    double capacPosition = INIT_CAPAC;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareRobot robot = new HardwareRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        ///CITIRE VARIABILE
        double y = -gamepad1.left_stick_y; ///FATA SPATE
        double x  =  gamepad1.right_stick_x; /// STANGA DREAPTA

        ///CALCULARE PUTERE
        double leftPower = Range.clip(y + x, MIN_PUTERE, MAX_PUTERE);
        double rightPower = Range.clip(y - x, MIN_PUTERE, MAX_PUTERE);

        if (gamepad1.dpad_up)
            armPower = ARM_POWER;
        else if (gamepad1.dpad_down)
            armPower = -ARM_POWER;
        else
            armPower = 0;

        if (gamepad1.dpad_left)
            sliderPower = SLIDER_POWER;
        else if (gamepad1.dpad_right)
            sliderPower = -SLIDER_POWER;
        else
            sliderPower = 0;

        if (gamepad1.left_bumper)
            colectorPower = COLECTOR_POWER;
        else if (gamepad1.right_bumper)
            colectorPower = -COLECTOR_POWER;
        else colectorPower = 0;

        if (gamepad1.a)
            rotatorPower = ROTATOR_POWER;
        else if (gamepad1.b)
            rotatorPower = -ROTATOR_POWER;
        else rotatorPower = 0;

        if (gamepad1.y)
            capacPosition = INIT_CAPAC;
        else if (gamepad1.x)
            capacPosition = MAX_CAPAC;
        //SETARE PUTERE
        robot.frontLeftMotor.setPower(leftPower);
        robot.backLeftMotor.setPower(leftPower);
        robot.frontRightMotor.setPower(rightPower);
        robot.backRightMotor.setPower(rightPower);

        robot.armMotor.setPower(armPower);
        robot.sliderMotor.setPower(sliderPower);
        robot.colectorMotor.setPower(colectorPower);
        robot.rotatorMotor.setPower(rotatorPower);

        robot.capac.setPosition(capacPosition);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

}
