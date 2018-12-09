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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareTurda
{
    ///DECLARARE MOTOARE
    public DcMotor  frontLeftMotor;
    public DcMotor  backLeftMotor;
    public DcMotor  frontRightMotor;
    public DcMotor  backRightMotor;

    public DcMotor  armMotor;
    public DcMotor  rotatorMotor;

    public Servo marker;

    ///DECLARARE SERVOURI

    public final static double MAX_PUTERE = 1.0;
    public final static double MIN_PUTERE = -1.0;

    public final static double MAX_CAPAC = 1.0;
    public final static double INIT_CAPAC = 0.5;
    public final static double MIN_CAPAC = 0;

    public final static double ARM_POWER = 1;
    public final static double SLIDER_POWER = 0.4;
    public final static double COLECTOR_POWER = 1;
    public final static double ROTATOR_POWER = 0.3;

    public final static double MARKER_START = 1;
    public final static double MARKER_DOWN = 0.2;
    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTurda() {}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // DEFINIRE MOTOARE
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        armMotor = hwMap.get(DcMotor.class, "armMotor");
        rotatorMotor = hwMap.get(DcMotor.class, "rotatorMotor");

        marker = hwMap.get(Servo.class, "marker");
        // SETARE DIRECTIE
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        rotatorMotor.setDirection(DcMotor.Direction.FORWARD);

        // PUTERE 0
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        armMotor.setPower(0);
        rotatorMotor.setPower(0);
        marker.setPosition(MARKER_START);
        //DEFINIRE SERVOURI
    }
}
