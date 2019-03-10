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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
public class HardwareFinal
{
    ///DECLARARE MOTOARE
    public DcMotor  frontLeftMotor;
    public DcMotor  backLeftMotor;
    public DcMotor  frontRightMotor;
    public DcMotor  backRightMotor;

    public DcMotor  armMotor;
    public DcMotor  extenderMotor;
    public DcMotor  collectionMotor;

    ///DECLARARE SERVOURI
    public Servo latchServo;
    public Servo mineralServo;
    //public Servo transmissionServo;

    DigitalChannel digitalTouch;

    public static final double      DRIVE_SPEED = 1;
    public static final double      TURN_SPEED = 0.5;

    ///LATCH LOCK VARIABLES
    public static final double      LATCH_LOCK_CLOSED = 0.9;
    public static final double      LATCH_LOCK_OPEN = 1;

    ///MINERAL LOCK VARIABLES
    public static final double      MINERAL_LOCK_CLOSED = 0.5;
    public static final double      MINERAL_LOCK_OPEN = 1;

    ///TRANSMISSION LOCK VARIABLES
    //public static final double      TRANSMISSION_LOCK_LATCH = 0;
    //public static final double      TRANSMISSION_LOCK_MINERAL = 1;

    ///ENCODER VARIABLES
    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder 1440 tetrix
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_MM   = 4.0 * 25.4;     // For figuring circumference
    public static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareFinal() {}

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
        extenderMotor = hwMap.get(DcMotor.class, "extenderMotor");
        collectionMotor = hwMap.get(DcMotor.class, "collectionMotor");

        //DEFINIRE SERVOURI
        latchServo = hwMap.get(Servo.class, "latchServo");
        mineralServo = hwMap.get(Servo.class, "mineralServo");
        //transmissionServo = hwMap.get(Servo.class, "transmissionServo");

        // SETARE DIRECTIE
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // PUTERE 0
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        armMotor.setPower(0);

        // SETARE POZITIE SERVO
        latchServo.setPosition(LATCH_LOCK_CLOSED);
        mineralServo.setPosition(MINERAL_LOCK_CLOSED);
        //transmissionServo.setPosition(TRANSMISSION_LOCK_LATCH);

        digitalTouch = hwMap.get(DigitalChannel.class, "touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }

}
