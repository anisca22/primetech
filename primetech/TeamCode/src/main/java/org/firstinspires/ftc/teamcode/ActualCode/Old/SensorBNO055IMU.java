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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.HardwareDemoCluj;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.HardwareDemoCluj.TURN_SPEED;

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Sensor: BNO055 IMU Test", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class SensorBNO055IMU extends LinearOpMode  {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;
    HardwareDemoCluj robot = new HardwareDemoCluj();
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
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

        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //sleep(1000);
            double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            double roll = noformatAngle(angles.angleUnit, angles.secondAngle);

            telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
            telemetry.addData("heading2", heading);
            telemetry.addData("roll2", roll);
            telemetry.update();

            if (gamepad1.a)rotateLeft(45);
            else if (gamepad1.b)rotateRight(45);
        }
    }

    private void rotation(double target)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);

        while ((Math.abs(heading - target) > 3) && opModeIsActive())
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
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            telemetry.addData("Heading", heading);
            telemetry.addData("NewDir", target);
            telemetry.update();
            sleep(1);
        }
        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

    }

    private void rotateLeft (double angle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Heading", heading);
        telemetry.addData("NewDir", heading - angle);
        telemetry.update();
        sleep(1000);
        rotation(heading + angle);
    }

    private void rotateRight (double angle){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Heading", heading);
        telemetry.addData("NewDir", heading - angle);
        telemetry.update();
        sleep(1000);
        rotation(heading - angle);
    }

    /*
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
            ///telemetry.addData("UUUU", heading);
            // Update telemetry & Allow time for other processes to run.
            ///telemetry.update();
        }
    }


    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = noformatAngle(angles.angleUnit, angles.firstAngle);
        error = angle - heading;

        telemetry.addData("Heading", heading);
        telemetry.addData("UNGHI", angle);
        telemetry.addData("Error", Math.abs(error));
        telemetry.addData("Headingth", HEADING_THRESHOLD);
        telemetry.update();
        //sleep(500);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else if (error < 0)
        {
            steer = Range.clip(error * 0.1, -1, 1);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }
        else
        {
            steer = Range.clip(error * 0.1, -1, 1);
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
    */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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

}