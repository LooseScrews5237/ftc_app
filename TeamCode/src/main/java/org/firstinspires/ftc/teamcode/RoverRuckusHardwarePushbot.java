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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class RoverRuckusHardwarePushbot extends HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftRearDrive    = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightRearDrive   = null;
    public DcMotor liftMotor        = null;

    public ColorSensor colorSensor  = null;

    /* Public constants */
    public static final double COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* local OpMode members. */
    private HardwareMap hwMap = null;

    /* Constructor */
    public RoverRuckusHardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive   = hwMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive  = hwMap.get(DcMotor.class, "right_rear_drive");

        // Set the default drive direction
        setDriveDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        stopDriveMotors();
        // Set all motors to run without encoders.
        setDriveMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set lift motor properties
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // color sensor
        colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
        // Turn off the LED by default so we don't burn it out.
        colorSensor.enableLed(false);
    }

    public void setDriveDirection(DcMotor.Direction direction) {

        DcMotor.Direction inverseDirection = null;
        switch (direction) {
            case FORWARD:
                inverseDirection = DcMotor.Direction.REVERSE;
                break;
            case REVERSE:
                inverseDirection = DcMotor.Direction.FORWARD;
                break;
        }

        leftFrontDrive.setDirection(direction);
        leftRearDrive.setDirection(direction);
        rightFrontDrive.setDirection(inverseDirection);
        rightRearDrive.setDirection(inverseDirection);
    }

    public void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public void setDriveMotorRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftRearDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightRearDrive.setMode(mode);
    }

    public void setDriveMotorPower(double left, double right) {
        leftFrontDrive.setPower(left);
        leftRearDrive.setPower(-left);
        rightFrontDrive.setPower(right);
        rightRearDrive.setPower(-right);
    }

    public void resetEncoders() {
        setDriveMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setNewTargetPosition(double leftInches, double rightInches) {
        int newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newLeftRearTarget  = leftRearDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        int newRightRearTarget  = rightRearDrive.getTargetPosition() + (int)(rightInches * COUNTS_PER_INCH);

        leftFrontDrive.setTargetPosition(newLeftFrontTarget);
        leftRearDrive.setTargetPosition(newLeftRearTarget);
        rightFrontDrive.setTargetPosition(newRightFrontTarget);
        rightRearDrive.setTargetPosition(newRightRearTarget);
    }

    public void raiseLiftArm(){
        liftMotor.setPower(.25);
    }

    public void stopLiftArm(){
        liftMotor.setPower(0);
    }

    public void lowerLiftArm(){
        liftMotor.setPower(-0.5);
    }
 }
