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
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class RoverRukusHardwarePushbot extends HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftRearDrive    = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightRearDrive   = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RoverRukusHardwarePushbot(){

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
    }

    public  void setDriveDirection(DcMotor.Direction direction) {

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
 }

