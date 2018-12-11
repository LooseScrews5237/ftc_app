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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Pushbot: Crater with Mineral", group="Pushbot")
public class Autonomous_Crater_WithMineralDetection extends LinearOpMode {

    /* Declare OpMode members. */
    private RoverRuckusHardwarePushbot robot = new RoverRuckusHardwarePushbot(this, telemetry);   // Use a Pushbot's hardware
    private double TURBO_BOOST = 0.8;
    private double SAMPLE_TIMEOUT = 5;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.resetEncoders();

        // Object Detection
        robot.initializeObjectDetection(telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.lowerToGround( -8, 5.0);
        robot.activateObjectDetection();

        // Sample Minerals
        String goldMineralPosition = "Unknown";
        int goldMineralX = -1;
        ElapsedTime runtime = new ElapsedTime();
        while(opModeIsActive() && goldMineralX == -1 && runtime.seconds() < SAMPLE_TIMEOUT) {
            goldMineralX = robot.getGoldMineralLeftX(telemetry);
        }
        robot.stopObjectDetection();
        //sleep(5000);

        if (goldMineralX != -1) {
            if (goldMineralX < 100) {
                goldMineralPosition = "Left";
            }
            else if (goldMineralX > 500) {
                goldMineralPosition = "Center";
            }
        }

        // Unhook from lander
        robot.drive(4, 4, 4.0);

        // Check for mineral position
        switch (goldMineralPosition) {
            case "Left":
                robot.turn(-45, 4.0);
                // Knock off mineral
                robot.drive(-4, -4, 2);
                robot.drive(-18, -18, 10);
                robot.turn(-45, 2);
                robot.drive(-16, -16, 10);

                // Drive to wall
                robot.drive(16, 16, 10);
                robot.turn(45, 4);
                robot.drive(-26, -26, 15);
                break;
            case "Right": // Not currently used
                robot.turn(-45, 4.0);
                robot.drive(-4, -4, 2);
                robot.turn(-90, 4.0);
                robot.drive(-20, -20, 10);
                robot.turn(45, 4.0);
                robot.drive(-12, -12, 10);
                break;
            case "Center":
                robot.turn(-45, 4.0);
                // Knock off mineral
                robot.drive(-4, -4, 2);
                robot.turn(-45, 4.0);
                robot.drive(-26, -26, 18);

                // Drive to wall
                robot.drive(25, 25, 12);
                robot.turn(47, 8);
                robot.drive(-40, -40, 10);
                break;
            case "Unknown":
                // Unhook from lander
                robot.turn(-45, 4.0);
                robot.drive(-4, -4, 2);

                // Try to knock off the right position
                robot.turn(-90, 4);
                robot.drive(-12, -12, 5);
                robot.turn(45, 4);
                robot.drive(-16, -16, 5);

                // Drive to wall
                robot.drive(16, 16, 5);
                robot.turn(45, 4);
                robot.drive(-40, -40, 10);
                robot.turn(-45, 4);
                robot.drive(-40, -40, 10);

                //robot.drive(-40, -40, TURBO_BOOST, 8.0);
                //robot.drive(-6, -6, 6);
                break;
        }

        // Drive to depot
        robot.turn(-98, 5.0);

        //drive to depot
        robot.drive(50, 50, TURBO_BOOST, 9.0);
        robot.drive(4, 4, 4);

        //drop scrubbing bubbles
        robot.runBeaterBar(DcMotorSimple.Direction.FORWARD);
        sleep(1000);
        robot.stopBeaterBar();

        //drive to crater
        robot.drive(-66, -66, TURBO_BOOST, 16);
        robot.drive(-4, -4, 4);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
