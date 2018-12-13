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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    // Motor variables
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftRearDrive    = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightRearDrive   = null;
    public DcMotor liftMotor        = null;
    public DcMotor beaterBarMotor   = null;
    public DcMotor armPivotMotor    = null;
    public DcMotor armExtensionMotor = null;
    public ColorSensor colorSensor  = null;

    // Hardware Status Flags
    public boolean LiftMotorEnabled = true;
    public boolean ColorSensorEnabled = true;
    public boolean BeaterBarEnabled = true;
    public boolean DriveMotorsEnabled = true;
    public boolean PivotMotorsEnabled = true;
    public boolean ArmExtensionMotorEnabled = true;


    /* Configuration constants */
    private static final double ROBOT_DIAMETER_INCHES   = 17.5;
    public static final double INCHES_PER_DEGREE       = (ROBOT_DIAMETER_INCHES * Math.PI) / 360; // Number of inches in 1 degree = ~0.1527 inch
    private static final double COUNTS_PER_MOTOR_REV    = 1440;
    private static final double DRIVE_GEAR_REDUCTION    = 0.40;
    private static final double LIFT_GEAR_REDUCTION     = 4.0;
    private static final double LIFT_DIAMETER_INCHES    = 1.25;
    private static final double PIVOT_DIAMETER_INCHES   = 2.0;
    private static final double WHEEL_DIAMETER_INCHES   = 3.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double COUNTS_PER_INCH_LIFT    = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION) /
                                                            (LIFT_DIAMETER_INCHES * 3.1415);
    public static final double COUNTS_PER_DEGREE_ARM_PIVOT = (PIVOT_DIAMETER_INCHES * Math.PI) / 360;

    /* Motor configuration values */
    private final double ARM_PIVOT_SPEED = 1.0;
    private final double ARM_EXTENSION_SPEED = 1.0;
    private final double LIFT_ARM_UP_POWER = 0.9;
    private final double LIFT_ARM_DOWN_POWER = 1.0;
    private final double BEATER_BAR_POWER = 0.75;
    public double AUTONOMOUS_DRIVE_SPEED = 0.2;

    /* Object Detection */
    private static final String VUFORIA_KEY = "AUgUe7L/////AAAAGfkap25dZEgYr4vc2DlGeBcCrt01n02Uo2jVtP0xRrm1VxUO2AJ+M174bXduQ7oxCYVJBUtlFCPq2yquilKmT3L9MN67UMdPHPYqFVS8sPGT7teMzq0vkU2QFGVQrtoHLffAIz5fH+gyAx951lZ+Upx3Zf8+B9BkvV7+1dpn5PC6EVDodaqYyEVzCv86BIIUiXteJ7zItKuwm+vHEXVWyLDRv7YOnkIlf/gO6192NFXRLU1xkYkxtX9wH+qlb+5SRBoiGh54JzpqiRqZQGJTK1k/AC8g9ndGwHYNol9lJcI5MpZzSuCPMmWt97WoO3WHYl7AE/mTKiNFPIqpfulua1GAZH99XuNgAXJtCiehEfeP";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final double CONFIDENCE_LEVEL = .5;


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /* local OpMode members. */
    private HardwareMap hwMap = null;
    private OpMode opMode;
    private Telemetry telemetry;


    /* Constructor */
    public RoverRuckusHardwarePushbot(OpMode opMode, Telemetry telemetry){
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Drive Motors
        try {
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
        catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            DriveMotorsEnabled = false;
        }

        // Lift Motor
        try {
            liftMotor = hwMap.get(DcMotor.class, "lift_motor");
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            LiftMotorEnabled = false;
        }

        // Color sensor
        try {
            colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
            // Turn off the LED by default so we don't burn it out.
            colorSensor.enableLed(false);
        }
        catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            ColorSensorEnabled = false;
        }

        // Arm Extension Motor
        try {
            armExtensionMotor = hwMap.get(DcMotor.class, "arm_extension_motor");
            armExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtensionMotor.setPower(0);
            armExtensionMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        catch(Exception e) {
            telemetry.addData("Error", e.getMessage());
            ArmExtensionMotorEnabled = false;
        }

        // Arm Pivot Motor
        try {
            armPivotMotor = hwMap.get(DcMotor.class, "arm_pivot_motor");
            armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armPivotMotor.setPower(0);
            armPivotMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            PivotMotorsEnabled = false;
        }

        // Beater bar
        try {
            beaterBarMotor = hwMap.get(DcMotor.class, "beaterbar_motor");
        }
        catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            BeaterBarEnabled = false;
        }
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

        leftFrontDrive.setDirection(inverseDirection);
        leftRearDrive.setDirection(inverseDirection);
        rightFrontDrive.setDirection(direction);
        rightRearDrive.setDirection(direction);
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

    public void raiseLiftArm(){
        liftMotor.setPower(-LIFT_ARM_UP_POWER);
    }

    public void stopLiftArm(){
        liftMotor.setPower(0);
    }

    public void lowerLiftArm(){
        liftMotor.setPower(LIFT_ARM_DOWN_POWER);
    }

    public void pivotArmForward() {
        armPivotMotor.setPower(ARM_PIVOT_SPEED);
    }

    public void pivotArmBackward() {
        armPivotMotor.setPower(-ARM_PIVOT_SPEED);
    }

    public void pivotArmStop() {
        armPivotMotor.setPower(0);
    }

    public void extensionArmExtend() {
        armExtensionMotor.setPower(ARM_EXTENSION_SPEED);
    }

    public void extensionArmRetract() {
        armExtensionMotor.setPower(-ARM_EXTENSION_SPEED);
    }

    public void extensionArmStop() {
        armExtensionMotor.setPower(0);
    }

    public void runBeaterBar (DcMotor.Direction direction) {
        beaterBarMotor.setDirection(direction);
        beaterBarMotor.setPower (BEATER_BAR_POWER);
    }

    public void stopBeaterBar() {
        beaterBarMotor.setPower(0);
    }

    /*************** Autonomous Functions ************************/

    public void resetEncoders() {
        setDriveMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setNewTargetPosition(double leftInches, double rightInches, Telemetry telemetry) {
        telemetry.addData("position", "current: %7d: %7d",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        int newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newLeftRearTarget  = leftRearDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        int newRightRearTarget  = rightRearDrive.getTargetPosition() + (int)(rightInches * COUNTS_PER_INCH);

        telemetry.addData("newPosition", "target: %7d: %7d", newLeftFrontTarget, newRightFrontTarget);
        leftFrontDrive.setTargetPosition(newLeftFrontTarget);
        leftRearDrive.setTargetPosition(newLeftRearTarget);
        rightFrontDrive.setTargetPosition(newRightFrontTarget);
        rightRearDrive.setTargetPosition(newRightRearTarget);
    }

    public boolean motorsAreBusy() {
        return leftFrontDrive.isBusy() && leftRearDrive.isBusy() &&
                rightFrontDrive.isBusy() && rightRearDrive.isBusy();
    }

    public void resetLiftMotorEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftMotorTargetPosition(double inchesToTravel) {
        int targetPosition = liftMotor.getCurrentPosition() + (int)(inchesToTravel * COUNTS_PER_INCH_LIFT);
        liftMotor.setTargetPosition(targetPosition);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void drive(double leftInches, double rightInches, double speed, double timeoutS) {

        // Ensure that the opmode is still active
        if (((LinearOpMode)opMode).opModeIsActive()) {
            ElapsedTime runtime = new ElapsedTime();
            // Determine new target position, and pass to motor controller
            setNewTargetPosition(leftInches, rightInches, telemetry);

            // Turn On RUN_TO_POSITION
            setDriveMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            double driveSpeed = Math.abs(speed);
            setDriveMotorPower(driveSpeed, driveSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (((LinearOpMode)opMode).opModeIsActive() && (runtime.seconds() < timeoutS) && motorsAreBusy()) {
                telemetry.addData("current", "%7d | %7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopDriveMotors();

            // Turn off RUN_TO_POSITION
            setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void drive(double leftInches, double rightInches, double timeoutS) {
        drive(leftInches,rightInches, AUTONOMOUS_DRIVE_SPEED, timeoutS);
    }

    public void turn(int degrees, double driveSpeed, double timeoutS) {
        double inches = degrees * INCHES_PER_DEGREE;
        drive(inches, -inches, driveSpeed, timeoutS);
    }

    public void turn(int degrees, double timeoutS) {
        double inches = degrees * INCHES_PER_DEGREE;
        drive(inches, -inches, timeoutS);
    }

    public void lowerToGround(double inches, double timeoutS) {
        if (((LinearOpMode)opMode).opModeIsActive())
        {
            ElapsedTime runtime = new ElapsedTime();
            resetLiftMotorEncoder();
            setLiftMotorTargetPosition(inches);
            lowerLiftArm();

            runtime.reset();

            while (((LinearOpMode)opMode).opModeIsActive() && liftMotor.isBusy() && (runtime.seconds() < timeoutS)) {
                telemetry.addData("Lift Position", "%d7", liftMotor.getCurrentPosition());
            }

            stopLiftArm();
        }
    }

    public void pivotArmWithEncoder(double degrees, double timeoutS) {
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTargetPosition = armPivotMotor.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE_ARM_PIVOT);
        armPivotMotor.setTargetPosition(newTargetPosition);

        if (((LinearOpMode)opMode).opModeIsActive()) {
            ElapsedTime runtime = new ElapsedTime();
            armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivotArmForward();
            while (((LinearOpMode)opMode).opModeIsActive() && (runtime.seconds() < timeoutS) && motorsAreBusy()) {
                telemetry.addData("current", "%7d", armPivotMotor.getCurrentPosition());
                telemetry.update();
            }
            pivotArmStop();
        }
    }

    /* Object Detection Methods */

    public void initializeObjectDetection(Telemetry telemetry) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    /** Activate Tensor Flow Object Detection. */
    public void activateObjectDetection() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void stopObjectDetection() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public boolean objectDetectionIsActive() {
        return  tfod != null && tfod.getUpdatedRecognitions() != null;
    }

    public int getGoldMineralLeftX(Telemetry telemetry) {
        int goldMineralX = -1;
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    if (recognition.getConfidence() > CONFIDENCE_LEVEL) {
                        goldMineralX = (int) recognition.getLeft();
                        telemetry.addData("GoldMineral Left", "%7d", goldMineralX);
                    }
                }
            }
            telemetry.update();
        }

        return goldMineralX;
    }

    public String tryGetGoldMineralPosition() {
        String goldPosition = null;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int count = updatedRecognitions.size();
                telemetry.addData("# Object Detected", count);

                if (count == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Label", recognition.getLabel());
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else if (silverMineral2X == -1) {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1) {
                        if (goldMineralX < silverMineral1X) {
                            goldPosition = "Left";
                        } else if (goldMineralX > silverMineral1X) {
                            goldPosition = "Center";
                        } else {
                            goldPosition = "Left";
                        }
                        telemetry.addData("Gold Mineral Position", goldPosition);
                    }
                    else {
                        telemetry.addData("Error", "Could not find gold mineral!");
                    }
                }
                else {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            telemetry.addData("Gold POS", recognition.getLeft());
                        }
                    }
                }
                telemetry.update();
            }
        }

        return goldPosition;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
 }
