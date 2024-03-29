/* Copyright (c) 2019 FIRST. All rights reserved.
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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoHighLeft", group = "Iterative Opmode")
@Disabled
public class AutoHighLeft extends OpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .1
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotor = null;
    private Servo armServo = null;
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;
    BNO055IMU imu;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    int count = 0;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AVjWNQH/////AAABmfTAg894fEL/rQj8b+u8l7Qw34HtrMgOnmf6xTlvK+Afn5EmrjzwTJ7/aTw0eGzNWdd0u+f1Rv8T8gH+kytJmYIPDIKOiLHuHJvMc0lwvEgKfiE33bZAoGW/ZoX2kyIHVWgr9I2yNKtE/SS4Ik4imJIJbe4QwFBMed02dz05R+j6Oi3wW4CutaknKYb5BH68RviV8b98QDV6FUwLa0u+biIkAEciicgHoQuDWCA2hrByaIEEm4XgXCF0H37hyv0Ra7SZsm6YMcTC2mNSIblMD77iL7MFyUoFdoQnykv+KJiNelhdjfswwCQWszNLYpqzwo56nAimSAr8s4C7Cub1GAlYVfq5XnG/7ZWH0oSg1x8T";
    //" -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private boolean middle = true;
    private boolean left = false;
    private boolean right = false;
    private final double TicksPerRev = 1425.1;
    private final double WheelDiameter = 3.75;
    private final double WheelCircumference = WheelDiameter * Math.PI;
    private final int TicksPerInch = (int) (TicksPerRev / WheelCircumference);
    private static final double OPEN_CLAW_POS = 0.74;
    private static final double CLOSED_CLAW_POS = 0.60;
    private int pos = 2;
    private int counter = 0;
    private double armPos = 0.5;
    private double backArmPos = 0.0;
    private double frontArmPos = .950;

    @Override
    public void init() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        leftFront = hardwareMap.get(DcMotor.class, "left_FrontDrive");
        leftBack = hardwareMap.get(DcMotor.class, "left_BackDrive");
        rightFront = hardwareMap.get(DcMotor.class, "right_FrontDrive");
        rightBack = hardwareMap.get(DcMotor.class, "right_BackDrive");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightClawServo.setDirection(Servo.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    @Override
    public void init_loop() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());
                    telemetry.addData("image", recognition.getLabel());
                    if (recognition.getLabel().equals("1 Bolt")) {
                        left = true;
                        middle = false;
                        right = false;
                        pos = 1;
                    } else if (recognition.getLabel().equals("3 Panel")) {
                        left = false;
                        middle = false;
                        right = true;
                        pos = 3;
                    }
                }
            }
        }
    }

    @Override
    public void start() {
        runtime.reset();
        armPos = backArmPos;
    }


    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        switch (pos) {
            case 1:
                switch (counter) {
                    case 0:
                        arm(backArmPos);
                        claw(CLOSED_CLAW_POS);
                        if (runtime.seconds() > 2) {
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 1:
                        liftMotor.setTargetPosition(1200);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 2) {
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 2:
                        setTargetPos(3300, 3300);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.5, 0.5);
                        if (runtime.seconds() > 6){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 3:
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        counter++;
                        break;
                    case 4:
                        strafeRight(900, 0.4);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 5:
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        mode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                        counter++;
                        break;
                    case 6:
                        liftMotor.setTargetPosition(5500);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 4) {

                            counter++;
                        }
                        break;
                    case 7:
                        runtime.reset();
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        mode(DcMotor.RunMode.RUN_USING_ENCODER);
                        counter++;

                        break;
                    case 8:
                        setTargetPos(-470, -470);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.2,0.2);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 9:
                        liftMotor.setTargetPosition(1200);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 10:
                        claw(OPEN_CLAW_POS);
                        runtime.reset();
                        counter++;
                        break;
                    case 11:
                        setTargetPos(470, 470);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.2,0.2);
                        if (runtime.seconds() > 1){
                            mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 12:
                        strafeLeft(2600);
                        break;
                }
                break;
            case 2:
                switch (counter) {
                    case 0:
                        arm(backArmPos);
                        claw(CLOSED_CLAW_POS);
                        if (runtime.seconds() > 2) {
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 1:
                        liftMotor.setTargetPosition(1200);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 2) {
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 2:
                        setTargetPos(3300, 3300);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.5, 0.5);
                        if (runtime.seconds() > 4){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 3:
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        counter++;
                        break;
                    case 4:
                        strafeRight(900, 0.4);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 5:
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        mode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                        counter++;
                        break;
                    case 6:
                        liftMotor.setTargetPosition(7000);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 6) {
                            counter++;
                        }
                        break;
                    case 7:
                        runtime.reset();
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        mode(DcMotor.RunMode.RUN_USING_ENCODER);
                        counter++;

                        break;
                    case 8:
                        setTargetPos(470, 470);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.2,0.2);
                        if (runtime.seconds() > 1){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 9:
                        liftMotor.setTargetPosition(1200);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 10:
                        claw(OPEN_CLAW_POS);
                        runtime.reset();
                        counter++;
                        break;
                    case 11:
                        setTargetPos(-470, -470);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.2,0.2);
                        if (runtime.seconds() > 1){
                            mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 12:
                        strafeLeft(900);
                        break;
                }
                break;
            case 3:
                switch (counter) {
                    case 0:
                        arm(backArmPos);
                        claw(CLOSED_CLAW_POS);
                        if (runtime.seconds() > 2) {
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 1:
                        liftMotor.setTargetPosition(1200);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 2) {
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 2:
                        setTargetPos(3300, 3300);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.5, 0.5);
                        if (runtime.seconds() > 5){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 3:
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        counter++;
                        break;
                    case 4:
                        strafeRight(900, 0.4);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 5:
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        mode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                        counter++;
                        break;
                    case 6:
                        liftMotor.setTargetPosition(5500);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 4) {

                            counter++;
                        }
                        break;
                    case 7:
                        runtime.reset();
                        mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        mode(DcMotor.RunMode.RUN_USING_ENCODER);
                        counter++;

                        break;
                    case 8:
                        setTargetPos(-470, -470);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.2,0.2);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 9:
                        liftMotor.setTargetPosition(1200);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1.0);
                        if (runtime.seconds() > 2){
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 10:
                        claw(OPEN_CLAW_POS);
                        runtime.reset();
                        counter++;
                        break;
                    case 11:
                        setTargetPos(470, 470);
                        mode(DcMotor.RunMode.RUN_TO_POSITION);
                        power(0.2,0.2);
                        if (runtime.seconds() > 1){
                            mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            runtime.reset();
                            counter++;
                        }
                        break;
                    case 12:
                        strafeRight(900, 0.2);
                        break;
                }
                break;
        }

//        telemetry.addData("going to middle", middle);
//        telemetry.addData("going to left", left);
//        telemetry.addData("going to right", right);
        telemetry.addData("Right Encoder", rightFront.getCurrentPosition());
        telemetry.addData("Left Encoder", leftFront.getCurrentPosition());
        telemetry.addData("Left Power", leftBack.getPower());
        telemetry.addData("Right power", rightFront.getPower());
        telemetry.addData("runtime", runtime.toString());
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("claw pos", leftClawServo.getPosition());
        telemetry.addData("pos", pos);
        telemetry.addData("counter", counter);
//        telemetry.update();
    }

    @Override
    public void stop() {

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void setTargetPos(int left, int right) {
        leftFront.setTargetPosition(left);
        leftBack.setTargetPosition(left);
        rightBack.setTargetPosition(right);
        rightFront.setTargetPosition(right);
        telemetry.addData("Target Pos of left encoders", left);
        telemetry.addData("target Pos of right encoders", right);
    }

    private void driveStraightForInches(double inches) {
        leftFront.setTargetPosition((int) (TicksPerInch * inches));
        leftBack.setTargetPosition((int) (TicksPerInch * inches));
        rightFront.setTargetPosition((int) (TicksPerInch * inches));
        rightBack.setTargetPosition((int) (TicksPerInch * inches));
//        telemetry.addData("Target position of encoders", TicksPerInch * inches);
    }

    private void power(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }

    private void mode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void strafeLeft(int value){
        leftFront.setTargetPosition(-value);
        leftBack.setTargetPosition(value);
        rightFront.setTargetPosition(value);
        rightBack.setTargetPosition(-value);
        mode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void strafeRight(int value, double power){
        leftFront.setTargetPosition(value);
        leftBack.setTargetPosition(-value);
        rightFront.setTargetPosition(-value);
        rightBack.setTargetPosition(value);
        power(power, power);
        mode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void claw(double clawPos){
        leftClawServo.setPosition(clawPos);
        rightClawServo.setPosition(clawPos);
    }

    private void arm(double armPosition){
        armServo.setPosition(armPosition);
    }
}