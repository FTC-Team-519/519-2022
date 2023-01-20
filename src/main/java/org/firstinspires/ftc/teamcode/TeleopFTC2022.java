/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counterclockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleopFTC2022", group = "Linear Opmode")
//@Disabled
public class TeleopFTC2022 extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo armServo = null;
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;
    BNO055IMU imu;
    Orientation angles;

    private double armPos = 0.5;
    private double middleArmPos = 0.472;
    private double backArmPos = 0.0;
    private double frontArmPos = .950;

    private boolean rightBumpPreviouslyPressed = false;
    private boolean slow = false;
    private double denom = 1.11;

    private double clawPos = 0.5;
    private static final double OPEN_CLAW_POS = 0.74;
    private static final double CLOSED_CLAW_POS = 0.60;
    private static final int LOW_GOAL = 3514;
    private static final int MID_GOAL = 5300;
    private static final int HIGH_GOAL = 6900;
    private static final int DEFAULT_PRESET_VALUE = 10;
    private static final double STALL_HIGH = 0.0075;
    private static final double STALL_MID = 0.005;
    private static final double STALL_LOW = 0.0;
    private boolean presetMode;
    private int targetEncoderValue;
    private double stallPos;


    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_FrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_BackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_FrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_BackDrive");
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "right_BackDrive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "right_FrontDrive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "left_BackDrive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "left_FrontDrive");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightClawServo.setDirection(Servo.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.REVERSE);


        presetMode = false;
        targetEncoderValue = DEFAULT_PRESET_VALUE;
        stallPos = STALL_LOW;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        clawPos = OPEN_CLAW_POS;
        armPos = backArmPos;
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // run until the end of the match (driver presses STOP)
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        axial = normalize(axial);
        lateral = normalize(lateral);
        yaw = normalize(yaw);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        double liftMotorPower = 0;

        boolean rightBumpPressed = gamepad1.right_bumper;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // slow mode on/off
        if(rightBumpPressed && !rightBumpPreviouslyPressed){
            slow = !slow;
        }
        if(slow){
            denom = 4;
        }else {
            denom = 1;
        }
        rightBumpPreviouslyPressed = gamepad1.right_bumper;



            // claw open/close
        if (gamepad2.circle) {
            clawPos = OPEN_CLAW_POS;
            clawPos = Math.min(clawPos, 1.0);
        }
        if (gamepad2.square) {
            clawPos = CLOSED_CLAW_POS;
            clawPos = Math.max(clawPos, 0.0);
        }

        //arm Position presets
//        if (gamepad2.dpad_right || gamepad2.dpad_left) {
//            armPos = middleArmPos;
//        }
        if (gamepad2.dpad_up) {
            armPos = frontArmPos;
            armPos = Math.min(armPos, 1.0);
        }
        if (gamepad2.dpad_down) {
            armPos = backArmPos;
            armPos = Math.max(armPos, 0.0);
        }


        // lift up/down
        // up
        if (gamepad2.right_bumper) {
            presetMode = false;
            liftMotorPower = 1.0;
        } else if (gamepad2.left_bumper) { // down
            presetMode = false;
            if (liftMotor.getCurrentPosition() <= 0 ){
                liftMotorPower=0.0;
            }else{
                liftMotorPower=-1.0;
            }
        }else if (!presetMode){ //move without protections
            liftMotorPower = -gamepad2.right_stick_y;
        }

        // lift encoder reset
        if (gamepad2.y){
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




        // lift motor presets
        if (gamepad2.a) {
            presetMode = true;
            targetEncoderValue = LOW_GOAL;
            stallPos = STALL_LOW;
        }

        if (gamepad2.left_trigger >= 0.75) {
            presetMode = true;
            targetEncoderValue = MID_GOAL;
            stallPos = STALL_MID;
        }

        if (gamepad2.right_trigger >= 0.75) {
            presetMode = true;
            targetEncoderValue = HIGH_GOAL;
            stallPos = STALL_HIGH;
        }

        if (presetMode) {
            int liftEncoderPos = liftMotor.getCurrentPosition();
            if (liftEncoderPos < targetEncoderValue) {
                if (targetEncoderValue - liftEncoderPos < 50){
                    liftMotorPower = 0.1;
                } else if (targetEncoderValue - liftEncoderPos < 200){
                    liftMotorPower = 0.25;
                } else {
                    liftMotorPower = 1.0;
                }
            } else if (liftEncoderPos > (targetEncoderValue + 20)) {
                if (liftEncoderPos - targetEncoderValue < 50){
                    liftMotorPower = -0.1;
                } else if (liftEncoderPos - targetEncoderValue < 200){
                    liftMotorPower = -0.2;
                } else {
                    liftMotorPower = -1.0;
                }
            } else {
                liftMotorPower = stallPos;
            }
        }

        leftClawServo.setPosition(clawPos);
        rightClawServo.setPosition(clawPos);
        armServo.setPosition(armPos);

        liftMotor.setPower(liftMotorPower);
        leftFrontDrive.setPower(leftFrontPower / denom);
        rightFrontDrive.setPower(rightFrontPower / denom);
        leftBackDrive.setPower(leftBackPower / denom);
        rightBackDrive.setPower(rightBackPower / denom);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "liftMotor: " + liftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Encoder", rightBackDrive.getCurrentPosition());
        telemetry.addData("Left Motor Encoder", leftBackDrive.getCurrentPosition());
        telemetry.addData("leftClawServo", "Position" + leftClawServo.getPosition());
        telemetry.addData("rightClawServo", "Position" + rightClawServo.getPosition());
        //telemetry.addData("Arm Servo Position", armServo.getPosition());
        //telemetry.addData("Right Trigger", "Servo" + gamepad1.right_trigger);
        telemetry.addData("claw pos value", clawPos);
        telemetry.addData("Slow Mode", slow);
        }
    public double normalize(double startValue){
        if(startValue > 0){
            startValue = Math.pow(startValue,2);
        }
        else{
            startValue = -(Math.pow(startValue,2));
        }
        return startValue;
    }
}
