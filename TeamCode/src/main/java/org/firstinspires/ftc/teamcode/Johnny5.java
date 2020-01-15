package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Johnny5", group = "SCC")
public class Johnny5 extends LinearOpMode
{
    private DcMotor liftMotor;
    private boolean mastLowerBreakApplied;

    private Servo gripServo;

    private DigitalChannel upperLiftLimit;
    private DigitalChannel lowerLiftLimit;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;

    long lastGripTime = 0;
    long currentTime = 0;
    long gripDebounceTimeMS = 600;
    double slowDriveFactor = 1.0;

    private TouchSensor tapeHomeSensor;
    private Servo tapePositionServo;
    private DcMotor tapeExtendMotor;
    long lastTapeRotateTime = 0;
    long tapeRotateDebounceTimeMS = 200;

    enum LiftState
    {
        LIFT_DISABLED, LIFT_UP_ENABLED, LIFT_DOWN_ENABLED;
    }
    private LiftState liftState = LiftState.LIFT_DISABLED;

    @Override
    public void runOpMode() throws InterruptedException
    {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        gripServo = hardwareMap.servo.get("gripServo");

        upperLiftLimit = hardwareMap.get(DigitalChannel.class, "upperLiftLimit");
        upperLiftLimit.setMode(DigitalChannel.Mode.INPUT);
        lowerLiftLimit = hardwareMap.get(DigitalChannel.class, "lowerLiftLimit");
        lowerLiftLimit.setMode(DigitalChannel.Mode.INPUT);

        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        mastLowerBreakApplied = true;

        lastGripTime = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();

        tapeHomeSensor = hardwareMap.touchSensor.get("tapeHomeSensor");
        tapePositionServo = hardwareMap.servo.get("tapePositionServo");
        tapeExtendMotor = hardwareMap.dcMotor.get("tapeExtendMotor");
        lastTapeRotateTime = System.currentTimeMillis();

        // Zero the robot
        tapePositionServo.setPosition(0.0);
        tapeRetractToHome();

        waitForStart();

        while (opModeIsActive()) {
            boolean upperLiftLimitState = !upperLiftLimit.getState();
            boolean lowerLiftLimitState = !lowerLiftLimit.getState();
            telemetry.addData("upperLiftLimitState", upperLiftLimitState);
            telemetry.addData("lowerLiftLimitState", lowerLiftLimitState);

            // Has the mast's lower break been applied?
            if (lowerLiftLimitState && !mastLowerBreakApplied) {
                // Raise the mast
                liftMotor.setPower(1);
                sleep(20);
                liftMotor.setPower(0);
                mastLowerBreakApplied = true;
            } else if (!lowerLiftLimitState) {
                // Reset the lower break flag
                mastLowerBreakApplied = false;
            }

            double motorFR = 0;
            double motorFL = 0;
            double motorBR = 0;
            double motorBL = 0;
            final float threshold = 0.2f;
            if (Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold) {
                motorFR = (gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
                motorFL = (-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
                motorBR = (-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
                motorBL = (gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
            }
            if (Math.abs(gamepad1.left_stick_x) > threshold) {
                motorFR *= -1;
                motorFL *= -1;
                motorBR *= -1;
                motorBL *= -1;
            }
            if (Math.abs(gamepad1.right_stick_x) > threshold) {
                //rotate
                motorFR = (gamepad1.right_stick_x) / 2;
                motorFL = (gamepad1.right_stick_x) / 2;
                motorBR = (-gamepad1.right_stick_x) / 2;
                motorBL = (-gamepad1.right_stick_x) / 2;
            }
            if (gamepad1.a) {
                slowDriveFactor = 0.3;
            } else if (gamepad1.x) {
                slowDriveFactor = 0.6;
            } else {
                slowDriveFactor = 1.0;
            }
            leftFrontMotor.setPower(motorFL * slowDriveFactor);
            rightFrontMotor.setPower(motorFR * slowDriveFactor);
            leftRearMotor.setPower(motorBL * slowDriveFactor);
            rightRearMotor.setPower(motorBR * slowDriveFactor);

            // Is the user requesting that the mast is lowered and we are not at the limit?
            if (-gamepad2.left_stick_y * threshold < 0 && !lowerLiftLimitState) {
                // Yes, lower the mast
                liftMotor.setPower(-gamepad2.left_stick_y);
            } else if (-gamepad2.left_stick_y * threshold > 0 && !upperLiftLimitState) {
                // Raise the mast
                liftMotor.setPower(-gamepad2.left_stick_y);
            } else {
                liftMotor.setPower(0);
            }

            // Check to see if we need to move the grip servo
            currentTime = System.currentTimeMillis();
            if (lastGripTime + gripDebounceTimeMS <= currentTime)
            {
                if (gamepad2.a && gripServo.getPosition() == 0.0) {
                    // move to 0 degrees.
                    gripServo.setPosition(0.5);
                    lastGripTime = System.currentTimeMillis();
                } else if (gamepad2.a && gripServo.getPosition() == 0.5) {
                    // move to 90 degrees.
                    gripServo.setPosition(0);
                    lastGripTime = System.currentTimeMillis();
                }
            }
            telemetry.addData("Servo Position", gripServo.getPosition());
            telemetry.addData("Lift Gamepad Input", gamepad2.left_stick_y);

            // Extend tape?
            if (gamepad2.y) {
                tapeExtendMotor.setPower(-1);
            } else if (gamepad2.b && !tapeHomeSensor.isPressed()) {
                tapeExtendMotor.setPower(1);
            } else {
                tapeExtendMotor.setPower(0);
            }

            // Check to see if we need to move the tape servo
            currentTime = System.currentTimeMillis();
            if (lastTapeRotateTime + tapeRotateDebounceTimeMS <= currentTime)
            {
                // Is the user requesting that tape be rotated?
                if (-gamepad2.right_stick_x * threshold < 0) {
                    // Yes, rotate the tape measure
                    tapePositionServo.setPosition(tapePositionServo.getPosition() + 0.1);
                } else if (-gamepad2.right_stick_x * threshold > 0) {
                    // Rotate the tape measure
                    tapePositionServo.setPosition(tapePositionServo.getPosition() - 0.1);
                }
                lastTapeRotateTime = System.currentTimeMillis();
            }

            telemetry.addData("currentTime MS", currentTime);

            //telemetry.addData("Status", "Version 0.1");
            telemetry.update();

            idle();
        }
    }

    private void tapeRetractToHome() {
        ElapsedTime retractTime = new ElapsedTime();
        retractTime.reset();
        // Retract tape measure to home
        tapeExtendMotor.setPower(1);
        while (!isStopRequested() && !tapeHomeSensor.isPressed() && retractTime.milliseconds() < 3000) {
            telemetry.addData("tapeHomeSensor.isPressed()", tapeHomeSensor.isPressed());
            telemetry.update();
            idle();
        }
        tapeExtendMotor.setPower(-0.3);
        sleep(5);
        tapeExtendMotor.setPower(0);
    }
}