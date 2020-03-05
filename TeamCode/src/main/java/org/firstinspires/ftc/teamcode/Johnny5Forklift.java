package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Johnny5Forklift", group = "SCC")
public class Johnny5Forklift extends LinearOpMode
{
    private DcMotor liftMotor;
    private boolean mastLowerBreakApplied;

    private DigitalChannel upperLiftLimit;
    private DigitalChannel lowerLiftLimit;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;

    double slowDriveFactor = 1.0;

    enum LiftState
    {
        LIFT_DISABLED, LIFT_UP_ENABLED, LIFT_DOWN_ENABLED;
    }
    private LiftState liftState = LiftState.LIFT_DISABLED;

    @Override
    public void runOpMode() throws InterruptedException
    {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

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
            if (gamepad1.b) {
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
            if ((gamepad1.a || gamepad1.left_bumper) && !lowerLiftLimitState) {
                // Yes, lower the mast
                liftMotor.setPower(1.0);
            } else if ((gamepad1.y || gamepad1.right_bumper) && !upperLiftLimitState) {
                // Raise the mast
                liftMotor.setPower(-1.0);
            } else {
                liftMotor.setPower(0);
            }

            telemetry.update();

            idle();
        }
    }
}