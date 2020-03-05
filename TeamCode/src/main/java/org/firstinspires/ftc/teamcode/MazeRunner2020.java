package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="MazeRunner2020", group="SCC")
public class MazeRunner2020 extends LinearOpMode
{
    //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  END NOTE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.8;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.058;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;

    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorRight;

    static final int LEFT_BLUE_THRESHOLD = 37;
    static final int LEFT_RED_THRESHOLD = 54;
    static final int RIGHT_BLUE_THRESHOLD = 58;
    static final int RIGHT_RED_THRESHOLD = 92;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //colorSensor = hardwareMap.colorSensor.get("colorSensor");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run Autonomous");    //
        telemetry.update();

        // Reset the runtime timer
        runtime.reset();

        waitForStart();

        // Debug color sensor
        /*while (!isStopRequested()) {
            telemetry.addData("Red  ", colorSensor.red());
            sleep(1000);
            telemetry.update();
            idle();
        }*/

        // Drive halfway to the stones
        driveFoward();
        sleep(500);
        driveStop();
        driveLeft();
        sleep(500);
        driveStop();
        driveBackward();
        sleep(500);
        driveStop();
        driveRight();
        sleep(500);
        driveStop();

        // Drive backwards
        //driveBackwardsInches(13, 0.8);
    }

    private void driveFoward() {
        leftDriveMotor.setPower(.2);
        rightDriveMotor.setPower(.2);
    }

    private void driveBackward() {
        leftDriveMotor.setPower(-.2);
        rightDriveMotor.setPower(-.2);
    }

    private void driveLeft() {
        leftDriveMotor.setPower(-.2);
        rightDriveMotor.setPower(.2);
    }

    private void driveRight() {
        leftDriveMotor.setPower(.2);
        rightDriveMotor.setPower(-.2);
    }

    private void driveStop() {
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        sleep(500);
    }
}
