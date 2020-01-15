package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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

@Autonomous(name="ParkBlueJohnny5", group="SCC")
public class ParkBlueJohnny5 extends LinearOpMode
{

    //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  NOTE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // This program is writen from the side of the blue team
    // So the program that is run is run must consider what side it's on
    // Always edit this program and name it BlueAutoJohnny5, then copy it for the RED side and flip
    // this BOOL to false
    protected boolean BLUE_TEAM = true;
    //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  END NOTE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;

    private DcMotor liftMotor;
    private DigitalChannel lowerLiftLimit;

    private Servo gripServo;

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
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        lowerLiftLimit = hardwareMap.get(DigitalChannel.class, "lowerLiftLimit");
        lowerLiftLimit.setMode(DigitalChannel.Mode.INPUT);

        gripServo = hardwareMap.servo.get("gripServo");

        //colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        //colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run Autonomous");    //
        telemetry.update();

        // Zero the robot
        lowerMast();

        // Close the grip servo
        gripServo.setPosition(0.5);
        sleep(100);

        // Reset the runtime timer
        runtime.reset();
        waitForStart();

        // Wait for other robot to finish
        sleep(20000);

        // Strafe left to move away from stone
        strafeLeft();
        sleep(2000);
        driveStop();

        // Drive backwards
        driveBackward();
        sleep(400);
        driveStop();
    }

    private double teamDriveFactor() {
        double teamFactor = 1.0;
        if (!BLUE_TEAM)
            teamFactor = -1.0;
        return teamFactor;
    }
    private void driveFoward() {
        leftFrontMotor.setPower(.4);
        rightFrontMotor.setPower(-.4);
        leftRearMotor.setPower(-.4);
        rightRearMotor.setPower(.4);
    }

    private void driveBackward() {
        leftFrontMotor.setPower(-.3);
        rightFrontMotor.setPower(.3);
        leftRearMotor.setPower(.3);
        rightRearMotor.setPower(-.3);
    }

    private void driveLeft() {
        leftFrontMotor.setPower(-.5 * teamDriveFactor());
        rightFrontMotor.setPower(-.5 * teamDriveFactor());
        leftRearMotor.setPower(.5 * teamDriveFactor());
        rightRearMotor.setPower(.5 * teamDriveFactor());
    }

    private void driveRight() {
        leftFrontMotor.setPower(.5 * teamDriveFactor());
        rightFrontMotor.setPower(.5 * teamDriveFactor());
        leftRearMotor.setPower(-.5 * teamDriveFactor());
        rightRearMotor.setPower(-.5 * teamDriveFactor());
    }

    private void driveStop() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    private void strafeLeft() {
        leftFrontMotor.setPower(-.3 * teamDriveFactor());
        rightFrontMotor.setPower(-.3 * teamDriveFactor());
        leftRearMotor.setPower(-.3 * teamDriveFactor());
        rightRearMotor.setPower(-.3 * teamDriveFactor());
    }

    private void strafeRight() {
        leftFrontMotor.setPower(.3 * teamDriveFactor());
        rightFrontMotor.setPower(.3 * teamDriveFactor());
        leftRearMotor.setPower(.3 * teamDriveFactor());
        rightRearMotor.setPower(.3 * teamDriveFactor());
    }

    private void lowerMast() {
        // While the lower lift limit is not met
        while (lowerLiftLimit.getState()) {
            // Lower the mast
            idle();
            liftMotor.setPower(-1);
        }
        // Apply the motor break
        liftMotor.setPower(1);
        sleep(20);
        liftMotor.setPower(0);
    }
}