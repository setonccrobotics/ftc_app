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

public class AutoJohnny5 extends LinearOpMode
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

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;

    private DcMotor liftMotor;
    private DigitalChannel lowerLiftLimit;

    private TouchSensor tapeHomeSensor;
    private Servo tapePositionServo;
    private DcMotor tapeExtendMotor;

    private Servo gripServo;

    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorRight;

    static final int LEFT_BLUE_THRESHOLD = 37;
    static final int LEFT_RED_THRESHOLD = 54;
    static final int RIGHT_BLUE_THRESHOLD = 58;
    static final int RIGHT_RED_THRESHOLD = 92;

    private int skystonePosition = 0;
    private boolean skystoneFound = false;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

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
    private static final String VUFORIA_KEY = "AWgjQOP/////AAABmfWwWGnZwkNwpAbSyPYv8nEUs+ELPJb8Nf3+FG7o3qm1K4Pql+XRpUqChCO5qO0HUyvi6ogLN9KG9trT5aCeE0aMao7BAOdwNk5zDG5AsxlQeKIcWKvf+ojH1MKfKoOpbmqtKwUuS1roW/S0A4+5pOiQERqoz6IEVLO7E1kV2dfCsmYKf6M8mGHhexp6p1a1dyw0FZyhKNxGYi6BcfODHumB8++rFNWsQwBEjZFjHSgKC4qsn8pchxUa4s2qhdnxCiNMZYGEZIxNynCCpXgSLBFcqrQV0SBj6GSuhpW9AUj8rKZQ/bw4FxavVH/23VwuRh0ax6NakaCM0hHielO2q//xtfqOa10wmucJMQssWJLj";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private float skystoneYpos    = 0;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        lowerLiftLimit = hardwareMap.get(DigitalChannel.class, "lowerLiftLimit");
        lowerLiftLimit.setMode(DigitalChannel.Mode.INPUT);

        gripServo = hardwareMap.servo.get("gripServo");

        //colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        //colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

        tapeHomeSensor = hardwareMap.touchSensor.get("tapeHomeSensor");
        tapePositionServo = hardwareMap.servo.get("tapePositionServo");
        tapeExtendMotor = hardwareMap.dcMotor.get("tapeExtendMotor");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run Autonomous");    //
        telemetry.update();

        // Zero the robot
        lowerMast();

        // Close the grip servo
        gripServo.setPosition(0.5);
        sleep(100);

        // Tape home sensor debug
        /*while (!isStopRequested()) {
            telemetry.addData("tapeHomeSensor.isPressed()", tapeHomeSensor.isPressed());
            telemetry.update();
            idle();
        }*/

        // Home the tape sensor and servo
        tapePositionServo.setPosition(0.15);
        tapeRetractToHome();

        // Reset the runtime timer
        runtime.reset();

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // Perimeter SkyStones
        /*VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");*/

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        // Vuforia username: setonccrobotics, password: Cyb3r2019

        // Debug color sensors
        /*while (!isStopRequested()) {
            telemetry.addData("Left Red  ", colorSensorLeft.red());
            telemetry.addData("Left Blue  ", colorSensorLeft.blue());
            telemetry.addData("Right Red  ", colorSensorRight.red());
            telemetry.addData("Right Blue  ", colorSensorRight.blue());
            sleep(1000);
            telemetry.update();
            idle();
        }*/

        // Open the grip
        gripServo.setPosition(0);

        // Drive halfway to the stones
        driveForwardInches(27, 0.8);

        targetsSkyStone.activate();
        while (!isStopRequested()) {
            skystonePosition++;
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                //Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
                skystoneYpos = translation.get(1) / mmPerInch;
                //continue; // Uncomment to debug vision parameters
                break; // Comment out to debug, uncomment to run production
            }
            else {
                telemetry.addData("Visible Target", "none");
                strafeRightInches(6, 0.5);
                sleep(100);
            }

            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();

        // Adjust the offset for the phone position
        telemetry.addData("BeforeskystoneYpos", "%.1f", skystoneYpos);
        skystoneYpos += 2.6;
        telemetry.addData("AfterskystoneYpos", "%.1f", skystoneYpos);

        // Get in front of the stone
        if (getTeam()) {
            if (skystoneYpos > 0.0) {
                // Strafe right
                telemetry.addData("StrafeRight", "%.1f", skystoneYpos);
                strafeRightInches(skystoneYpos + 2, 0.6);
            } else {
                // Strafe left
                skystoneYpos *= -1.0;
                telemetry.addData("StrafeLeft", "%.1f", skystoneYpos);
                strafeLeftInches(skystoneYpos + 2, 0.6);
            }
        } else {
            // RED TEAM
            if (skystoneYpos > 0.0) {
                // Strafe left
                telemetry.addData("StrafeLeft", "%.1f", skystoneYpos);
                strafeLeftInches(skystoneYpos + 2, 0.6);
            } else {
                // Strafe right
                skystoneYpos *= -1.0;
                telemetry.addData("StrafeRight", "%.1f", skystoneYpos);
                strafeRightInches(skystoneYpos + 2, 0.6);
            }
        }
        telemetry.update();

        // Drive up to the stones
        if (getTeam())
            driveForwardInches(16, 0.6);
        else
            driveForwardInches(14, 0.6);

        // Close the grip servo
        gripServo.setPosition(0.5);
        sleep(800);

        // Drive backwards
        driveBackwardsInches(13, 0.8);

        // Turn 90 degrees to the left or right depending on if red or blue team
        if (getTeam())
            driveLeftInches(19, 0.6);
        else
            driveLeftInches(18, 0.6);

        // Drive forward until on the other side of board
        driveFoward();
        if (getTeam()) {
            for (int i = 0; colorSensorLeft.blue() < 31 || colorSensorRight.blue() < 49; i++) {
                sleep(10);
                if (i > 600)
                    break;
            }
        } else {
            for (int i = 0; colorSensorLeft.red() < 36 || colorSensorRight.red() < 57; i++) {
                sleep(10);
                if (i > 600)
                    break;
            }
        }
        driveStop();

        // Drive from under the bridge to the platform
        driveForwardInches(48, 1);

        // Turn 90 degrees towards platform
        driveRightInches(18, 0.6);

        // Raise the mast/skystone up 4 inches
        liftMotor.setPower(1);
        sleep(720);
        liftMotor.setPower(0);

        // Move forward to edge of platform
        driveFoward();
        for (int i = 0; colorSensorLeft.blue() < LEFT_BLUE_THRESHOLD && colorSensorRight.blue() < RIGHT_BLUE_THRESHOLD; i++) {
            sleep(200);
            if (i > 15)
                break;
        }
        driveStop();

        // Place skystone on platform
        gripServo.setPosition(0); // Open the grip servo
        sleep(300);

        // Strafe left to move away from stone
        strafeLeftInches(10, 0.6);

        // Move forward to edge of platform
        driveForwardInches(3, 0.4);

        // Lower mast until platform is gripped
        lowerMast();
        sleep(100);

        // Extend the tape
        tapePositionServo.setPosition(0.15);
        //tapeExtendMotor.setPower(-1);

        // Drive backwards with platform
        driveBackwardsInches(50, 1);

        // Move forward to put some space between the robot and platform in the corner
        driveForwardInches(4, 0.8);

        // Raise the mast/skystone up
        liftMotor.setPower(1);
        sleep(400);
        liftMotor.setPower(0);

        // Drive backwards away from the platform
        driveBackwardsInches(6, 1);
        //tapeExtendMotor.setPower(0);
    }

    public boolean getTeam()
    {
        return false;
    }

    private double teamDriveFactor() {
        double teamFactor = 1.0;
        if (!getTeam())
            teamFactor = -1.0;
        return teamFactor;
    }

    private void driveFoward() {
        leftFrontMotor.setPower(.8);
        rightFrontMotor.setPower(.8);
        leftRearMotor.setPower(.8);
        rightRearMotor.setPower(.8);
    }

    private void driveBackward() {
        leftFrontMotor.setPower(-.3);
        rightFrontMotor.setPower(-.3);
        leftRearMotor.setPower(-.3);
        rightRearMotor.setPower(-.3);
    }

    private void driveLeft() {
        leftFrontMotor.setPower(-.5 * teamDriveFactor());
        rightFrontMotor.setPower(.5 * teamDriveFactor());
        leftRearMotor.setPower(-.5 * teamDriveFactor());
        rightRearMotor.setPower(.5 * teamDriveFactor());
    }

    private void driveRight() {
        leftFrontMotor.setPower(.5 * teamDriveFactor());
        rightFrontMotor.setPower(-.5 * teamDriveFactor());
        leftRearMotor.setPower(.5 * teamDriveFactor());
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
        rightFrontMotor.setPower(.3 * teamDriveFactor());
        leftRearMotor.setPower(.3 * teamDriveFactor());
        rightRearMotor.setPower(-.3 * teamDriveFactor());
    }

    private void strafeRight() {
        rightFrontMotor.setPower(.3 * teamDriveFactor());
        leftFrontMotor.setPower(-.3 * teamDriveFactor());
        rightRearMotor.setPower(-.35 * teamDriveFactor());
        leftRearMotor.setPower(.3 * teamDriveFactor());
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

    private void driveForwardInches(double inches, double speed) {
        driveInInches(inches, speed, 1, 1, 1, 1);
    }

    private void driveBackwardsInches(double inches, double speed) {
        driveInInches(inches, speed, -1, -1, -1, -1);
    }

    private void strafeLeftInches(double inches, double speed) {
        double f = teamDriveFactor();
        driveInInches(inches, speed, -1*f, 1*f, 1*f, -1*f);
    }

    private void strafeRightInches(double inches, double speed) {
        double f = teamDriveFactor();
        driveInInches(inches, speed, 1*f, -1*f, -1*f, 1*f);
    }

    private void driveLeftInches(double inches, double speed) {
        double f = teamDriveFactor();
        driveInInches(inches, speed, -1*f, 1*f, -1*f, 1*f);
    }

    private void driveRightInches(double inches, double speed) {
        double f = teamDriveFactor();
        driveInInches(inches, speed, 1*f, -1*f, 1*f, -1*f);
    }

    private void driveInInches(double inches, double speed, double leftFrontFactor, double rightFrontFactor, double leftRearFactor, double rightRearFactor) {
        int newLeftFrontMotorTarget;
        int newRightFrontMotorTarget;
        int newLeftRearMotorTarget;
        int newRightRearMotorTarget;

        // Determine new target position, and pass to motor controller
        newLeftFrontMotorTarget = (int)((leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)) * leftFrontFactor);
        newRightFrontMotorTarget = (int)((rightFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)) * rightFrontFactor);
        newLeftRearMotorTarget = (int)((leftRearMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)) * leftRearFactor);
        newRightRearMotorTarget = (int)((rightRearMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)) * rightRearFactor);
        leftFrontMotor.setTargetPosition(newLeftFrontMotorTarget);
        rightFrontMotor.setTargetPosition(newRightFrontMotorTarget);
        leftRearMotor.setTargetPosition(newLeftRearMotorTarget);
        rightRearMotor.setTargetPosition(newRightRearMotorTarget);

        // Turn On RUN_TO_POSITION
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion.
        //runtime.reset();
        rightRearMotor.setPower(speed);// * rightRearFactor);
        leftFrontMotor.setPower(speed);// * leftFrontFactor);
        rightFrontMotor.setPower(speed);// * rightFrontFactor);
        leftRearMotor.setPower(speed);// * leftRearFactor);

        while (opModeIsActive() &&
                (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                        leftRearMotor.isBusy() && rightRearMotor.isBusy())) {
            /*// Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newLeftFrontMotorTarget, newRightFrontMotorTarget,
                    newLeftRearMotorTarget, newRightRearMotorTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                    leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(),
                    leftRearMotor.getCurrentPosition(),
                    rightRearMotor.getCurrentPosition());
            telemetry.update();*/
        }

        // Stop all motion;
        rightRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
