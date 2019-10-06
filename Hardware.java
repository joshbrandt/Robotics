package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class Hardware {

    HardwareMap hwMap;

    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeRight, intakeLeft;

    BNO055IMU imu;

    Orientation angles;

    AnalogInput potentiometer;


    enum turnDirection {clockWise, counterClockWise, notSet}
    enum Direction {left, right}
    enum IntakeDirection {in, out, off}
    enum color {blue, red, notSet}

    final int marginOfError = 15;
    final double slow = 0.2;

    public static final String VUFORIA_KEY = "AYeAfdj/////AAABmUaE10+gXEPxkuBBWS87DYN48PlcZ/n1JEi/LJMcZWkivMh/FFzMe+2bw7ch30+wbiSwL441e037a160QaE/D2P0LQxsjrXDNLGyyB2F3nXDu6AYeIFvPNOsrHYMytw8IQ2GidWhADX694rbAOTDzfYEAvQ6zVQuNVzH6KjziWeE/pTzMKMnPNY792U7w4aps1LCPGM+iW/w7ppupN+m42I67Lqs/EB+OWfpGTs+QML8mqXcM0798LoJj38hdopOPB4lUkbORofEwBAHxqrgDPJAy6pkvm1Bs8VmqKKJClXTM/s/UuCAaM7LcxVmHZvP5ONPYe9HU5dsAN3KOR2uDVa500mNUbOwD8+SKf08rxEj";

    static final float mmPerInch = 25.4f;
    static final float stoneZ = 2.00f * mmPerInch;

    static OpenGLMatrix lastLocation;
    VuforiaLocalizer vuforia;

    static boolean targetVisible;

    static List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    static float[] robotDistanceToStone = new float[2]; //x,y

    DistanceSensor rangeSensorFront;

    ModernRoboticsI2cRangeSensor rangeSensorSide;

    ColorSensor colorSensor;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //Map Motors
        frontRightMotor = hwMap.get(DcMotor.class, "fr");
        frontLeftMotor = hwMap.get(DcMotor.class, "fl");
        backRightMotor = hwMap.get(DcMotor.class, "br");
        backLeftMotor = hwMap.get(DcMotor.class, "bl");
        intakeRight = hwMap.get(DcMotor.class, "ir");
        intakeLeft = hwMap.get(DcMotor.class, "il");

        //Map Potentiometer
        potentiometer = hwMap.analogInput.get("pot");

        //Reverse Direction
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        //Zero Power Behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU Init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //IMU Map
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Front Range Sensor Map
        rangeSensorFront = hwMap.get(DistanceSensor.class, "rsF");

        //Side Range Sensor Mao
        rangeSensorSide = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsS");


    }

    public int getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int heading = Math.round(angles.firstAngle);
        if (heading < 0) heading += 360;
        return heading;
    }

    /**
     * Sets the power of the intake wheels based on the direction and power inputted.
     * @param direction The direction the intake will be going. Either in, out, or off.
     * @param motorPower The motor power of the intake motors. Must be between 0 and 1.
     */
    public void intake(IntakeDirection direction, double motorPower) {

        //Range Clip
        if (motorPower > 1) {
            motorPower = 1;
        }
        else if (motorPower < 0) {
            motorPower = 0;
        }

        //Set Motor Powers based on IntakeDirection
        switch (direction) {
            case in:
                intakeRight.setPower(motorPower);
                intakeLeft.setPower(motorPower);
                break;

            case out:
                intakeRight.setPower(-motorPower);
                intakeLeft.setPower(-motorPower);
                break;

            case off:
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
                break;
        }
    }

    public void intake(IntakeDirection direction) {
        intake(direction, 1);
    }



    public int getPitch() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Math.round(angles.thirdAngle);

    }


    public void rotate(int angle, double speed, boolean opModeIsActive) {

        int differenceInPosition = getHeading() - angle;

        turnDirection rotateDirection = turnDirection.notSet;

        if ((differenceInPosition > 0 && differenceInPosition < 180) || (differenceInPosition < 0 && Math.abs(differenceInPosition) > 180))
            rotateDirection = turnDirection.clockWise;
        if ((differenceInPosition > 0 && differenceInPosition > 180) || (differenceInPosition < 0 && Math.abs(differenceInPosition) < 180))
            rotateDirection = turnDirection.counterClockWise;

        boolean firstTurnDone = false;

        while (!firstTurnDone && opModeIsActive) {


            if ((getHeading() > angle - marginOfError) && (getHeading() < angle + marginOfError))
                firstTurnDone = true;

            switch (rotateDirection) {
                case clockWise:
                    turn(Direction.left, speed);
                    break;

                case counterClockWise:
                    turn(Direction.right, speed);
                    break;
            }
        }
        brake();

        boolean secondTurnDone = false;

        while (!secondTurnDone && opModeIsActive) {

            if (getHeading() == angle) secondTurnDone = true;

            if (getHeading() < angle && angle != 0) {
                turn(Direction.right, slow);
            } else if (angle != 0) {
                turn(Direction.left, slow);
            }

            if (angle == 0) {
                if (getHeading() < 330) turn(Direction.left, slow);
                if (getHeading() > 330) turn(Direction.right, slow);
            }
        }
        brake();

    }

    public void brake() {
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void drive(double power) {
        backLeftMotor.setPower(power);
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(power);

    }

    public void turn(Direction direction, double power) {

        if (direction == Direction.right) {
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
        }

        if (direction == Direction.left) {
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(-power);
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
        }
    }


    public void strafe(Direction direction, double power) {

        if (direction == Direction.left) {
            frontRightMotor.setPower(power);
            backRightMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(power);

        }

        if (direction == Direction.right) {
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(-power);


        }

    }

    public void driveToPos(int position, double power, boolean opModeIsActive) {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean frontLeftMotorInPosition = false;
        boolean frontRightMotorInPosition = false;
        boolean backLeftMotorInPosition = false;
        boolean backRightMotorInPosition = false;

        while ((!frontLeftMotorInPosition || !frontRightMotorInPosition || !backLeftMotorInPosition || !backRightMotorInPosition) && opModeIsActive) {

            if (Math.abs(frontLeftMotor.getCurrentPosition()) < position)
                frontLeftMotor.setPower(power);
            if (Math.abs(frontLeftMotor.getCurrentPosition()) > position) {
                frontLeftMotor.setPower(0);
                frontLeftMotorInPosition = true;
            }

            if (Math.abs(frontRightMotor.getCurrentPosition()) < position)
                frontRightMotor.setPower(power);
            if (Math.abs(frontRightMotor.getCurrentPosition()) > position) {
                frontRightMotor.setPower(0);
                frontRightMotorInPosition = true;
            }

            if (Math.abs(backLeftMotor.getCurrentPosition()) < position)
                backLeftMotor.setPower(power);
            if (Math.abs(backLeftMotor.getCurrentPosition()) > position) {
                backLeftMotor.setPower(0);
                backLeftMotorInPosition = true;
            }

            if (Math.abs(backRightMotor.getCurrentPosition()) < position)
                backRightMotor.setPower(power);
            if (Math.abs(backRightMotor.getCurrentPosition()) > position) {
                backRightMotor.setPower(0);
                backRightMotorInPosition = true;
            }

        }


    }


    double getPotAngle(double x, double in_min, double in_max, double out_min, double out_max) {

        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    }

    public void strafeToPos(Direction direction, int position, double power) {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean frontLeftMotorInPosition = false;
        boolean frontRightMotorInPosition = false;
        boolean backLeftMotorInPosition = false;
        boolean backRightMotorInPosition = false;

        if (direction == Direction.right) {

            while (!frontLeftMotorInPosition || !frontRightMotorInPosition || !backLeftMotorInPosition || !backRightMotorInPosition) {

                if (Math.abs(frontLeftMotor.getCurrentPosition()) < position)
                    frontLeftMotor.setPower(power);
                if (Math.abs(frontLeftMotor.getCurrentPosition()) > position)
                    frontLeftMotor.setPower(-power);
                if (Math.abs(frontLeftMotor.getCurrentPosition()) == position) {
                    frontLeftMotor.setPower(0);
                    frontLeftMotorInPosition = true;
                }

                if (Math.abs(frontRightMotor.getCurrentPosition()) < position)
                    frontRightMotor.setPower(-power);
                if (Math.abs(frontRightMotor.getCurrentPosition()) > position)
                    frontRightMotor.setPower(power);
                if (Math.abs(frontRightMotor.getCurrentPosition()) == position) {
                    frontRightMotor.setPower(0);
                    frontRightMotorInPosition = true;
                }

                if (Math.abs(backLeftMotor.getCurrentPosition()) < position)
                    backLeftMotor.setPower(-power);
                if (Math.abs(backLeftMotor.getCurrentPosition()) > position)
                    backLeftMotor.setPower(power);
                if (Math.abs(backLeftMotor.getCurrentPosition()) == position) {
                    backLeftMotor.setPower(0);
                    backLeftMotorInPosition = true;
                }

                if (Math.abs(backRightMotor.getCurrentPosition()) < position)
                    backRightMotor.setPower(power);
                if (Math.abs(backRightMotor.getCurrentPosition()) > position)
                    backRightMotor.setPower(-power);
                if (Math.abs(backRightMotor.getCurrentPosition()) == position) {
                    backRightMotor.setPower(0);
                    backRightMotorInPosition = true;
                }
            }
        } else if (direction == Direction.left) {

            while (!frontLeftMotorInPosition || !frontRightMotorInPosition || !backLeftMotorInPosition || !backRightMotorInPosition) {

                if (Math.abs(frontLeftMotor.getCurrentPosition()) < position)
                    frontLeftMotor.setPower(-power);
                if (Math.abs(frontLeftMotor.getCurrentPosition()) > position) {
                    frontLeftMotor.setPower(0);
                    frontLeftMotorInPosition = true;
                }

                if (Math.abs(frontRightMotor.getCurrentPosition()) < position)
                    frontRightMotor.setPower(power);
                if (Math.abs(frontRightMotor.getCurrentPosition()) > position) {
                    frontRightMotor.setPower(0);
                    frontRightMotorInPosition = true;
                }

                if (Math.abs(backLeftMotor.getCurrentPosition()) < position)
                    backLeftMotor.setPower(power);
                if (Math.abs(backLeftMotor.getCurrentPosition()) > position) {
                    backLeftMotor.setPower(0);
                    backLeftMotorInPosition = true;
                }

                if (Math.abs(backRightMotor.getCurrentPosition()) < position)
                    backRightMotor.setPower(-power);
                if (Math.abs(backRightMotor.getCurrentPosition()) > position) {
                    backRightMotor.setPower(0);
                    backRightMotorInPosition = true;
                }
            }

        }

    }


    public void sleepRobot(long time) {
        try {Thread.sleep(time);} catch (InterruptedException e) {}
    }


    public void initVuforia() {

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;

        try {
            vuforiaParameters.cameraName = hwMap.get(WebcamName.class, "Webcam");
        } catch (Exception e) {}
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuforiaParameters.cameraDirection);
        }

        targetsSkyStone.activate();
    }

    public void lookForStone() {

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        if (targetVisible) {

            VectorF translation = lastLocation.getTranslation();

            robotDistanceToStone[1] = -translation.get(0) / mmPerInch;
            robotDistanceToStone[0] = -translation.get(1) / mmPerInch;


        }

    }

    public void driveToStone() {

        int angleToTurn = (int) Math.round(Math.toDegrees(Math.atan(robotDistanceToStone[0]/robotDistanceToStone[1])));

        if (angleToTurn < 0) angleToTurn += 360;

        double distanceToTravel = Math.sqrt(Math.pow(robotDistanceToStone[0], 2) + Math.pow(robotDistanceToStone[1], 2));

        rotate(angleToTurn, 0.5, true);

        driveDistance(distanceToTravel, 0.5);


    }

    /**
     * This method gets the color the color sensor is reading.
     * @return The color the sensor reads. Either red, blue or notSet.
     */
    public color getColor() {

        if (colorSensor.red() >= 40 && colorSensor.blue() < 8) {
            return color.red;
        }
        else if (colorSensor.red() < 8 && colorSensor.blue() >= 40) {
            return color.blue;
        }
        else {
            return color.notSet;
        }

    }

    //inches
    public void driveDistance(double distance, double speed) {

        int wheelRadius = 2;

        int wheelRevolutions = (int) Math.round(distance/(2 * Math.PI * wheelRadius));

        driveToPos(wheelRevolutions, speed, true);
    }
}

