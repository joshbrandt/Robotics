package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Josh on 1/29/2018.
 */

@Autonomous(name="NeckingRobot")
public class NeckRobot extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;

   static  IntegratingGyroscope gyro;
   static ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    BNO055IMU imu;
    Orientation angles;

    Servo neckArm;

    CRServo rangeSensorSwivel;

    DcMotor leftMotor, rightMotor;

    enum State {
        clear, obstructed
    }

    enum rangeSensorPos {
        forward, backward, left, right
    }

    enum rbtState {
        goingForward, goingBackward, goingLeft, goingRight
    }

    static State rightState = State.obstructed;
    static State leftState = State.obstructed;
    static State forwardState = State.obstructed;
    static State backwardState = State.obstructed;

    static rangeSensorPos rsPos = rangeSensorPos.forward;

    static rbtState robotState = rbtState.goingForward;


    public void runOpMode() {


        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        neckArm = hardwareMap.get(Servo.class, "neckArm");

        rangeSensorSwivel = hardwareMap.get(CRServo.class, "ss");

        leftMotor = hardwareMap.get(DcMotor.class, "lm");
        rightMotor = hardwareMap.get(DcMotor.class, "rm");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        //Loop while stop isn't pressed
        while (opModeIsActive()) {

            scanSurroundings(modernRoboticsI2cGyro);

            //Compose telemetry
            telemetry.addLine().addData("Distance (cm)", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addLine().addData("Tentative Robot State", robotState);
            telemetry.addLine().addData("Range Sensor Pos", rsPos);
            telemetry.addLine().addData("Tentative Forward State", forwardState);
            telemetry.addLine().addData("Tentative Right State", rightState);
            telemetry.addLine().addData("Tentative Backward State", backwardState);
            telemetry.addLine().addData("Tentative Right State", rightState);
            telemetry.addLine().addData("Robot Heading", angles.firstAngle);
            telemetry.addLine().addData("Range Sensor Heading", modernRoboticsI2cGyro.getHeading());
            telemetry.update();


            //If it wants to go forward, first make sure its orientated at 0 degrees then move forward
            if (robotState == rbtState.goingForward && forwardState == State.clear) {

                if (angles.firstAngle == 0) {
                    drive(1);
                } else {
                    turn (0, 0.5);
                    drive(1);
                }

            }

            //If it wants to go backward, first make sure its orientated at 0 degrees and then move backward
            if (robotState == rbtState.goingBackward && backwardState == State.clear) {

                if (angles.firstAngle == 0) {
                    drive(-1);
                } else {
                    turn (0, 0.5);
                    drive(-1);
                }
            }

            //If it wants to go right, first make sure its orientated at -90 degrees then move forward
            if (robotState == rbtState.goingRight && rightState == State.clear) {

                if (angles.firstAngle == -90) {
                    drive(1);
                } else {
                    turn (-90, 0.5);
                    drive(1);
                }

            }

            //If it wants to go left, first make sure its orientated at 90 degrees and then move forward
            if (robotState == rbtState.goingLeft && leftState == State.clear) {

                if (angles.firstAngle == 90) {
                    drive(1);
                } else {
                    turn (90, 0.5);
                    drive(1);
                }
            }
        }
    }


    public void resetEncoders() {

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void brakeMotors() {

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    public void drive(double power) {

        leftMotor.setPower(power);
        rightMotor.setPower(power);

    }

    public void sleepThread(long time) {

        try {
            Thread.sleep(time * 1000);

        } catch (InterruptedException e) {}


    }

    public void turn(int destinationDegree, double power) {

        while ((((angles.firstAngle > 0 && destinationDegree > 0) || (angles.firstAngle < 0 && destinationDegree < 0)) && angles.firstAngle > destinationDegree) && opModeIsActive()) {

            rightMotor.setPower(power);
            leftMotor.setPower(-power);
        }
        brakeMotors();

        while ((((angles.firstAngle > 0 && destinationDegree > 0) || (angles.firstAngle < 0 && destinationDegree < 0)) && angles.firstAngle < destinationDegree) && opModeIsActive()) {

            rightMotor.setPower(-power);
            leftMotor.setPower(power);

        }
        brakeMotors();


        while ((angles.firstAngle > 90 && destinationDegree > -90) && opModeIsActive()) {

            rightMotor.setPower(power);
            leftMotor.setPower(-power);
        }
        brakeMotors();

        while ((angles.firstAngle < 90 && destinationDegree < -90) && opModeIsActive()) {

            rightMotor.setPower(-power);
            leftMotor.setPower(power);

        }
        brakeMotors();


    }

    public void scanSurroundings(ModernRoboticsI2cGyro gyro) {


       boolean scanDone = false;

       while (!scanDone) {

           //If the range is more than 10, there's no object in the immediate area, so where ever its looking is clear
           if (rangeSensor.getDistance(DistanceUnit.CM) > 10) {

               switch (rsPos) {

                   case forward:
                       forwardState = State.clear;
                       break;

                   case backward:
                       backwardState = State.clear;
                       break;

                   case right:
                       rightState = State.clear;
                       break;

                   case left:
                       leftState = State.clear;
                       break;

               }
           }


           //If the range is less than 10, there's an object in the immediate area, so where ever its looking is obstructed
           if (rangeSensor.getDistance(DistanceUnit.CM) < 10) {

               switch (rsPos) {

                   case forward:
                       forwardState = State.obstructed;
                       break;

                   case backward:
                       backwardState = State.obstructed;
                       break;

                   case right:
                       rightState = State.obstructed;
                       break;

                   case left:
                       leftState = State.obstructed;
                       break;

               }
           }


           //If the robot is going forward but forward is obstructed, scan to see if the left is obstructed
           if (robotState == rbtState.goingForward && forwardState == State.obstructed) {

               while (gyro.getHeading() < 90) {
                   rangeSensorSwivel.setPower(-0.5);
               }
               rangeSensorSwivel.setPower(0);

               while (gyro.getHeading() > 90) {
                   rangeSensorSwivel.setPower(0.5);
               }
               rangeSensorSwivel.setPower(0);

               sleepThread(1);

               robotState = rbtState.goingLeft;
               rsPos = rangeSensorPos.left;

           }

           //If the robot is going left but left is obstructed, scan to see if behind is obstructed
           if (robotState == rbtState.goingLeft && leftState == State.obstructed) {

               while (gyro.getHeading() < 180) {
                   rangeSensorSwivel.setPower(-0.5);
               }
               rangeSensorSwivel.setPower(0);

               while (gyro.getHeading() > 180) {
                   rangeSensorSwivel.setPower(0.5);
               }
               rangeSensorSwivel.setPower(0);

               sleepThread(1);

               robotState = rbtState.goingBackward;
               rsPos = rangeSensorPos.backward;


           }

           //If the robot is going backward but backward is obstructed, scan to see if right is obstructed
           if (robotState == rbtState.goingBackward && backwardState == State.obstructed) {

               while (gyro.getHeading() < 270) {
                   rangeSensorSwivel.setPower(-0.5);
               }
               rangeSensorSwivel.setPower(0);


               while (gyro.getHeading() > 270) {
                   rangeSensorSwivel.setPower(0.5);
               }
               rangeSensorSwivel.setPower(0);

               sleepThread(1);

               robotState = rbtState.goingRight;
               rsPos = rangeSensorPos.right;


           }

           //If the robot is going right but right is obstructed, scan to see if behind is obstructed
           if (robotState == rbtState.goingRight && rightState == State.obstructed) {

               while (gyro.getHeading() < 180) {
                   rangeSensorSwivel.setPower(0.5);
               }
               rangeSensorSwivel.setPower(0);

               while (gyro.getHeading() > 180) {
                   rangeSensorSwivel.setPower(-0.5);
               }
               rangeSensorSwivel.setPower(0);

               sleepThread(1);

               robotState = rbtState.goingBackward;
               rsPos = rangeSensorPos.backward;

           }


           //If the robot wants to move a direction and that direction is clear then no need to scan for obstructions until the next cycle

           if (robotState == rbtState.goingForward && forwardState == State.clear) {
               scanDone = true;
           }

           if (robotState == rbtState.goingBackward && backwardState == State.clear) {
               scanDone = true;
           }

           if (robotState == rbtState.goingRight && rightState == State.clear){
               scanDone = true;
           }

           if (robotState == rbtState.goingLeft && leftState == State.clear) {
               scanDone = true;
           }

       }
    }
    }

