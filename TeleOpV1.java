package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "TeleOp")
public class TeleOpV1 extends LinearOpMode {

    //UI Screen Number
    static int screenNum = 1;

    static boolean telemetryOn = false;

    Hardware robot = new Hardware();

    public void userInterface() {

        //Screen #1
        if (screenNum == 1) {
            telemetry.addLine("MENU");
            telemetry.addLine("(controlled by gamepad.2)");
            telemetry.addLine();
            telemetry.addLine("Telemetry (X)");
            telemetry.addLine("Adjust Power (Y)");
            telemetry.addLine("Run Autonomous (B)");
            telemetry.addLine("Additional Settings (A)");

            if (gamepad2.x) {
                telemetryOn = true;
                screenNum = 2;
            }
            else if (gamepad2.y) {
                screenNum = 3;
            }
            else if (gamepad2.b) {
                screenNum = 4;
            }
            else if (gamepad2.a) {
                screenNum = 5;
            }
        }

        //Screen #2
        if (screenNum == 2) {
            telemetry.addLine("Exit (X)");

            if (gamepad2.x) {
                telemetryOn = false;
                screenNum = 1;
            }
        }

        //Screen 3
        if (screenNum == 3) {

        }

        //Screen 4
        if (screenNum == 4) {

        }

        //Screen 5
        if (screenNum == 5) {

        }
    }

    //Power Variables
    static double flPower = 0, frPower = 0, brPower = 0, blPower = 0;

    public void runOpMode() {

        //Hardware Map
        robot.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {

            //Mecanum
            flPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            frPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            blPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            brPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;

            //Range Clips
            flPower = Range.clip(flPower, -1, 1);
            frPower = Range.clip(frPower, -1, 1);
            blPower = Range.clip(blPower, -1, 1);
            blPower = Range.clip(blPower, -1, 1);

            //Set Motor Power
            robot.frontLeftMotor.setPower(flPower);
            robot.frontRightMotor.setPower(frPower);
            robot.backLeftMotor.setPower(blPower);
            robot.backRightMotor.setPower(brPower);

            //Intake Power, sets direction
            if (gamepad1.x) {
                robot.intake(Hardware.IntakeDirection.in, 0.5);
            }
            else if (gamepad1.y) {
                robot.intake(Hardware.IntakeDirection.out, 0.5);
            }
            else {
                robot.intake(Hardware.IntakeDirection.off);
            }

            //Telemetry Display
            if (telemetryOn) {
                telemetry.addData("Color", robot.getColor());
                telemetry.addData("Front Distance (cm)", robot.rangeSensorFront.getDistance(DistanceUnit.CM));
                telemetry.addData("Side Distance (cm)", robot.rangeSensorSide.getDistance(DistanceUnit.CM));
            }

            //User Interface
            userInterface();
        }
    }
}
