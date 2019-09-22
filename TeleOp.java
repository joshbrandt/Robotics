package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * The TeleOp class extends the Hardware class to inherit various
 * methods and initialized hardware components specific to the
 * hardware of the robot.
 */
@
public class TeleOp extends Hardware {

        @Override
        public void runOpMode() throws InterruptedException {

            hardwareMap();

            double frPower, flPower, rrPower, rlPower = 0;

            while (opModeIsActive()) {
                //Mecanum Drive
                frPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
                flPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
                rrPower = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
                rlPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            }
        }
    }
}
