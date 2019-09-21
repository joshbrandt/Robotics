package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The Hardware class initializes various hardware components and variables as
 * well as condensing various processes into specific methods.
 */
@TeleOp (name = "TeleOp V1")
public abstract class Hardware extends LinearOpMode{

    DcMotor frontLeft;

    /**
     * Maps the hardware of every component in the robot.
     */
    public void hardwareMap()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
    }

}
