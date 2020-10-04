//Basic tank drive

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Drive", group = "Taus")
public class Drive extends LinearOpMode {

    Hardware robot = new Hardware(true);
    AutonomousMethods method = new AutonomousMethods();

    @Override
    public void runOpMode() {

        method.initializeRobot();

        while (opModeIsActive()) {
            drive();
            telemetry.addData("time", robot.period.seconds());
            telemetry.update();
            telemetry.clear();
        }
    }


    public void drive() {

        method.setPowerOfMotorsTo(gamepad1.left_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_y);


    }
}

