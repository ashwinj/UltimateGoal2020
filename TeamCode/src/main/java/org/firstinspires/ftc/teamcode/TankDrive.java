//Tank Drive


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "TankDrive", group = "Taus")

public class TankDrive extends LinearOpMode {

    public Hardware robot = new Hardware(true);

    @Override
    public void runOpMode() {

        robot.initializeHardware(hardwareMap);

        telemetry.addLine("Waiting for Start Button");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);



        while (opModeIsActive()) {

            drive();
            telemetry.addLine("");
            telemetry.addData("Time ::", robot.period.seconds());
            telemetry.update();
            telemetry.clear();
        }


        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

    }


    public void drive() {
        robot.frontLeftMotor.setPower(-gamepad1.left_stick_y);
        robot.backLeftMotor.setPower(-gamepad1.left_stick_y);
        robot.frontRightMotor.setPower(gamepad1.left_stick_y);
        robot.backRightMotor.setPower(gamepad1.left_stick_y);

    }
}








