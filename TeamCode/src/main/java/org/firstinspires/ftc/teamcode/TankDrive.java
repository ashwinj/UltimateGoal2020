//Tank Drive


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TankDrive", group = "Taus")

public class TankDrive extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = false;
    boolean isBPressed = false;
    boolean clawClosed = false;

    @Override
    public void runOpMode() {

        method.robot.initializeHardware(hardwareMap);
        method.runWithouthEncoders();
        method.robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        method.robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        method.robot.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        method.robot.backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Waiting for Start Button");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);



        while (opModeIsActive()) {

            //drive();
            shooter();
            intake();
            claw();
            telemetry.addLine("");
            telemetry.addData("Time ::", method.robot.period.seconds());
            telemetry.addData("isAPressed ::", isAPressed);
            telemetry.addData("Shooter On ::", shooterOn);
            telemetry.addData("isBPressed ::", isBPressed);
            telemetry.addData("Claw closed ::", clawClosed);
            telemetry.addData("Left stick ::", -gamepad1.left_stick_y);
            telemetry.addData("Right stick ::", gamepad1.right_stick_y);

            telemetry.update();
            telemetry.clear();
        }


        method.setAllMotorsTo(0);
    }


    public void drive() {
        method.setPowerOfMotorsTo(-gamepad1.left_stick_y, -gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_y);

    }

    public void shooter(){

        if(gamepad1.a && !isAPressed){
            isAPressed = true;
            if (!shooterOn) {
              method.setShooterPower(1);
              shooterOn = true;
            }
            else{
             method.setShooterPower(0);
             shooterOn = false;
            }
        }
        if(!gamepad1.a){
            isAPressed = false;
        }

    }

    public void intake(){

        if(gamepad1.right_bumper){
           method.setIntakePower(1);
        }
        else if (gamepad1.left_bumper){
            method.setIntakePower(-1);
        }
        else{
          method.setIntakePower(0);
        }

    }

    public void claw(){

        if(gamepad1.b && !isBPressed){
            isBPressed = true;
            if (!clawClosed) {
                method.controlClawServo(.25);//closing claw
                method.controlArmServo(0);//move arm up
                clawClosed = true;
            }
            else{
                method.controlArmServo(1);//moving arm down
                method.controlClawServo(.7);//opening claw

                clawClosed = false;
            }
        }
        if(!gamepad1.b){
            isBPressed = false;
        }

    }
}








