//Autonomous methods to be used in run programs

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//for reading gyro


public class AutonomousMethods extends LinearOpMode {
    public Hardware robot = new Hardware(true);
    private double gearRatio = 2;
    private double wheelDiameter = 4;
    //private double countsPerRotation = 1115; //counts per 360 degrees
    private double encoderCounts = 383.6*4; //counts per one rotation of output shaft
    private double currentxPosition = 0;
    private double currentyPosition = 0;
    public ElapsedTime runtime = new ElapsedTime();

    public void initializeRobot() {
        robot.initializeHardware(hardwareMap);
        //waiting until gyro is calibrated
        while(!robot.imu.isGyroCalibrated()){
            idle();
        }
        telemetry.addLine("ready!");
        telemetry.update();
        waitForStart();
    }

    //moving forward distance (inch) with power [0, 1]
    public void forward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading();
        int counts = (int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));
        while (counts >= robot.backLeftMotor.getCurrentPosition()) {
            setAllMotorsTo(power);
        }

        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        updatePosition(distance, angle);
    }

    //moving backward distance (inch) with power [0, 1]
    public void backward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading() + 180;
        int counts = -(int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));

        while (counts <= robot.backLeftMotor.getCurrentPosition()) {
            setAllMotorsTo(-power);
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        updatePosition(-distance, angle);
    }

    //strafing left distance (inch) with power [0, 1]
    public void strafeLeft(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading() - 90;
        int counts = (int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));
        //setting all motors to go forward (positive)
        while (counts >= robot.backLeftMotor.getCurrentPosition()) {
            setPowerOfMotorsTo(power, -power , -power , power );
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        //sleep(500);
        updatePosition(distance, angle);
    }

    //strafing right distance (inch) with power [0, 1]
    public void strafeRight(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading() + 90;
        int counts = -(int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));
        while (counts <= robot.backLeftMotor.getCurrentPosition()) {
            setPowerOfMotorsTo(-power, power, power, -power);
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
    }

    public void right(double power, double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (initAngle - getHeading() <= degrees) {
            telemetry.addData("right", initAngle - getHeading());
            telemetry.update();


            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(power);
            robot.frontLeftMotor.setPower(power);

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(-power);
            robot.frontRightMotor.setPower(-power);
        }

        //setting motor value to 0 (stop)
        stopAndResetEncoders();

    }

    public void left(double power, double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (Math.abs(getHeading() - initAngle) <= degrees) {
            telemetry.addData("left", getHeading() - initAngle);
            telemetry.update();


            //setting left motors to go backward (negitive power)
            robot.backLeftMotor.setPower(-power);
            robot.frontLeftMotor.setPower(-power);

            //setting right motors to go forward (positive power)
            robot.backRightMotor.setPower(power);
            robot.frontRightMotor.setPower(power);
        }

        //setting motor value to 0 (stop)
        stopAndResetEncoders();

    }


    public void turnRightWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
        double error = targetAngle - getHeading();
        double previousError = error;
        double correction;
        double slope;
        runtime.reset();

        while (Math.abs(error) > threshold && opModeIsActive()) {

            error = targetAngle - getHeading(); //updating error
            slope = (error - previousError) / (double) runtime.time();
            telemetry.addData("slope", slope);
            telemetry.addData("error", error);
            telemetry.update();

            runtime.reset();

            correction = (kp * error)+(kd * slope);
            setPowerOfMotorsTo(correction, correction, -correction, -correction);

        }
        setAllMotorsTo(0);
        stopAndResetEncoders();

    }

    public void turnLeftWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
        double error = targetAngle - getHeading();
        double previousError = error;
        double correction;
        double slope = 0;
        runtime.reset();

        while (Math.abs(error) > threshold && opModeIsActive()) {

            error = targetAngle - getHeading();
            slope = (error - previousError) / (double) runtime.time();
            previousError = error;
            telemetry.addData("slope", slope);
            telemetry.addData("error", error);
            telemetry.update();

            runtime.reset();

            correction = (kp * error) + (kd * slope);
            setPowerOfMotorsTo(-correction, -correction, correction, correction);

        }

    }

    //sets the power of motors to seperate inputted powers
    public void setPowerOfMotorsTo(double bl, double fl, double br, double fr) {
        robot.backLeftMotor.setPower(bl);
        robot.backRightMotor.setPower(br);
        robot.frontRightMotor.setPower(fr);
        robot.frontLeftMotor.setPower(fl);
    }

    //goes to a position and angle on the field
    public void goToPosition(double power, double x, double y, double angle) {
        double deltaX = currentxPosition - x; //required change in x position from current to desired
        double deltaY = currentyPosition - y; //required change in y position from current to desired
        double theta = Math.atan(deltaY / deltaX) + getHeading() + 45;
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double xDist = Math.cos(theta) * distance;
        double yDist = Math.sin(theta) * distance;

        // setting scale factor to the biggest value
        // dividing by this when setting powers will make sure that speeds are in the format 0-1
        // will also insure speeds are proportional to distance needed to travel in each direction so robot can move at angle theta
        double scaleFactor = xDist;
        if (yDist > xDist) {
            scaleFactor = yDist;
        }
        int counts = (int) ((xDist / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));

        setPowerOfMotorsTo(yDist / scaleFactor, xDist / scaleFactor, xDist / scaleFactor, yDist / scaleFactor);
        while(robot.backLeftMotor.getCurrentPosition()<counts){
            idle();
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        currentyPosition=y;
        currentxPosition=x;
    }

    //sets all motors to the same power
    public void setAllMotorsTo(double power) {
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);
    }

    //resets all wheel encoders to 0
    public void stopAndResetEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //updates position on the field
    public void updatePosition(double distance, double angle) {
        currentxPosition += Math.sin(angle) * distance;
        currentyPosition += Math.cos(angle) * distance;
    }

    //gets the angle in degrees
    public double getHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; // left [0,-180] right[0,180]
    }



    @Override
    public void runOpMode() throws InterruptedException {
    }
}

