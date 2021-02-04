//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "RedLeft", group = "Tau")
public class RedLeft extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        int numberOfRings = 0;
        double a = 48;//inches/square

        //initializing robot
        initializeRobot();
        //detect number of rings
        numberOfRings = findNumRings(bmp);
        bmp.recycle();

        //pick up wobble goal
        controlClawServo(1);
        controlArmServo(.75);

        //shoot
        setShooterPower(.45);
        forward(.75, a*2);
        powerShot(-20, -16,-10, .45, .45, .45, 0);
        toAngle(.2, 0);

        switch (numberOfRings){
            case 0:
                //move to square
                forward(.75, a);
                strafeRight(.5, a*.5);//move right half a square
                //drop wobble goal
                controlArmServo(0);
                controlClawServo(0);
                controlArmServo(.75);
                //move back
                break;
            case 1:
                strafeLeft(.5, 12);//move left half a square
                forward(.5, 96);//move forward 4 squares
                //drop wobble goal
                backward(.5, 24);//move back 1 square
                break;
            case 4:
                strafeRight(.5, 12);//move right half a square
                forward(.5,120);//move forward 5 squares
                //drop wobble goal
                backward(.5, 48);//move back 2 square
                break;
        }

    }
}
