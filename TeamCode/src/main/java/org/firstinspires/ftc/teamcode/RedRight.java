//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "BlueLeft", group = "Tau")
@Disabled
public class RedRight extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        int numberOfRings = 0;

        //initializing robot
        initializeRobot();
        //shoot?
        //detect number of rings
        switch (numberOfRings){
            case 0:
                strafeRight(.5, 36);//move right 1.5 square
                forward(.5,72);//move forward 3 square
                //drop wobble goal
                break;
            case 1:
                strafeRight(.5,12);//move right half a square
                forward(.5, 96);//move forward 4 squares
                //drop wobble goal
                backward(.5, 24);//move back 1 square
                break;
            case 4:
                strafeRight(.5, 36);//move right 1.5 a squares
                forward(.5,120);//move forward 5 squares
                //drop wobble goal
                backward(.5, 48);//move back 2 square
                break;
        }

    }
}
