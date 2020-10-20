//for testing autonomous methods

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "test run", group = "Tau")
@Disabled
public class Test extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        forward(.5, 12);
        backward(.5, 12);
        strafeLeft(.5, 12);
        strafeRight(.5, 12);
        right(.2, 90);
        left(.2, 90);
        //forward(1, 100);

    }
}
