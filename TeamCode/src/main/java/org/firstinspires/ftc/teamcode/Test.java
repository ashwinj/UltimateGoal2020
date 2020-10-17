//for testing autonomous methods

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "test run", group = "Tau")

public class Test extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        right(.2, 90);
        left(.2, 90);
        //forward(1, 100);

    }
}
