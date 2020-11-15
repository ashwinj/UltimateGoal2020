//for testing autonomous methods

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "test run", group = "Tau")
public class Test extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Starting RunOpMode");
        //sleep(5000);
        //initializing robot
        initializeRobot();
        int numRings = findNumRings(bmp);
        telemetry.addData("rings", numRings);
        telemetry.update();
        sleep(10000);



    }
}
