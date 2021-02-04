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
        bmp.recycle()
        telemetry.addData("rings", numRings);
        telemetry.update();
        forward(1, 48);
        backward(1, 48);
        strafeRight(1, 48);
        strafeLeft(1, 48);
        toAngle(1, -90);
        toAngle(1, 90);



    }
}
