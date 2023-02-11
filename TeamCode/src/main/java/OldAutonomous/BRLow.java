package OldAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.Variables.*;

import org.firstinspires.ftc.teamcode.DriveMethods;

@Autonomous(name ="BRLow", group = "A")
@Disabled
public class BRLow extends DriveMethods {
    public void runOpMode() {
        globalTargetRotation = 0;
        initMotorsBlue();

        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.05, Direction.FORWARD, 0.35, globalTargetRotation);
        driveForDistance(0.19, Direction.RIGHT, 0.35, globalTargetRotation);
        driveForDistance(0.15, Direction.FORWARD, 0.35, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.3, Direction.BACKWARD, 0.35, globalTargetRotation);

        GoToHeight(0);

        driveForDistance(0.5, Direction.RIGHT, .35, globalTargetRotation);

        while(opModeIsActive()) {

        }
    }



}
