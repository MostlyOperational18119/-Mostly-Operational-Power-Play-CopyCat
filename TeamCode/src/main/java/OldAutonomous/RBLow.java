package OldAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.Variables.*;

import org.firstinspires.ftc.teamcode.DriveMethods;

@Autonomous(name ="RBLow", group = "A")
@Disabled
public class RBLow extends DriveMethods {
    public void runOpMode() {
        globalTargetRotation = 0;
        initMotorsBlue();

        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.05, Direction.FORWARD, 0.3, globalTargetRotation);
        driveForDistance(0.25, Direction.LEFT, 0.3, globalTargetRotation);
        driveForDistance(0.17, Direction.FORWARD, 0.3, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.3, Direction.BACKWARD, 0.5, globalTargetRotation);

        GoToHeight(0);

        driveForDistance(0.5, Direction.LEFT, 0.5, globalTargetRotation);

        while(opModeIsActive()) {

        }
    }



}
