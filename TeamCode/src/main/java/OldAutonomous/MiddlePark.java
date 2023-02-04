package OldAutonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Variables.*;

import android.graphics.drawable.GradientDrawable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveMethods;

@Autonomous(name ="MiddlePark", group = "A")
@Disabled
public class MiddlePark extends DriveMethods {
    public void runOpMode() {
        globalTargetRotation = 0;
        initMotorsBlue();

        waitForStart();

        driveForDistance(0.4, Direction.FORWARD, 0.5, globalTargetRotation);

        while(opModeIsActive()) {

        }


    }



}
