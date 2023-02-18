package TestPrograms;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.TinyWebServer;

public class HTTPTeleOp extends DriveMethods {
    private int port;
    private String ipaddr;
    private Boolean srvRunning;

    @Override
    public void runOpMode() {
        ipaddr = "192.168.43.1";
        port = 6969;

        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        TinyWebServer.startServer(ipaddr,port,"/");
        srvRunning = true;
        while (opModeIsActive()) {
            if (gamepad1.a && !srvRunning) {
                TinyWebServer.startServer(ipaddr,port,"/");
            }
            if (gamepad1.b && srvRunning) {
                TinyWebServer.stopServer();
            }
            if (srvRunning) {
                telemetry.addLine("Currently active.");
            } else {
                telemetry.addLine("Currently stopped, start me!");
            }
            telemetry.addLine("Hosting at " + ipaddr + ":" + port + ".");
            telemetry.update();
        }
        if (srvRunning) {
            TinyWebServer.stopServer();
        }
        srvRunning = false;
    }
}
