package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Opmode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsFarBlue;

@Autonomous(name = "Hunter Blue Far", preselectTeleOp = "TeleOp")

public class HunterFarAutoBlue extends FarAuto {
    @Override
    public Paths getPaths() {
        return new PathsFarBlue(pedro);
    }
}
