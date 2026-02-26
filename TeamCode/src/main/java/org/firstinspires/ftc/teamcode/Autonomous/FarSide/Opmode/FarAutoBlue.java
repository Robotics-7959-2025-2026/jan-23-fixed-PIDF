package org.firstinspires.ftc.teamcode.Autonomous.FarSide.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPathsBlue;

//Aries spears: "this fellas a homosexual"
@Autonomous(name = "Keir Far Blue", preselectTeleOp = "TeleOp")
public class FarAutoBlue extends FarAuto {
    @Override
    public FarPaths getPaths() {
        return new FarPathsBlue(pedro);
    }
}
