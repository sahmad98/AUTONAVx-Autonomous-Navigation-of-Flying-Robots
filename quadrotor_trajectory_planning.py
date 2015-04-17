import quadrotor.command as cmd
from math import sqrt

def plan_mission(mission):

    # this is an example illustrating the different motion commands,
    # replace them with your own commands and activate all beacons
    commands  = [
        cmd.up(1),
        cmd.forward(6),
        cmd.turn_left(90),
        cmd.forward(2),
        cmd.turn_left(90),
        cmd.forward(6),
        cmd.turn_left(90),
        cmd.forward(4),
        cmd.turn_left(90),
        cmd.forward(6),
    ]

    mission.add_commands(commands)
