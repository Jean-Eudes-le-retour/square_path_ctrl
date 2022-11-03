"""square_path_supervisor controller."""

from controller import Supervisor
from square_path_metric import SquarePathMetric

import os
import sys

# Constant used for the automated benchmark evaluation script
# - can also be used to generate an animation in storage folder if set to True
RECORD_ANIMATION = False

if RECORD_ANIMATION:
    import recorder.recorder as rec

# Set to true to enable information displayed in labels.
ALLOW_LABELS = False

# The color used for the labels.
TEXT_COLOR = 0x0000ff


metric = SquarePathMetric(True)


# Create the Supervisor instance.
supervisor = Supervisor()

# Gets the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# Gets the reference to the robot.
pioneer = supervisor.getFromDef('PIONEER')

if RECORD_ANIMATION:
    # Recorder code: wait for the controller to connect and start the animation
    rec.animation_start_and_connection_wait(supervisor)
    step_max = 1000 * rec.MAX_DURATION / timestep
    step_counter = 0

# Main loop starts here.
while (supervisor.step(timestep) != -1 and
       not metric.isBenchmarkOver()):

    # Recovers current time and position/orientation of the robot.
    pos = pioneer.getPosition()
    pos2d = [pos[0], -pos[1]]

    orientation = pioneer.getOrientation()
    time = supervisor.getTime()

    metric.update(pos2d, orientation, time)

    if ALLOW_LABELS:
        metric.updateLabels(0x0000ff, supervisor, time)

    supervisor.wwiSendText('update:' + metric.getWebMetricUpdate())

    for pointMessage in metric.getWebNewPoints():
        supervisor.wwiSendText(pointMessage)
    
    if RECORD_ANIMATION:
        # Stops the simulation if the controller takes too much time
        step_counter += 1
        if step_counter >= step_max:
            break

if RECORD_ANIMATION:
    # Write performance to file, stop recording and close Webots
    rec.record_performance(not metric.isBenchmarkOver(), metric.getPerformance())
    rec.animation_stop(supervisor, timestep)
    supervisor.simulationQuit(0)
else:
    print(f"Benchmark finished with a performance of {metric.getPerformance()*100:.2f}%")

supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
