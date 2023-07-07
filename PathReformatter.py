from __future__ import annotations
import math
import numpy as np
import csv
import sys
import json

from typing import Any, Iterable, Tuple, List
from matplotlib import pyplot as plt
from numpy import ndarray
from pandas import read_csv, Series, DataFrame, read_json
from scipy.interpolate import CubicSpline, CubicHermiteSpline


def process_path_planner_csv(file: str, conversion_factor: float) -> Tuple[
    Any, List[float | Any], List[float | Any], List[float], Any, List[float], List[float], List[Any]]:
    data = read_csv(file)
    t = data['timeSeconds'].tolist()
    x = data[' xPositionMeters'].tolist()
    y = data[' yPositionMeters'].tolist()

    heading = data[' holonomicRotationDegrees'].tolist()

    lin_vel = data[' velocityMetersPerSecond'].tolist()
    ang_vel = data[' holonomicAngularVelocityDegreesPerSec'].tolist()

    directionalMotion = data[' headingDegrees'].tolist()

    x_offset = x[0]
    y_offset = y[0]
    heading_offset = heading[0]

    x = [(position - x_offset) * -conversion_factor for position in x]
    y = [(position - y_offset) * conversion_factor for position in y]

    # events = [[(x_pos - x_offset) * -1, y_pos-y_offset, name] for x_pos, y_pos, name in events]

    heading_shifted = [position + 180 for position in heading]
    last_direction = heading_shifted[0]
    heading_offset = heading_shifted[0]
    overflows = 0
    heading_filtered = list()
    for direction in heading_shifted:
        if direction - last_direction > 180:
            overflows -= 1
        elif direction - last_direction < -180:
            overflows += 1
        heading_filtered.append(math.radians(overflows * 360 + direction - heading_offset))
        last_direction = direction

    ang_vel = [math.radians(omega) for omega in ang_vel]

    directionalMotion = [math.radians((direction % 360) - 90) for direction in directionalMotion]
    return t, x, y, heading_filtered, lin_vel, ang_vel, directionalMotion, [x_offset, y_offset]


# events is [[x, y, name]]
def export_reformatted_csv(file_name, t, x, y, heading, lin_vel, ang_vel, directionalMotion, events):
    with open(file_name, "w", newline='') as file:
        csv_writer = csv.writer(file)
        j = 0
        for i in range(0, len(x)):
            event = ""
            if j < len(events):
                if math.isclose(x[i], events[j][0], rel_tol=.01) and math.isclose(y[i], events[j][1], rel_tol=.01):
                    event = events[j][2]
                    events[j][0] = x[i]
                    events[j][1] = y[i]
                    j += 1
            csv_writer.writerow([t[i], x[i], y[i], heading[i], lin_vel[i], ang_vel[i], directionalMotion[i], event])
        file.flush()
        file.close()

    return events


def display_charts(linear_eval_space: ndarray, x_eval: ndarray, y_eval: ndarray, heading: ndarray, events):
    figure, axis = plt.subplots(2, 1)
    axis[0].plot(x_eval, y_eval)
    axis[0].plot([i[0] for i in events], [i[1] for i in events], 'o')
    axis[0].set_title("actual path")
    axis[1].plot(linear_eval_space, heading)
    axis[1].set_title("heading")
    plt.show()


def process_path_json(file_path: str):
    data = json.load(open(file_path, "r"))
    return data


if __name__ == '__main__':

    filename = 'three_piece_nobump'

    t, y, x, heading, lin_vel, ang_vel, directional_motion, offsets = process_path_planner_csv('input.csv', 1)

    event_poses = [
        [1, "intake.extend"],
        [2, "intake.runIn"],
        [4, "intake.stop"],
        [6, "intake.runOut"],
        [7, "intake.stop"],
        [8, 'intake.runIn'],
        [10, 'intake.stop'],
        [12, 'intake.runOut']
    ]

    waypoints = process_path_json("input.json")['waypoints']

    for i in range(len(event_poses)):
        wp_num = event_poses[i][0]
        ev_name = event_poses[i][1]
        event_poses[i] = [waypoints[wp_num]['anchorPoint']['y'], waypoints[wp_num]['anchorPoint']['x'], ev_name]

    for i in range(len(event_poses)):
        event_poses[i][0] = event_poses[i][0] - offsets[1]
        event_poses[i][1] = offsets[0] - event_poses[i][1]

    event_poses = export_reformatted_csv('blue_'+ filename + '.csv', t, x, y, heading, lin_vel, ang_vel, directional_motion, event_poses)
    export_reformatted_csv('red_'+ filename + '.csv', t, [-i for i in x], y, [-i for i in heading], lin_vel, ang_vel, [-i for i in directional_motion], [[-x, y, name] for x, y, name in event_poses])
    display_charts(t, [-i for i in x], y, [-i for i in heading], [[-x, y, name] for x, y, name in event_poses])