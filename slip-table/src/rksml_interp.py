# Script adopted from Steven Myint's M2020 SOPS rksml_lib.py

#!/usr/bin/env python3

import numpy as np
import pandas as pd
import sys
import defusedxml.ElementTree as xml
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

pd.set_option("display.max_rows", None, "display.max_columns", None)
pd.set_option('display.float_format', '{:.6f}'.format)

def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n
    
def load_rksml_file(rksml_file, start_sclk=None, end_sclk=None, crop_params=None):
    if rksml_file is None:
        return None
    print('Loading RKSML: {}'.format(rksml_file))

    start_sclk_float = float(
        '-inf') if start_sclk is None else float(start_sclk)
    end_sclk_float = float('inf') if end_sclk is None else float(end_sclk)

    try:
        # Using defusedxml for secure XML parsing
        rksml_xml = xml.parse(rksml_file).getroot()
    except xml.ParseError:
        print('ERROR: Unable to parse RKSML: {}'.format(rksml_file))
        sys.exit(1)

    state_history = rksml_xml.find('./{RPK}State_History')

    states = {}
    for node in state_history.findall('./{RPK}Node'):
        # only load RKSML nodes within specified SCLK range
        node_time_float = float(node.get('Time'))
        if node_time_float < start_sclk_float or node_time_float > end_sclk_float:
            continue

        knot_dict = {'SCLK': node_time_float}
        for knot in node.findall('{RPK}Knot'):
            units = knot.get('Units', None)
            if units is None:
                if knot.text in ['TRUE', 'FALSE']:
                    knot_dict[knot.get('Name')] = knot.text
                else:
                    knot_dict[knot.get('Name')] = float(knot.text)
            else:
                knot_dict['{} [{}]'.format(
                    knot.get('Name'), units)] = float(knot.text)

        # use the SCLK string ad key and later use pandas to convert to float64 index
        states[node.get('Time')] = knot_dict

    # create DataFrame containing RKSML data, indexed by SCLK
    rksml_df = pd.DataFrame.from_dict(states, orient='index')

    # convert the SCLK indexes from strings to float64s
    rksml_df.set_index(rksml_df.index.astype('float64'), inplace=True)

    # use quaternion to calculate roll, pitch, yaw, and tilt
    rksml_df['ROLL_FROM_QUAT [RADIANS]'] = np.arctan2(
        2 * (rksml_df['QUAT_C'] * rksml_df['QUAT_X'] +
             rksml_df['QUAT_Y'] * rksml_df['QUAT_Z']),
        1 - 2 * (np.power(rksml_df['QUAT_X'], 2) + np.power(rksml_df['QUAT_Y'], 2)))

    rksml_df['PITCH_FROM_QUAT [RADIANS]'] = np.arcsin(
        2 * (rksml_df['QUAT_C'] * rksml_df['QUAT_Y'] - rksml_df['QUAT_Z'] * rksml_df['QUAT_X']))

    rksml_df['YAW_FROM_QUAT [RADIANS]'] = np.arctan2(
        2 * (rksml_df['QUAT_C'] * rksml_df['QUAT_Z'] +
             rksml_df['QUAT_X'] * rksml_df['QUAT_Y']),
        1 - 2 * (np.power(rksml_df['QUAT_Y'], 2) + np.power(rksml_df['QUAT_Z'], 2)))

    rksml_df['TILT_FROM_QUAT [RADIANS]'] = np.fabs(2 * np.arccos(
        np.cos(rksml_df['ROLL_FROM_QUAT [RADIANS]'] / 2) *
        np.cos(rksml_df['PITCH_FROM_QUAT [RADIANS]'] / 2)))

    # ### identify the highest rate attitude channels
    # if rksml_df['ROLL_FROM_QUAT [RADIANS]'].count() > rksml_df['ROVER_R [RADIANS]'].count():
    # 	rksml_df['HIGH_RATE_ROLL [RADIANS]'] = rksml_df['ROLL_FROM_QUAT [RADIANS]']
    # else:
    # 	rksml_df['HIGH_RATE_ROLL [RADIANS]'] = rksml_df['ROVER_R [RADIANS]']

    # if rksml_df['PITCH_FROM_QUAT [RADIANS]'].count() > rksml_df['ROVER_P [RADIANS]'].count():
    # 	rksml_df['HIGH_RATE_PITCH [RADIANS]'] = rksml_df['PITCH_FROM_QUAT [RADIANS]']
    # else:
    # 	rksml_df['HIGH_RATE_PITCH [RADIANS]'] = rksml_df['ROVER_P [RADIANS]']

    # if rksml_df['YAW_FROM_QUAT [RADIANS]'].count() > rksml_df['ROVER_H [RADIANS]'].count():
    # 	rksml_df['HIGH_RATE_YAW [RADIANS]'] = rksml_df['YAW_FROM_QUAT [RADIANS]']
    # else:
    # 	rksml_df['HIGH_RATE_YAW [RADIANS]'] = rksml_df['ROVER_H [RADIANS]']

    # rksml_df['HIGH_RATE_TILT [RADIANS]'] = np.fabs(2 * np.arccos(
    # 	np.cos(rksml_df['HIGH_RATE_ROLL [RADIANS]'] / 2) *
    # 	np.cos(rksml_df['HIGH_RATE_PITCH [RADIANS]'] / 2)))

    # fill NAN values for a select set of channels
    fillna_channels = [
        'ROVER_X [METERS]', 'ROVER_Y [METERS]', 'ROVER_Z [METERS]',
        # 'HIGH_RATE_ROLL [RADIANS]', 'HIGH_RATE_PITCH [RADIANS]',
        # 'HIGH_RATE_YAW [RADIANS]', 'HIGH_RATE_TILT [RADIANS]',
        'LEFT_BOGIE [RADIANS]', 'RIGHT_BOGIE [RADIANS]',
        'LEFT_DIFFERENTIAL [RADIANS]', 'RIGHT_DIFFERENTIAL [RADIANS]',
        'LF_STEER [RADIANS]', 'RF_STEER [RADIANS]',
        'RR_STEER [RADIANS]', 'LR_STEER [RADIANS]',
        # 'ROVER_X_SLIP [METERS]', 'ROVER_Y_SLIP [METERS]'
    ]

    for chan in fillna_channels:
        # rksml_df[chan].ffill(inplace=True)
        rksml_df[chan] = rksml_df[chan].ffill()
        # rksml_df[chan].bfill(inplace=True)
        rksml_df[chan] = rksml_df[chan].bfill()

    # create columns for degrees
    for col, vals in rksml_df.items():
        if col.endswith(' [RADIANS]'):
            rksml_df['{} [DEGREES]'.format(
                col.split(' ')[0])] = np.degrees(vals)

    # identify a useful data range for visualizations
    if crop_params is not None:
        max_hidden_nodes = crop_params[0]
        max_sclk_gap = crop_params[1]

    sclk_range = [rksml_df.index[0], rksml_df.index[-1]]

    if start_sclk is None:
        if crop_params is not None:
            for idx in range(1, max_hidden_nodes):
                if rksml_df.index[idx] - rksml_df.index[idx-1] > max_sclk_gap:
                    sclk_range[0] = rksml_df.index[idx]
    else:
        sclk_range[0] = start_sclk_float

    if end_sclk is None:
        if crop_params is not None:
            for idx in range(-1, -max_hidden_nodes, -1):
                if rksml_df.index[idx] - rksml_df.index[idx-1] > max_sclk_gap:
                    sclk_range[1] = rksml_df.index[idx-1]
    else:
        sclk_range[1] = end_sclk_float

    # identify mean, max absolute, and opposite max values
    channels = [
        'ROVER_X [METERS]', 'ROVER_Y [METERS]', 'ROVER_Z [METERS]',
        # 'HIGH_RATE_ROLL [DEGREES]', 'HIGH_RATE_PITCH [DEGREES]',
        # 'HIGH_RATE_YAW [DEGREES]', 'HIGH_RATE_TILT [DEGREES]',
        'LEFT_BOGIE [DEGREES]', 'RIGHT_BOGIE [DEGREES]',
        'LEFT_DIFFERENTIAL [DEGREES]', 'RIGHT_DIFFERENTIAL [DEGREES]']

    chan_stats = {
        'mean': {},
        'max_abs': {},
        'opp_max': {}}
    for chan in channels:
        chan_stats['mean'][chan] = rksml_df[chan][sclk_range[0]
            :sclk_range[1]].mean()
        max_val = rksml_df[chan][sclk_range[0]:sclk_range[1]].max()
        max_idx = rksml_df[chan][sclk_range[0]:sclk_range[1]].idxmax()
        min_val = rksml_df[chan][sclk_range[0]:sclk_range[1]].min()
        min_idx = rksml_df[chan][sclk_range[0]:sclk_range[1]].idxmin()
        if abs(max_val) > abs(min_val):
            chan_stats['max_abs'][chan] = (max_idx, max_val)
            chan_stats['opp_max'][chan] = (min_idx, min_val)
        else:
            chan_stats['max_abs'][chan] = (min_idx, min_val)
            chan_stats['opp_max'][chan] = (max_idx, max_val)

    # identify straight, arc, and turn-in-place steering
    steer_tol = 5e-3
    rksml_df['TURN_IN_PLACE_STEERING'] = np.logical_and(np.logical_and(
        np.abs(rksml_df['LF_STEER [RADIANS]'] - 0.840) < steer_tol,
        np.abs(rksml_df['RF_STEER [RADIANS]'] + 0.840) < steer_tol), np.logical_and(
        np.abs(rksml_df['RR_STEER [RADIANS]'] - 0.791) < steer_tol,
        np.abs(rksml_df['LR_STEER [RADIANS]'] + 0.791) < steer_tol))
    rksml_df['STRAIGHT_STEERING'] = np.logical_and(np.logical_and(
        np.abs(rksml_df['LF_STEER [RADIANS]']) < steer_tol,
        np.abs(rksml_df['RF_STEER [RADIANS]']) < steer_tol), np.logical_and(
        np.abs(rksml_df['RR_STEER [RADIANS]']) < steer_tol,
        np.abs(rksml_df['LR_STEER [RADIANS]']) < steer_tol))

    turn_steering_sclks = []
    turn_sclk0 = None
    for sclk, is_turn in rksml_df['TURN_IN_PLACE_STEERING'].items():
        if is_turn:
            if turn_sclk0 is None:
                turn_sclk0 = sclk
        else:
            if turn_sclk0 is not None:
                turn_steering_sclks.append((turn_sclk0, sclk))
                turn_sclk0 = None

    if turn_sclk0 is not None:
        # rksml ends with wheels steered
        turn_steering_sclks.append((turn_sclk0, sclk))

    # print some useful statistics to the terminal
    # print('- Max Tilt: {:.2f}deg'.format(chan_stats['max_abs']['HIGH_RATE_TILT [DEGREES]'][1]))
    print('- Max Absolute Bogie: {:.2f}deg'.format(max(
        abs(chan_stats['max_abs']['LEFT_BOGIE [DEGREES]'][1]),
        abs(chan_stats['max_abs']['RIGHT_BOGIE [DEGREES]'][1]))))
    print('- Max Absolute Differential: {:.2f}deg'.format(max(
        abs(chan_stats['max_abs']['LEFT_DIFFERENTIAL [DEGREES]'][1]),
        abs(chan_stats['max_abs']['RIGHT_DIFFERENTIAL [DEGREES]'][1]))))
    # FIXME print('- Distance: {.2f}m'.format(drive_dist))
    # print('- Final Tilt: {:.2f}deg'.format(rksml_df['HIGH_RATE_TILT [DEGREES]'].iloc[-1]))
    # print('- Final Yaw: {:.2f}deg'.format(rksml_df['HIGH_RATE_YAW [DEGREES]'].iloc[-1]))

    return {'data': rksml_df, 'range': sclk_range,
            'stats': chan_stats, 'turns': turn_steering_sclks}


if __name__ == "__main__":

    res = load_rksml_file("../downlink/sol01294_playback_vo_corrected.rksml")

    data = res['data']
    data = data[data['SCLK'] > res['range'][0]]
    data = data[data['SCLK'] < res['range'][1]]
    

    # Choose only relevant channels for export
    data = data[['SCLK', 'ROVER_X [METERS]', 'ROVER_Y [METERS]', 'ROVER_Z [METERS]', 'QUAT_X', 'QUAT_Y', 'QUAT_Z', 'QUAT_C',
                 'LF_STEER [RADIANS]', 'RF_STEER [RADIANS]', 'RR_STEER [RADIANS]', 'LR_STEER [RADIANS]', 'LEFT_DIFFERENTIAL [RADIANS]', 'RIGHT_DIFFERENTIAL [RADIANS]', 'LEFT_BOGIE [RADIANS]',
                 'RIGHT_BOGIE [RADIANS]', 'TURN_IN_PLACE_STEERING', 'STRAIGHT_STEERING', 'JOINT1_ENC [RADIANS]','JOINT2_ENC [RADIANS]','JOINT3_ENC [RADIANS]','JOINT4_ENC [RADIANS]','JOINT5_ENC [RADIANS]',
                 ]]

    slip_df = pd.read_csv("../downlink/sol01294_slip_incons.txt")
    
    
    rotations = data[['QUAT_X','QUAT_Y','QUAT_Z','QUAT_C']]
    r = R.from_quat(rotations).as_euler(seq="XYZ",degrees=True)
    
    
    print(r) 
    