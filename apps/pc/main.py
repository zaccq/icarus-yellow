import serial
import re
import argparse
import io
import knnAlg as knn
import dashboard as dash
import csv
import numpy as np

ansi_escape = re.compile(r'(?:\x1B[@-Z\\-_]|[\x80-\x9A\x9C-\x9F]|(?:\x1B\[|\x9B)[0-?]*[ -/]*[@-~])')
number_rexp = re.compile(r'(?:[0-9]+)')
base_sens = [2.0, 250.0]


def loop(prompt, f: io.TextIOWrapper, ser: serial.Serial):
    if (f):
        f.write('ax_s,ay_s,az_s,gx_s,gy_s,gz_s,A_s,G_s\n')

    filename = '../../data/knn_dataset.csv'
    c = knn.Classifier(filename, num_neighbours=1)
    frame = [];

    while True:

        serialString = ser.readline()
        sw = serialString.decode('ascii')

        # remove garbage
        sw_stripped = ansi_escape.sub('', sw).strip('\n\r').replace(args.prompt, '')

        # if empty or not valid
        if not sw_stripped or sw_stripped == '' or sw_stripped[0] != '%':
            continue

        reading = list(csv.reader([sw_stripped[1:]]))[0]

        s_ag = [float(scale) * (128 / base_sens[i]) for i, scale in enumerate(reading[:2])]
        a_xyz_r = [float(axis) for axis in reading[2:5]]
        g_xyz_r = [float(axis) for axis in reading[5:8]]
        q_wxyz = [float(axis) for axis in reading[-4:]]

        a_xyz = [axis / s_ag[0] for axis in a_xyz_r]
        g_xyz = [axis / s_ag[1] for axis in g_xyz_r]

        data = [*a_xyz, *g_xyz]
        data.append(np.linalg.norm(a_xyz))
        data.append(np.linalg.norm(g_xyz))
        frame.append(list(data))

        if (f):
            f.write(','.join([str(round(i, 4)) for i in data]) + '\n')

        if len(frame) != 80:
            continue
        frame = np.array(frame).transpose().tolist()

        means = [round(np.abs(np.mean(sensor)), 4) for sensor in frame]
        stdevs = [round(np.std(sensor), 4) for sensor in frame]

        out_data = [*means[:3], *[means[-1]], *stdevs]
        est = number_rexp.sub('', c.predict_classification(out_data))
        print(est)  # print out the classification

        # Send to the dashboard
        classify_activity = [{'variable': 'activity', 'unit': 'movement', 'value': est}]
        tagio_data_a_avgs = [{'variable': 'a_avg' + str(i), 'unit': 'm/s^2', 'value': str(round(avg, 2))} for i, avg in
                             enumerate(means[:3])]
        tagio_data_a_stdevs = [{'variable': 'a_stdev' + str(i), 'unit': 'm/s^2', 'value': str(round(stdev, 2))} for
                               i, stdev in enumerate([*stdevs[:3], *[stdevs[6]]])]
        tagio_data_g_avgs = [{'variable': 'g_avg' + str(0), 'unit': 'dps', 'value': str(round(means[7], 2))}]
        tagio_data_g_stdevs = [{'variable': 'g_stdev' + str(i), 'unit': 'dps', 'value': str(round(stdev, 2))} for
                               i, stdev in enumerate([*stdevs[3:6], *[stdevs[7]]])]
        tagio_data = [*classify_activity, *tagio_data_a_avgs, *tagio_data_a_stdevs, *tagio_data_g_avgs,
                      *tagio_data_g_stdevs]
        dash.send_data(tagio_data)

        frame = []


def main(args):
    ser = serial.Serial(args.tty, baudrate=115200, timeout=3.0)

    f = None
    if args.out_file:
        with open(args.out_file, args.out_mode) as f:
            loop(args.prompt, f, ser)
    else:
        loop(args.prompt, None, ser)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', action='store', dest='tty', required=False, default="/dev/ttyACM0")
    parser.add_argument('-p', action='store', dest='prompt', required=False, default="CSSE4011:~$")
    parser.add_argument('-o', action='store', dest='out_file', required=False)
    parser.add_argument('-om', action='store', dest='out_mode', required=False, default="a")
    args = parser.parse_args()
    main(args)
