import serial
import re
import argparse
import io
import csv
import numpy as np
from time import sleep
from csv import reader
from math import sqrt

# Load a CSV dataset file
def load_csv(filename):
    dataset = list()
    with open(filename, 'r', encoding='utf-8-sig') as file:
        csv_reader = reader(file)
        for row in csv_reader:
            if not row:
                continue
            dataset.append(row)
    return dataset

# Convert string column to float
def str_column_to_float(dataset, column):
    for row in dataset:
        row[column] = float(row[column].strip())

# Convert string column to integer
def str_column_to_int(dataset, column):
    class_values = [row[column] for row in dataset]
    unique = set(class_values)
    lookup = dict()
    for i, value in enumerate(unique):
        lookup[value] = i
    for row in dataset:
        row[column] = lookup[row[column]]
    return lookup

# Find the min and max values for each column
def dataset_minmax(dataset):
    minmax = list()
    for i in range(len(dataset[0])):
        col_values = [row[i] for row in dataset]
        value_min = min(col_values)
        value_max = max(col_values)
        minmax.append([value_min, value_max])
    return minmax

# Rescale dataset columns to the range 0-1
def normalise_dataset(dataset, minmax):
    for row in dataset:
        for i in range(len(row) - 1):
            row[i] = round((row[i] - minmax[i][0]) / (minmax[i][1] - minmax[i][0]),4)

# Rescale dataset columns to the range 0-1
def normalise_reading(reading, minmax):
    output = list()
    for i in range(len(reading)):
        output.append((reading[i] - minmax[i][0]) / (minmax[i][1] - minmax[i][0]))
    return output

# Calculate the Euclidean distance between two vectors
def euclidean_distance(row1, row2):
    distance = 0.0
    for i in range(len(row1)-1):
        distance += (row1[i] - row2[i])**2
    return sqrt(distance)

# Locate the most similar neighbours
def get_neighbours(train, test_row, num_neighbours):
    distances = list()
    for train_row in train:
        dist = euclidean_distance(test_row, train_row)
        distances.append((train_row, round(dist,3)))
    distances.sort(key=lambda tup: tup[1])
    neighbours = list()
    for i in range(num_neighbours):
        neighbours.append(distances[i][0])
    return neighbours

# Make a prediction with neighbours
def predict_classification(train, test_row, num_neighbours, minmax):
    test_row = normalise_reading(test_row, minmax)
    neighbours = get_neighbours(train, test_row, num_neighbours)
    output_values = [row[-1] for row in neighbours]
    prediction = max(set(output_values), key=output_values.count)
    return prediction

ansi_escape = re.compile(r'(?:\x1B[@-Z\\-_]|[\x80-\x9A\x9C-\x9F]|(?:\x1B\[|\x9B)[0-?]*[ -/]*[@-~])')
base_sens = [2.0, 250.0]

def loop(prompt, f: io.TextIOWrapper, ser: serial.Serial):
    if(f):
        f.write('ax_s,ay_s,az_s,gx_s,gy_s,gz_s,A_s,G_s\n')

    filename = '../../data/knn_dataset.csv'
    dataset = load_csv(filename)
    for i in range(len(dataset[0])-1):
        str_column_to_float(dataset, i)
    # convert class column to integers
    lookup = str_column_to_int(dataset, len(dataset[0])-1)
    lookup = {v:k for k,v in lookup.items()}
    minmax = dataset_minmax([row[0:12] for row in dataset])
    normalise_dataset(dataset, minmax)
    # define model parameter
    num_neighbours = 1;
    frame = [];

    while True:

        serialString=ser.readline()
        sw = serialString.decode('ascii')

        # remove garbage
        sw_stripped = ansi_escape.sub('', sw).strip('\n\r').replace(args.prompt,'')

        # if empty or not valid
        if not sw_stripped or sw_stripped == '' or sw_stripped[0] != '%':
            continue

        reading = list(csv.reader([sw_stripped[1:]]))[0]

        s_ag = [float(scale)*(128/base_sens[i]) for i,scale in enumerate(reading[:2])]
        a_xyz_r = [float(axis) for axis in reading[2:5]]
        g_xyz_r = [float(axis) for axis in reading[5:8]]
        q_wxyz = [float(axis) for axis in reading[-4:]]

        a_xyz = [axis/s_ag[0] for axis in a_xyz_r]
        g_xyz = [axis/s_ag[1] for axis in g_xyz_r]

        data = [*a_xyz, *g_xyz]
        data.append(np.linalg.norm(a_xyz))
        data.append(np.linalg.norm(g_xyz))
        #print(data)
        frame.append(list(data))

        if(f):
            f.write(','.join([str(round(i,4)) for i in data]) + '\n')

        if len(frame) != 80:
            continue
        frame = np.array(frame).transpose().tolist()

        means = [round(np.abs(np.mean(sensor)),4) for sensor in frame]
        stdevs = [round(np.std(sensor),4) for sensor in frame]

        out_data = [*means[:3], *[means[-1]], *stdevs]
        label = predict_classification(dataset, out_data, num_neighbours, minmax)
        print(lookup[label])
        frame = []

def main(args):
    ser = serial.Serial(args.tty, baudrate=115200, timeout=3.0)
    
    f = None
    if args.out_file:
        with open(args.out_file, args.out_mode) as f:
            loop(args.prompt, f, ser)
    else:
        loop(args.prompt, None, ser)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', action='store', dest='tty', required=False, default="/dev/ttyACM0")
    parser.add_argument('-p', action='store', dest='prompt', required=False, default="CSSE4011:~$")
    parser.add_argument('-o', action='store', dest='out_file', required=False)
    parser.add_argument('-om', action='store', dest='out_mode', required=False, default="a")
    args = parser.parse_args()
    main(args)
