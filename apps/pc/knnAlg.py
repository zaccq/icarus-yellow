# Make Predictions with k-nearest neighbours
import csv
from math import sqrt

class Classifier:
    def __init__(self, file_name, num_neighbours):
        self.dataset = self.load_csv(file_name)
        for i in range(len(self.dataset[0]) - 1):
            self.str_column_to_float(self.dataset, i)
        # convert class column to integers
        self.lookup = self.str_column_to_int(self.dataset, len(self.dataset[0]) - 1)
        self.lookup = {v:k for k,v in self.lookup.items()}
        self.minmax = self.dataset_minmax()
        self.normalise_dataset()
        # define model parameter
        self.num_neighbours = num_neighbours


    # Load a CSV dataset file
    def load_csv(self, filename):
        dataset = list()
        with open(filename, 'r', encoding='utf-8-sig') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                if not row:
                    continue
                dataset.append(row)
        return dataset


    # Convert string column to float in place
    def str_column_to_float(self, dataset, column):
        for row in dataset:
            row[column] = float(row[column].strip())


    # Convert string column to integer in place
    def str_column_to_int(self, dataset, column):
        class_values = [row[column] for row in dataset]
        unique = set(class_values)
        lookup = dict()
        for i, value in enumerate(unique):
            lookup[value] = i
            print('[%s] => %d' % (value, i))
        for row in dataset:
            row[column] = lookup[row[column]]
        return lookup


    # Find the min and max values for each column
    def dataset_minmax(self):
        local_dataset = [row[0:12] for row in self.dataset]
        minmax = list()
        for i in range(len(local_dataset[0])):
            col_values = [row[i] for row in local_dataset]
            value_min = min(col_values)
            value_max = max(col_values)
            minmax.append([value_min, value_max])
        return minmax


    # Rescale dataset columns to the range 0-1
    def normalise_dataset(self):
        for row in self.dataset:
            for i in range(len(row) - 1):
                row[i] = (row[i] - self.minmax[i][0]) / (self.minmax[i][1] - self.minmax[i][0])

    # Rescale dataset columns to the range 0-1
    def normalise_reading(self, reading):
        output = list()
        for i in range(len(reading)):
            output.append((reading[i] - self.minmax[i][0]) / (self.minmax[i][1] - self.minmax[i][0]))
        return output

    # Calculate the Euclidean distance between two vectors
    def euclidean_distance(self, row1, row2):
        distance = 0.0
        for i in range(len(row1) - 1):
            distance += (row1[i] - row2[i]) ** 2
        return sqrt(distance)


    # Locate the most similar neighbours
    def get_neighbours(self, train, test_row, num_neighbours):
        distances = list()
        for train_row in train:
            dist = self.euclidean_distance(test_row, train_row)
            distances.append((train_row, dist))
        distances.sort(key=lambda tup: tup[1])
        neighbours = list()
        for i in range(num_neighbours):
            neighbours.append(distances[i][0])
        return neighbours


    # Make a prediction with neighbours
    def predict_classification(self, test_row):
        test_row = self.normalise_reading(test_row)
        neighbours = self.get_neighbours(self.dataset, test_row, self.num_neighbours)
        output_values = [row[-1] for row in neighbours]
        prediction = max(set(output_values), key=output_values.count)
        return self.lookup[prediction]

    #def compute_knn(file_name, row):
        #dataset = load_csv(file_name)
        #for i in range(len(dataset[0]) - 1):
            #str_column_to_float(dataset, i)
        ## convert class column to integers
        #str_column_to_int(dataset, len(dataset[0]) - 1)
        ## define model parameter
        #num_neighbors = 1
        ## predict the label
        #label = predict_classification(dataset, row, num_neighbors)
        #print('Data= %s, Predicted: %s' % (row, label))
