# This is a sample Python script.
import knnAlg as knn
import dashboard as dash

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # dash.printInfo()
    #
    # testData = {
    #     'variable': 'hmov',
    #     'unit': 'cm',
    #     'value': 30
    # }
    # dash.send_data(testData)
    #
    # fil = {
    #     'variable': 'hmov',
    #     'query': 'last_value',
    # }
    # dash.get_data(fil)

    # we want to use knn
    file = 'knn_dataset.csv'
    row = [0.894779726, 0.366540055, 0.177406301, 1.339092879, 0.066174796, 0.063936819, 0.052950531, 1.994015052, 0.860659541, 0.206841843, 0.011416504, 1.727318415]
    knn.compute_knn(file, row)
