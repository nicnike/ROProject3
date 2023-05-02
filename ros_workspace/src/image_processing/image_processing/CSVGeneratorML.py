import csv


def appendToCSV(inputData):
    filename = "MachineLearningDataSet.csv"

    with open(filename, 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        row = inputData
        csvwriter.writerow(row)