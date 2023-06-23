import csv
class FileWriterCSV:
    """
    A class to write data to a CSV file.
    """
    def __init__(self,filename):
        """
        Initializes the FileWriterCSV object.
        @param id: The ID of the object to write data for.
        """
        self.filename = filename
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['id', 'timestamp', 'x', 'y'])

    def write_data(self, id, timestamp, x, y):
        """
        Writes a row of data to the CSV file.

        @param data: A list containing the data to write.
        """
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([id, timestamp, x, y])
    

    