import serial
import time

class ThroughputTest:

    def __init__(self):
        self.port = "COM4"
        self.baud = 9600
        self.byte = serial.EIGHTBITS
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.timeout = 1  # sec
        ser = serial.Serial(self.port, self.baud, self.byte, self.parity, self.stopbits)
        self.ser = ser
        self.timestamp_diff = 0
        self.starttime = 0
        self.endtime = 0
        self.output_folder = "results"
        self.dataset_type = "10X"
        self.attenuation = "95db"
        self.iteration = "2"
        self.output_filename = (self.output_folder + "/" + self.dataset_type + "/" + self.attenuation + "/" +
                                self.attenuation + "-" + self.iteration + ".csv")
        self.readData()
        # self.test()

    def test(self):
        csv_file = open("testing.csv", "w+")
        csv_file.write("TESTING if can create file \n")
        csv_file.close()
        csv_file = open("testing.csv", "a", newline='')
        csv_file.write("TESTING if can append to file")
        csv_file.close()

    def readData(self):
        if not self.ser.isOpen():
            self.ser.open()
        else:
            timelosttotal = 0
            timelost_array = []
            csv_file = open(self.output_filename, "w+")
            csv_file.write(self.port + " is open: " + str(self.ser.isOpen()) + "\n")
            csv_file.close()
            notFirstData = False
            while True:
                received_data = self.ser.readline().decode('utf-8')
                startwrite = time.time()
                csv_file = open(self.output_filename, "a", newline='')
                if received_data and not notFirstData:  # if it's first data
                    notFirstData = True                 #update flag
                    self.starttime = time.time()
                    csv_file.write("[start time]: " + str(self.starttime) + "\n")
                elif received_data.startswith("999") and notFirstData:
                    isFirstData = False
                    csv_file.write(received_data)
                    self.endtime = time.time()
                    csv_file.write("[end time]: " + str(self.endtime) + "\n")
                    csv_file.close()
                    endwrite = time.time()
                    timelost_array.append(endwrite - startwrite)
                    print(timelost_array)
                    print(sum(timelost_array))
                    timelosttotal = timelosttotal + endwrite - startwrite
                    self.timestamp_diff = self.endtime - self.starttime - timelosttotal
                    csv_file = open(self.output_filename, "a", newline='')
                    csv_file.write("[Total effective time]: " + str(self.timestamp_diff))
                    return
                csv_file.write(received_data)
                csv_file.close()
                endwrite = time.time()
                timelost_array.append(endwrite - startwrite)
                timelosttotal = timelosttotal + endwrite - startwrite


def main():
    test = ThroughputTest()


if __name__ == "__main__":
    main()
