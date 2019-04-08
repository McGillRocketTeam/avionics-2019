import serial
import time

class CompareData:

    def __init__(self):
        self.dataset_type = "5X"
        self.output_folder = "difference/" + self.dataset_type
        self.original_file = "datasets/fakedataset_" + self.dataset_type + ".csv"
        self.new_file_folder = "results/" + self.dataset_type
        # self.attenuations = ["65db", "70db", "75db", "80db", "85db", "90db", "95db", "100db", "110db"]
        self.attenuations = ["100db"]

    def compareData(self, attenuation, test_name):
        orig = open(self.original_file, 'r')
        new = open(self.new_file_folder + "/" + attenuation + "/" + test_name + '.csv', 'r')
        bigb = set(new) - set(orig)                         # in new but not in orig
        print(bigb)                                         # To see results in console if desired
        with open(self.output_folder + "/" + attenuation + "/" + test_name + '.csv', 'w') as file_out:       # Write to output file
            for line in bigb:
                file_out.write(line)
        orig.close()
        new.close()
        file_out.close()


def main():
    diff = CompareData()

    for attenuation in diff.attenuations:
        for num in range(3):
            diff.compareData(attenuation, attenuation + "-" + str(num + 1))

if __name__ == "__main__":
    main()

