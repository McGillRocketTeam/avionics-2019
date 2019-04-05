import csv
# Code to read unformatted data and write to a single column csv file

# Change path
pathRead = '/Users/alessia/PycharmProjects/plotgps/adafruit.csv'
pathWrite = '/Users/alessia/PycharmProjects/plotgps/formatdata.csv'

inputfile = csv.reader(open(pathRead,'r'))
outputfile = open(pathWrite,'w')

total = []
final = []
for row in inputfile:
    if row[0] == '$GPGSV':
        total.append(row[7][:2])
        total.append(row[11][:2])
        total.append(row[15][:2])
        total.append(row[19][:2])

for element in total:
    if element != '*':
        final.append(element)

for n in final:
        outputfile.write(n)
        outputfile.write(',')
        outputfile.write('\n')
