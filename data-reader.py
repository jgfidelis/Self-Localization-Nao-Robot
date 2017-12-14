import csv
import math

import numpy as np

# file_list = ['results/90-100-1graus.csv', 'results/90-100-2graus.csv', 'results/90-100-3graus.csv']
# file_list = ['results/90-500-comPosicaoDitagraus.csv']

file_list = []
delta_x_run = []
delta_y_run = []
delta_ang_run = []

posFile = open("tempFile.txt", 'w')
for i in range(10):
    file_list.append("results/90-100-"+str(i+1)+ "graus.csv")
    delta_x_run.append([])
    delta_y_run.append([])
    delta_ang_run.append([])
delta_x = []
# delta_x_run = [[], [], []]
delta_y = []
# delta_y_run = [[], [], []]
delta_ang = []
# delta_ang_run = [[], [], []]
count = 0
run =0
for file in file_list:
    with open(file, 'rb') as csvfile:
        run += 1
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        spamreader.next()
        for row in spamreader:
            delta_x.append(math.fabs(float(row[1]) - float(row[4])))
            delta_x_run[run-1].append(math.fabs(float(row[1]) - float(row[4])))
            delta_y.append(math.fabs(float(row[2]) - float(row[5])))
            delta_y_run[run-1].append(math.fabs(float(row[2]) - float(row[5])))
            delta_angt = math.fabs(float(row[3]) - float(row[6]))
            if (delta_angt > 180):
                delta_angt = 360 - delta_angt
            delta_ang.append(delta_angt)
            delta_ang_run[run-1].append(delta_angt)


print "avgDeltaX: " + str(np.mean(delta_x))
print "avgDeltaY: " + str(np.mean(delta_y))
print "avgDeltaAng: " + str(np.mean(delta_ang))
print "StdDevX: " + str(np.std(delta_x))
print "StdDevY: " + str(np.std(delta_y))
print "StdDevAng: " + str(np.std(delta_ang))


erro_medio_x = []
erro_medio_y = []
erro_medio_ang = []
for i in range(len(file_list)):
    print "Run " + str(i+1)
    print "File: " + str(file_list[i])
    posFile.write("%d & %.6f & %.6f & %.6f \\\\\n\\hline\n" % (i+1, np.mean(delta_x_run[i]), np.mean(delta_y_run[i]), np.mean(delta_ang_run[i])))
    erro_medio_x.append(np.mean(delta_x_run[i]))
    erro_medio_y.append(np.mean(delta_y_run[i]))
    erro_medio_ang.append(np.mean(delta_ang_run[i]))
    # print("avgDeltaX: %.6f" % np.mean(delta_x_run[i]))
    # print("avgDeltaY: %.6f" % np.mean(delta_y_run[i]))
    # print("avgDeltaAng: %.6f" % np.mean(delta_ang_run[i]))
    # print "StdDevX: " + str(np.std(delta_x_run[i]))
    # print "StdDevY: " + str(np.std(delta_y_run[i]))
    # print "StdDevAng: " + str(np.std(delta_ang_run[i]))

posFile.write("%.6f & %.6f & %.6f \\\\\n\\hline\n" % (np.mean(erro_medio_x), np.mean(erro_medio_y), np.mean(erro_medio_ang)))