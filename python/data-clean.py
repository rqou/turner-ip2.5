# process telemetry file to make easier to plot
#updated for IP2.5 Jan. 2013
import csv

# angle range 16 bits 0 to 2 pi (truncated angle)
def angleEncoder(value):
    temp = (value*2*3.14159)/(2**16)
    return temp


filedata = csv.reader(open('Data/imudata.txt','rb'), delimiter=',',quotechar='"')
# discard first 8 rows
for i in range(1,7):
    r=filedata.next()
    print '#',r
#get first numeric row
r=filedata.next()
#time, LPos, RPos, DCL, DCR, GyroX, GryoY, GryoZ, GryoZAvg
# , AX, AY, AZ, LBEMF, RBEMF, VBatt SteerOut\n')
row=map(int,r)  # convert string list to numbers
t0=row[0]  # 
print (row[0]-t0)/1000,
print '%6.2f' % angleEncoder(row[1]),
print '%6.2f' % angleEncoder(row[2]),
for i in range(3,15):
    print row[i],
print ' '
for r in filedata:
    row=map(int,r)
    print (row[0]-t0)/1000,
    # change angles to 2 pi range (so can see a whole cycle better)
    print '%6.2f' % angleEncoder(row[1]),
    print '%6.2f' % angleEncoder(row[2]),
    for i in range(3,15):
        print " %d" % row[i],
    print ' '

    
 
