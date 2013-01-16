# commands to grace to plot IMU telemetry data
# run with xmgrace -batch grace.cmd -nxy imudata.dat
# ARRANGE(nrows, ncols, offset, hgap, vgap) 
arrange ( 2, 2, .20, .3, .30)
focus g0
autoscale
yaxis label "gyro"
xaxis label "time (ms)"
# change colors for gyro/accel
s4 line color 1
s4 line linewidth 2.0
s4 line linestyle 4
s4 legend "wx"
s5 line color 3
s5 line linewidth 2.0
s5 legend "wy"
s6 line color 4
s6 line linewidth 2.0
s6 legend "wz"
legend 0.5, 0.8
legend box false
legend char size 0.5
#drop wz avg and accelerometer
kill g0.s7
kill g0.s8
kill g0.s9
kill g0.s10
# pwm values
move g0.s2 to g2.s0
move g0.s3 to g2.s1
focus g2
autoscale
xaxis label "time (ms)"
yaxis label "motor PWM cmd"
# set line color and thickness
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "right"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "left"
    legend 0.48, 0.46
legend char size 0.5
# 
# move motor position to graph 1
move g0.s0 to g1.s0
move g0.s1 to g1.s1
focus g1
autoscale
xaxis label "time (ms)"
yaxis label "leg phase (rad)"
#world ymax 12.6
# set line color and thickness
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "right"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "left"
    legend 0.73, 0.8
legend char size 0.5
# back emf and VBat values
move g0.s11 to g3.s0
move g0.s12 to g3.s1
# Vbat:
#move g0.s13 to g3.s2
focus g3
autoscale
xaxis label "time (ms)"
yaxis  label "back emf"
# set line color and thickness
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "right"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "left"
    #uncomment s2 to show VBat 	
    #s2 line color 2
    #s2 line linewidth 2.0
    #s2 legend "VBat"
    legend 0.98, 0.46
legend char size 0.5
#legend
    altxaxis  off
    altyaxis  off
    legend on
    legend loctype view

    legend box color 1
    legend box pattern 1
    legend box linewidth 1.0
    legend box linestyle 1
    legend box fill color 0
    legend box fill pattern 1
    legend font 0
    legend color 1
    legend length 4
    legend vgap 1
    legend hgap 1
    legend invert false
legend char size 0.5
    frame type 0
    frame linestyle 1
    frame linewidth 1.0
    frame color 1
    frame pattern 1
    frame background color 0
    frame background pattern 0


