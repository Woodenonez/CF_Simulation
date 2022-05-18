import pandas as pd
import matplotlib.pyplot as plt

dataFrame = pd.read_csv("simulation.csv")

t = dataFrame.loc[:,"time"].values

roll_actual = dataFrame.loc[:,"actual_roll"].values
roll_est    = dataFrame.loc[:,"estimated_roll"].values
roll_ref    = dataFrame.loc[:,"reference_roll"].values

pitch_actual = dataFrame.loc[:,"actual_pitch"].values
pitch_est    = dataFrame.loc[:,"estimated_pitch"].values
pitch_ref    = dataFrame.loc[:,"reference_pitch"].values

yaw_actual = dataFrame.loc[:,"actual_yaw"].values
yaw_est    = dataFrame.loc[:,"estimated_yaw"].values
yaw_ref    = [0]*len(t)

m1 = dataFrame.loc[:,"m1"].values
m2 = dataFrame.loc[:,"m2"].values
m3 = dataFrame.loc[:,"m3"].values
m4 = dataFrame.loc[:,"m4"].values

plt.figure()
plt.subplot(311)
plt.plot(t,roll_actual,"b", t,roll_est,"bo", t,roll_ref,"b--", fillstyle="none")
plt.ylabel("degree")
plt.legend(["actual roll","estimated roll","reference roll"])

plt.subplot(312)
plt.plot(t,pitch_actual,"r", t,pitch_est,"ro", t,pitch_ref,"r--", fillstyle="none")
plt.ylabel("degree")
plt.legend(["actual pitch","estimated pitch","reference pitch"])

plt.subplot(313)
plt.plot(t,yaw_actual,"y", t,yaw_est,"yo", t,yaw_ref,"y--", fillstyle="none")
plt.ylabel("degree"), plt.xlabel("t/s")
plt.legend(["actual yaw","estimated yaw","reference yaw"])

plt.figure(),
plt.plot(t[5:],m1[5:], t[5:],m2[5:], t[5:],m3[5:], t[5:],m4[5:])
plt.xlabel("t [s]"), plt.ylabel("motor signal [%]")
plt.legend(["m1","m2","m3","m4"])

plt.show()