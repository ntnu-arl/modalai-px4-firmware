#reported power is 0-100, but in ESC firmware it is 0-1000
motor_voltages = [pwms[i]*10.0/999.0*(voltages[i]*1000) for i in range(len(pwms))]
#print motor_voltages

# linear fit or quadratic fit
ply = np.polyfit(rpms, motor_voltages, 2)
motor_voltages_fit = np.polyval(ply, rpms)

# print corresponding params
print('Quadratic fit: motor_voltage = a2*rpm_desired^2 + a1*rpm_desired + a0')
print('    a0 = ' + str(ply[2]))
print('    a1 = ' + str(ply[1]))
print('    a2 = ' + str(ply[0]))
print('ESC Params (after scaling):')
print('    pwm_vs_rpm_curve_a0 = ' + str(ply[2]))
print('    pwm_vs_rpm_curve_a1 = ' + str(ply[1]))
print('    pwm_vs_rpm_curve_a2 = ' + str(ply[0]))

# plot results if possible
try:
  import matplotlib.pyplot as plt
except:
  print('WARNING: In order to plot the results, install the Python "matplotlib" module')
  sys.exit(0)

# plot the results
plt.plot(rpms, motor_voltages, 'bo')
plt.plot(rpms, motor_voltages_fit)
plt.xlabel('Measured RPM')
plt.ylabel('Commanded Motor Voltage (mV)')
plt.title('Motor Voltage vs. RPM Test')
plt.legend(['Data', 'Fit'], loc=2)
plt.ylim([0, np.max(motor_voltages)+1000])
#plt.show()

plt.figure()
plt.plot(pwms, rpms, 'bo')
plt.xlabel('Commanded PWM (x/100)')
plt.ylabel('Measured RPM')
plt.title('RPM vs. PWM Test')
plt.show()
