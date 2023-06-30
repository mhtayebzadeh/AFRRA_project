% clc; clear all 

table = step7;

t_ms = table.time_ms;
t = (t_ms - t_ms(1))/1000.0;
force = table.force_loadcell + 0.2;
force_sp = table.force_sp;
theta_ankle = table.theta_ankle;
theta_ankle_sp = table.theta_ankle_sp;
I_m = table.motor_current;


plot(t , force)
hold on
plot(t, I_m)

