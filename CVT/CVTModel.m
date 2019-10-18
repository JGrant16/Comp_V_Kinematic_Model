clear
clc

%{ 
Goal of this model is to determine how adding a Continous Variable
Transmission (CVT) would benefit the UMD Loop 2019 pod by allowing for less
weight removal and lower power
%}

%Conversions
lbToN = 4.45;
lbsToKg = 0.453592;
inToM = 0.0254;
mpsToMph = 2.23694;
global radToRPM;
radToRPM = 9.5492965964254;

% Flags
mass_optimize_flag = true;
emergency_brake_flag = false;
global power_boost_flag;
power_boost_flag = false;

% Battery Data
num_batteries = 30;
battery_voltage_at_max_current = 16.58; %V
total_voltage = battery_voltage_at_max_current*num_batteries;
max_battery_power = 115000; %kW

%Parameters
global max_acceleration;
max_acceleration = 9.8*2.5; %m/s^2
goal_speed = 310/mpsToMph;
coef_rolling_resistance = 0.1*inToM; %m
mass_tread = 4.96473952078*lbToN/9.8; %Kg
c = 2300; %J*(kg*K)
wheel_height = 4*inToM; %m
deformation = 0.0178*inToM; %m
Preload = 1748.03; %N
init_temp = 90;     %F
belt_loss = 0.95;    %percentage loss
decel = 9.8*2.5;    %m/sec^2
%gr_1 = 0.44;           %gear ratio
wheel_rad = 4*inToM;    %Outer wheel radius, m
tread_rad = 3.5*inToM;  %Radius to inner tread, m
pod_mass = 330*lbsToKg;   %kg
max_dist = 1220;    %m
sheer_modulus = 15*10^6; %Pa
psi = (wheel_rad-0.5*inToM)/(wheel_rad+0.5*inToM);
m = 1.00068 - 0.82768*psi + 24.6*psi^2 - 126.41*psi^3 + 291.53*psi^4-307.32*psi^5+122.85*psi^6;
force_friction_prelim = 3.4*3/16*Preload^(4/3)/(wheel_rad)*m*(3/2*wheel_rad*0.5*inToM/(wheel_rad+0.5*inToM)*(1-0.5^2)/sheer_modulus)^(1/3);

%Initial Values
power_motor = 0;
init_position = 5; %m
init_speed = 0.01;  %m/sec^2
init_time = 0.0;   %sec
step_size = 0.001; %sec
mass_step = 2; %lbs
gr_step = 0.1;
RPM_step = 1;
v_max_ideal = 0;

%% Ideal Profile
motor_RPM = 0;
%determining the maximum motor RPM that the current battery output power
%will give
while cvt_power(motor_RPM) <= max_battery_power
   motor_RPM = motor_RPM + RPM_step;
end
%This value will determine when CVT come into effect
max_motor_power = cvt_power(motor_RPM);

%acceleration loop
while true
    count = 1;
    power_motor = 0;
    %Data arrays
    gr_array = [];
    power_array = [];
    position_array = [];
    velocity_array = [];
    acceleration_array = [];
    time_array = [];
    %loop to find the initial gear ratio of the CVT
    init_gr = find_init_gr(pod_mass,wheel_rad,init_speed);
    gr_array(count,1) = init_gr;
    time_array(count,1) = init_time;
    velocity_array(count,1) = init_speed;
    position_array(count,1) = init_position;
    while true
        curr_vel = velocity_array(count,1);
        %determine if we are still gaining speed to max RPM or already
        %withing CVT operating range
        if (power_motor < max_motor_power)
            power_motor = ideal_power(curr_vel,gr_array(count,1),wheel_rad);
            gr_array(count+1,1) = init_gr;
        else
            power_motor = max_motor_power;
            %determines the current GR needed to maintain maximum power
            gr_array(count+1,1) = get_gr(motor_RPM,curr_vel, wheel_rad);
        end
        
        power_array(count,1) = power_motor;
        acceleration_array(count,1) = power_motor/(pod_mass*curr_vel);
        %Forward Euler Method
        velocity_array(count + 1,1) = curr_vel + step_size*acceleration_array(count,1);
        position_array(count + 1,1) = position_array(count,1) + step_size*curr_vel;
        time_array(count + 1,1) = time_array(count,1) + step_size;
        %Determine braking distance
        if((position_array(count + 1,1)+(velocity_array(count + 1,1))^2/(decel*2)) > max_dist)
            v_max_ideal = velocity_array(count + 1,1);
            p_max_ideal = power_motor;
            rpm_max_ideal = motor_RPM;
            time_max_ideal = time_array(count + 1,1);
            count = count + 1;
            break;
        end
        count = count + 1;  
    end
    
    %Braking loop
    while true
    acceleration_array(count,1) = -1*decel;
    %at zero velocity
    if(velocity_array(count,1) + step_size*acceleration_array(count,1) <= 0)
        break
    end
    time_array(count + 1,1) = time_array(count,1) + step_size;
    %Forward Euler
    velocity_array(count + 1,1) = velocity_array(count,1) + step_size*acceleration_array(count,1);
    position_array(count + 1,1) = position_array(count,1) + step_size*velocity_array(count,1);
    power_array(count + 1,1) = 0;
    gr_array(count+1,1) = 1;
    count = count + 1;
    end
    
    %only used if want to find smallest mass needed to reach goal speed
    if(mass_optimize_flag == true && v_max_ideal < goal_speed)
        pod_mass = pod_mass - mass_step*lbsToKg;
    else
        break;
    end
end

%results of model
if(mass_optimize_flag == true)
    disp(['Initial Gear Ratio of ' num2str(init_gr) ' at a mass of ' num2str(pod_mass/lbsToKg) 'lbs will give ' num2str(v_max_ideal*mpsToMph) 'mph at a peak acceleration of ' num2str(max_acceleration) 'm/s^2'])
end

%store final values in struct
data(1).time = time_array;
data(1).position = position_array;
data(1).velocity = velocity_array;
data(1).acceleration = acceleration_array;
data(1).power = power_array;
data(1).gr_ratio = gr_array;

%% Loss Profile
%{
outer = 2;
v_max_loss = [];
p_max_loss = [];
rpm_max_loss = [];
while outer < 5
    count = 1;
    power_loss_array = [];
    power_array = [];
    position_array = [];
    velocity_array = [];
    acceleration_array = [];
    time_array = [];
    wheel_temp_array = [];
    time_array(count,1) = init_time;
    velocity_array(count,1) = init_speed;
    position_array(count,1) = init_position;
    wheel_temp_array(count,1) = init_temp;
    heat_gen_sum = 0;
    while true
        curr_vel = velocity_array(count,1);
        power_motor = ideal_power(curr_vel,gr,wheel_rad);
        power_array(count,1) = power_motor;
        Power = power_motor*belt_loss;
        curr_temp = (heat_gen_sum/(c*mass_tread))*9/5 + init_temp;
        
        if (outer == 2) 
            power_loss = Hysteresis_Model(curr_temp, Preload, deformation, wheel_rad, wheel_height, tread_rad, curr_vel);
        elseif (outer == 3)
            power_loss = Coefficient_Model(coef_rolling_resistance,Preload,wheel_rad,curr_vel);
        else
            power_loss = Theory_Model(curr_temp, force_friction_prelim, curr_vel);
        end
        
        heat_gen_sum = heat_gen_sum + power_loss*step_size;
        Power = Power - power_loss;
        power_loss_array(count,1) = power_loss;
        
        acceleration_array(count,1) = Power/(pod_mass*curr_vel);
        velocity_array(count + 1,1) = curr_vel + step_size*acceleration_array(count,1);
        position_array(count + 1,1) = position_array(count,1) + step_size*curr_vel;
        time_array(count + 1,1) = time_array(count,1) + step_size;
        wheel_temp_array(count + 1,1) = curr_temp;

        if((position_array(count + 1,1)+(velocity_array(count + 1,1))^2/(decel*2)) > max_dist)
            v_max_loss(outer-1,1) = velocity_array(count + 1,1);
            p_max_loss(outer-1,1) = power_motor;
            angular_v_wheel = curr_vel/wheel_rad;
            angular_v_motor = angular_v_wheel*gr;
            motor_rpm = angular_v_motor*radToRPM;
            rpm_max_loss(outer-1,1) = motor_rpm;
            time_max_loss = time_array(count + 1,1);
            count = count + 1;
            break;
        end
        count = count + 1; 
    end

    while true
        curr_vel = velocity_array(count,1);
        curr_temp = (heat_gen_sum/(c*mass_tread))*9/5 + init_temp;
        
        if (outer == 2) 
            power_loss = Hysteresis_Model(curr_temp, Preload, deformation, wheel_rad, wheel_height, tread_rad, curr_vel);
        elseif (outer == 3)
            power_loss = Coefficient_Model(coef_rolling_resistance,Preload,wheel_rad,curr_vel);
        else
            power_loss = Theory_Model(curr_temp, force_friction_prelim, curr_vel);
        end

        heat_gen_sum = heat_gen_sum + power_loss*step_size;
        power_loss_array(count,1) = power_loss;
        
        acceleration_array(count,1) = -1*decel;
        if(velocity_array(count,1) + step_size*acceleration_array(count,1) <= 0)
            break
        end
        time_array(count + 1,1) = time_array(count,1) + step_size;
        velocity_array(count + 1,1) = curr_vel + step_size*acceleration_array(count,1);
        position_array(count + 1,1) = position_array(count,1) + step_size*curr_vel;
        power_array(count + 1,1) = 0;
        wheel_temp_array(count + 1,1) = curr_temp;
        count = count + 1;
    end
    data(outer).position = position_array;
    data(outer).time = time_array;
    data(outer).velocity = velocity_array;
    data(outer).acceleration = acceleration_array;
    data(outer).power = power_array;
    data(outer).power_loss = power_loss_array;
    data(outer).temp = wheel_temp_array;
    outer = outer + 1;
end
%}
%% Plotting
figure(1)
subplot(1,3,1)
plot(data(1).time,data(1).position);
title('Position')
xlabel('Time (sec)')
ylabel('Position (m)')
%{
hold on
plot(data(2).time,data(2).position);
hold on
plot(data(3).time,data(3).position);
hold on
plot(data(4).time,data(4).position);
legend('Ideal','Loss (Hysteresis)','Loss (Coefficient)','Loss (Theory)','Location','southwest')
hold off
%}

subplot(1,3,2)
plot(data(1).time,data(1).velocity*mpsToMph);
title('Velocity')
ylabel('Velocity (mph)')
xlabel('Time (sec)')
%{
hold on
plot(data(2).time,data(2).velocity*mpsToMph);
hold on
plot(data(3).time,data(3).velocity*mpsToMph);
hold on
plot(data(4).time,data(4).velocity*mpsToMph);
%}
legend(['Ideal: ' num2str(v_max_ideal*mpsToMph) ' mph'])%,['Loss (Hysteresis): ' num2str(v_max_loss(1,1)*mpsToMph) ' mph'],['Loss (Coefficient): ' num2str(v_max_loss(2,1)*mpsToMph) ' mph'],['Loss (Theory): ' num2str(v_max_loss(3,1)*mpsToMph) ' mph'],'Location','southwest')
%hold off


subplot(1,3,3)
plot(data(1).time,data(1).acceleration);
title('Acceleration')
xlabel('Time (sec)')
ylabel('Acceleration (m/sec^2)')
%{
hold on
plot(data(2).time,data(2).acceleration);
hold on
plot(data(3).time,data(3).acceleration);
hold on
plot(data(4).time,data(4).acceleration);
legend('Ideal','Loss (Hysteresis)','Loss (Coefficient)','Loss (Theory)','Location','southwest')
hold off
%}

figure(2)
subplot(1,2,1)
plot(data(1).time,data(1).power);
title('Motor Power (label of max motor power)')
xlabel('Time (sec)')
ylabel('Watts')

subplot(1,2,2)
plot(data(1).time,data(1).gr_ratio);
title('Gear Ratio versus Time')
xlabel('Time (sec)')
ylabel('GR')
%{
hold on
plot(data(2).time,data(2).power);
hold on
plot(data(3).time,data(3).power);
hold on
plot(data(4).time,data(4).power);
legend(['Ideal: ' num2str(p_max_ideal/1000) ' kW, ' num2str(rpm_max_ideal) ' rpm'],['Loss (Hysteresis): ' num2str(p_max_loss(1,1)/1000) ' kW, ' num2str(rpm_max_loss(1,1)) ' rpm'],['Loss (Coefficient): ' num2str(p_max_loss(2,1)/1000) ' kW, ' num2str(rpm_max_loss(2,1)) ' rpm'],['Loss (Theory): ' num2str(p_max_loss(3,1)/1000) ' kW, ' num2str(rpm_max_loss(3,1)) ' rpm'],'Location','southwest')
hold off
%}

%{
subplot(1,2,2)
plot(data(2).time,data(2).power_loss);
title('Power Loss')
xlabel('Time (sec)')
ylabel('Watts')
hold on
plot(data(3).time,data(3).power_loss);
hold on
plot(data(4).time,data(4).power_loss);
legend('Loss (Hysteresis)','Loss (Coefficient)','Loss (Theory)','Location','northwest')
hold off
%}

%{
figure(3)
plot(data(2).velocity*mpsToMph,data(2).temp);
title('Drive Wheel Temperature')
xlabel('Velocity (mph)')
ylabel('Temperature (F)')
hold on
plot(data(3).velocity*mpsToMph,data(3).temp);
hold on
plot(data(4).velocity*mpsToMph,data(4).temp);
legend('Loss (Hysteresis)','Loss (Coefficient)','Loss (Theory)','Location','northwest')
hold off
%}
%% Functions
function power_motor = ideal_power(v,gr,radius) 
global radToRPM power_boost_flag
%c0 = -186.2; c1 = 269; c2 = -0.6867; c3 = 0.004759; c4 = -0.00001214; c5 = 0.00000000957;
%Power series coefficients for torque vs RPM for EMRAX 228 motor
c0= 239.8790812; c1= 0.00036227; c2 = 5.02763E-07; c3=-4.9019E-10; c4=3.7E-14; 
angular_v_wheel = v/radius;
angular_v_motor = angular_v_wheel*gr;
motor_rpm = angular_v_motor*radToRPM;
torque = c0+c1*motor_rpm+c2*(motor_rpm)^2+c3*(motor_rpm)^3+c4*(motor_rpm)^4;
power_motor = torque*angular_v_motor;
%ensure the RPm of the motor does not get too high
if (motor_rpm >= 6500)
    power_motor = -1; %emmergency brake
end
if (power_boost_flag == true)
    power_motor = power_motor*1.1;
end
end

%{
function power_loss = Hysteresis_Model(curr_temp, Preload, deformation, wheel_rad, wheel_height, tread_rad, curr_vel)
    frequency = curr_vel/(2*pi*wheel_rad);
    hr = loss_mod_value(curr_temp)/sqrt(loss_mod_value(curr_temp)^2+storage_mod_value(curr_temp)^2);
    Total_Strain_energy_density = 1/2*Preload*deformation/(pi*wheel_rad^2*wheel_height-pi*tread_rad^2*wheel_height);
    Loss_Part = Total_Strain_energy_density*hr;
    power_loss = Loss_Part*frequency;
    disp(pi*wheel_rad^2*wheel_height-pi*tread_rad^2*wheel_height)
end

function power_loss = Coefficient_Model(coef_rolling_resistance,Preload,wheel_rad,curr_vel)
    rolling_force = coef_rolling_resistance*Preload/wheel_rad;
    power_loss = rolling_force*curr_vel;
end

function power_loss = Theory_Model(curr_temp, force_friction_prelim, curr_vel)
    hr = loss_mod_value(curr_temp)/sqrt(loss_mod_value(curr_temp)^2+storage_mod_value(curr_temp)^2);  
    rolling_force = force_friction_prelim*hr;
    power_loss = rolling_force*curr_vel;
end
%}
function power_motor = cvt_power(motor_rpm)
    global radToRPM 
    %power series to estimate torque for EMRAX 228 Motor versus RPM
    c0= 239.8790812; c1= 0.00036227; c2 = 5.02763E-07; c3=-4.9019E-10; c4=3.7E-14; %torque vs. rpm
    torque = c0+c1*motor_rpm+c2*(motor_rpm)^2+c3*(motor_rpm)^3+c4*(motor_rpm)^4;
    w_motor = motor_rpm/radToRPM;
    power_motor = w_motor*torque;
end

function gr = get_gr(max_motor_rpm,v, radius)
    global radToRPM 
    angular_v_wheel = v/radius;
    angular_v_motor = max_motor_rpm/radToRPM;
    gr = angular_v_motor/angular_v_wheel;
end

%function to find the initial gear ratio of CVT 
function init_gr = find_init_gr(pod_mass,wheel_rad,init_speed)
global max_acceleration    %increasing the gr too much will put too much acceleration force on pod
    init_gr = 50;
    while true
        motor_power = ideal_power(init_speed,init_gr,wheel_rad);
        a = motor_power/(pod_mass*init_speed);
        %loop to find what starting GR the CVT will have that is below max
        %acceleration
        if a > max_acceleration
            init_gr = init_gr - 0.01;
        else
           break
        end  
    end
end