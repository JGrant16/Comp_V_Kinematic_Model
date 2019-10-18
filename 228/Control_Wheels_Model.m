clear
clc

%Conversions
lbToN = 4.45;
lbsToKg = 0.453592;
inToM = 0.0254;
mpsToMph = 2.23694;
global radToRPM;
radToRPM = 9.5492965964254;

% Flags
mass_optimize_flag = false;
emergency_brake_flag = false;
global power_boost_flag;
power_boost_flag = false;

% Battery Data
num_batteries = 30;
battery_voltage_at_max_current = 16.58; %V
total_voltage = battery_voltage_at_max_current*num_batteries;

%Parameters
num_idler_wheels = 10;
ratio = 8400/8000;
idler_ratio = 0;
coef_rolling_resistance = 0.1*inToM; %m
mass_tread = 4.96473952078*lbToN/9.8; %Kg
mass_tread_idler = 0.0684*lbToN/9.8; %Kg
c = 2300; %J*(kg*K)
wheel_height = 4*inToM; %m
wheel_height_idler = 0.625*inToM;
deformation = 0.0178*inToM; %m
deformation_idler = 0.0022*inToM;
Preload = 1748.03; %N
Preload_idler = 200*lbToN;
init_temp = 90;     %F
belt_loss = 0.95;    %percentage loss
decel = 9.8*2.5;    %m/sec^2
gr = 0.51;           %gear ratio
wheel_rad = 4*inToM;    %Outer wheel radius, m
tread_rad = 3.5*inToM;  %Radius to inner tread, m
idler_rad = 1.375*inToM;
idler_tread_rad = 1.0625*inToM;
idler_volume_tread = 2.4508493e-05;
pod_mass = 330*lbsToKg;   %kg
max_dist = 1250;    %m
sheer_modulus = 15*10^6; %Pa
psi_idler = (idler_rad-idler_tread_rad)/(idler_rad+idler_tread_rad);
psi = (wheel_rad-0.5*inToM)/(wheel_rad+0.5*inToM);
m = 1.00068 - 0.82768*psi + 24.6*psi^2 - 126.41*psi^3 + 291.53*psi^4-307.32*psi^5+122.85*psi^6;
m_idler = 1.00068 - 0.82768*psi_idler + 24.6*psi_idler^2 - 126.41*psi_idler^3 + 291.53*psi_idler^4-307.32*psi_idler^5+122.85*psi_idler^6;
force_friction_prelim = 3.4*3/16*Preload^(4/3)/(wheel_rad)*m*(3/2*wheel_rad*0.5*inToM/(wheel_rad+0.5*inToM)*(1-0.5^2)/sheer_modulus)^(1/3);
force_friction_prelim_idler = 3.4*3/16*Preload^(4/3)/(idler_rad)*m_idler*(3/2*idler_rad*idler_tread_rad/(idler_rad+idler_tread_rad)*(1-0.5^2)/sheer_modulus)^(1/3);

%Initial Values
init_position = 5; %m
init_speed = 0.4;  %m/sec^2
init_time = 0.0;   %sec
step_size = 0.001; %sec
mass_step = 2; %lbs
v_max_ideal = 0;

%% Ideal Profile
while true
    count = 1;
    %Data arrays
    power_array = [];
    position_array = [];
    velocity_array = [];
    acceleration_array = [];
    time_array = [];
    time_array(count,1) = init_time;
    velocity_array(count,1) = init_speed;
    position_array(count,1) = init_position;
    while true
        curr_vel = velocity_array(count,1);
        power_motor = ideal_power(curr_vel,gr,wheel_rad);
        if (power_motor < 0) %emergency brake
            disp('Emergency Brake due to Motor RPM > 6500')
            if (mass_optimize_flag == true)
                disp(['Not Possible to Reach 300mph with gr of ' num2str(gr)])
                emergency_brake_flag = true;
            end
            v_max_ideal = curr_vel;
            p_max_ideal = power_array(count-1,1);
            angular_v_wheel = curr_vel/wheel_rad;
            angular_v_motor = angular_v_wheel*gr;
            motor_rpm = angular_v_motor*radToRPM;
            rpm_max_ideal = motor_rpm;
            break
        end
        power_array(count,1) = power_motor;
        acceleration_array(count,1) = power_motor/(pod_mass*curr_vel);
        velocity_array(count + 1,1) = curr_vel + step_size*acceleration_array(count,1);
        position_array(count + 1,1) = position_array(count,1) + step_size*curr_vel;
        time_array(count + 1,1) = time_array(count,1) + step_size;
        if((position_array(count + 1,1)+(velocity_array(count + 1,1))^2/(decel*2)) > max_dist)
            v_max_ideal = velocity_array(count + 1,1);
            p_max_ideal = power_motor;
            angular_v_wheel = curr_vel/wheel_rad;
            angular_v_motor = angular_v_wheel*gr;
            motor_rpm = angular_v_motor*radToRPM;
            rpm_max_ideal = motor_rpm;
            time_max_ideal = time_array(count + 1,1);
            count = count + 1;
            break;
        end
        count = count + 1;  
    end

    while true
    acceleration_array(count,1) = -1*decel;
    if(velocity_array(count,1) + step_size*acceleration_array(count,1) <= 0)
        break
    end
    time_array(count + 1,1) = time_array(count,1) + step_size;
    velocity_array(count + 1,1) = velocity_array(count,1) + step_size*acceleration_array(count,1);
    position_array(count + 1,1) = position_array(count,1) + step_size*velocity_array(count,1);
    power_array(count + 1,1) = 0;
    count = count + 1;
    end
    
    if (mass_optimize_flag == true && v_max_ideal < 300/mpsToMph && emergency_brake_flag == false) 
        pod_mass = pod_mass - mass_step*lbsToKg;
        continue;
    else 
        break;
    end
end

if (mass_optimize_flag == true) 
    disp(['Optimizing for speed with gr of ' num2str(gr) ' gives max speed of ' num2str(v_max_ideal*mpsToMph) 'mph at ' num2str(pod_mass/lbsToKg) 'lbs'])
end

data(1).time = time_array;
data(1).position = position_array;
data(1).velocity = velocity_array;
data(1).acceleration = acceleration_array;
data(1).power = power_array;

%% Loss Profile
outer = 2;
v_max_loss = [];
p_max_loss = [];
rpm_max_loss = [];
while outer < 5
    count = 1;
    power_loss_array = [];
    power_loss_idler_array = [];
    power_array = [];
    position_array = [];
    velocity_array = [];
    acceleration_array = [];
    time_array = [];
    wheel_temp_array = [];
    wheel_idler_temp_array = [];
    time_array(count,1) = init_time;
    velocity_array(count,1) = init_speed;
    position_array(count,1) = init_position;
    wheel_temp_array(count,1) = init_temp;
    heat_gen_sum = 0;
    heat_gen_sum_idler = 0;
    while true
        curr_vel = velocity_array(count,1);
        power_motor = ideal_power(curr_vel,gr,wheel_rad);
        power_array(count,1) = power_motor;
        Power = power_motor*belt_loss;
        curr_temp = (heat_gen_sum/(c*mass_tread))*9/5 + init_temp;
        curr_temp_idler = (heat_gen_sum_idler/(c*mass_tread_idler))*9/5 + init_temp;
        
        if (outer == 2) 
            power_loss = Hysteresis_Model(curr_temp, Preload, deformation, wheel_rad, wheel_height, tread_rad, curr_vel, ratio);
            power_loss_idler = Hysteresis_Model(curr_temp_idler, Preload_idler, deformation_idler, idler_rad, wheel_height_idler, idler_tread_rad, curr_vel, idler_ratio);
        elseif (outer == 3)
            power_loss = Coefficient_Model(coef_rolling_resistance,Preload,wheel_rad,curr_vel);
            power_loss_idler = Coefficient_Model(coef_rolling_resistance,Preload_idler,idler_rad,curr_vel);
        else
            power_loss = Theory_Model(curr_temp, force_friction_prelim, curr_vel,ratio);
            power_loss_idler = Theory_Model(curr_temp_idler, force_friction_prelim_idler, curr_vel,idler_ratio);
        end
        
        heat_gen_sum = heat_gen_sum + power_loss*step_size;
        heat_gen_sum_idler  = heat_gen_sum_idler + power_loss_idler*step_size;
        Power = Power - power_loss - power_loss_idler*num_idler_wheels;
        power_loss_array(count,1) = power_loss;
        power_loss_idler_array(count,1) = power_loss_idler*num_idler_wheels;
        
        acceleration_array(count,1) = Power/(pod_mass*curr_vel);
        velocity_array(count + 1,1) = curr_vel + step_size*acceleration_array(count,1);
        position_array(count + 1,1) = position_array(count,1) + step_size*curr_vel;
        time_array(count + 1,1) = time_array(count,1) + step_size;
        wheel_temp_array(count + 1,1) = curr_temp;
        wheel_idler_temp_array(count + 1,1) = curr_temp_idler;

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
        curr_temp_idler = (heat_gen_sum_idler/(c*mass_tread_idler))*9/5 + init_temp;
        
        if (outer == 2) 
            power_loss = Hysteresis_Model(curr_temp, Preload, deformation, wheel_rad, wheel_height, tread_rad, curr_vel, ratio);
            power_loss_idler = Hysteresis_Model(curr_temp_idler, Preload_idler, deformation_idler, idler_rad, wheel_height_idler, idler_tread_rad, curr_vel, idler_ratio);
        elseif (outer == 3)
            power_loss = Coefficient_Model(coef_rolling_resistance,Preload,wheel_rad,curr_vel);
            power_loss_idler = Coefficient_Model(coef_rolling_resistance,Preload_idler,idler_rad,curr_vel);
        else
            power_loss = Theory_Model(curr_temp, force_friction_prelim, curr_vel,ratio);
            power_loss_idler = Theory_Model(curr_temp_idler, force_friction_prelim_idler, curr_vel,idler_ratio);
        end

        heat_gen_sum = heat_gen_sum + power_loss*step_size;
        heat_gen_sum_idler  = heat_gen_sum_idler + power_loss_idler*step_size;
        power_loss_array(count,1) = power_loss;
        power_loss_idler_array(count,1) = power_loss_idler*num_idler_wheels;
        
        acceleration_array(count,1) = -1*decel;
        if(velocity_array(count,1) + step_size*acceleration_array(count,1) <= 0)
            break
        end
        time_array(count + 1,1) = time_array(count,1) + step_size;
        velocity_array(count + 1,1) = curr_vel + step_size*acceleration_array(count,1);
        position_array(count + 1,1) = position_array(count,1) + step_size*curr_vel;
        power_array(count + 1,1) = 0;
        wheel_temp_array(count + 1,1) = curr_temp;
        wheel_idler_temp_array(count + 1,1) = curr_temp_idler;
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

%% Plotting
figure(1)
subplot(1,3,1)
plot(data(1).time,data(1).position);
title('Position')
xlabel('Time (sec)')
ylabel('Position (m)')
hold on
plot(data(2).time,data(2).position);
hold on
plot(data(3).time,data(3).position);
hold on
plot(data(4).time,data(4).position);
legend('Ideal','Loss (Hysteresis)','Loss (Coefficient)','Loss (Theory)','Location','southwest')
hold off

subplot(1,3,2)
plot(data(1).time,data(1).velocity*mpsToMph);
title('Velocity')
ylabel('Velocity (mph)')
xlabel('Time (sec)')
hold on
plot(data(2).time,data(2).velocity*mpsToMph);
hold on
plot(data(3).time,data(3).velocity*mpsToMph);
hold on
plot(data(4).time,data(4).velocity*mpsToMph);
legend(['Ideal: ' num2str(v_max_ideal*mpsToMph) ' mph'],['Loss (Hysteresis): ' num2str(v_max_loss(1,1)*mpsToMph) ' mph'],['Loss (Coefficient): ' num2str(v_max_loss(2,1)*mpsToMph) ' mph'],['Loss (Theory): ' num2str(v_max_loss(3,1)*mpsToMph) ' mph'],'Location','southwest')
hold off

subplot(1,3,3)
plot(data(1).time,data(1).acceleration);
title('Accelertion')
xlabel('Time (sec)')
ylabel('Acceleration (m/sec^2)')
hold on
plot(data(2).time,data(2).acceleration);
hold on
plot(data(3).time,data(3).acceleration);
hold on
plot(data(4).time,data(4).acceleration);
legend('Ideal','Loss (Hysteresis)','Loss (Coefficient)','Loss (Theory)','Location','southwest')
hold off

figure(2)
subplot(1,2,1)
plot(data(1).time,data(1).power);
title('Motor Power (label of max motor power)')
xlabel('Time (sec)')
ylabel('Watts')
hold on
plot(data(2).time,data(2).power);
hold on
plot(data(3).time,data(3).power);
hold on
plot(data(4).time,data(4).power);
legend(['Ideal: ' num2str(p_max_ideal/1000) ' kW, ' num2str(rpm_max_ideal) ' rpm'],['Loss (Hysteresis): ' num2str(p_max_loss(1,1)/1000) ' kW, ' num2str(rpm_max_loss(1,1)) ' rpm'],['Loss (Coefficient): ' num2str(p_max_loss(2,1)/1000) ' kW, ' num2str(rpm_max_loss(2,1)) ' rpm'],['Loss (Theory): ' num2str(p_max_loss(3,1)/1000) ' kW, ' num2str(rpm_max_loss(3,1)) ' rpm'],'Location','southwest')
hold off

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

%% Functions
function power_motor = ideal_power(v,gr,radius) 
global radToRPM power_boost_flag
%c0 = -186.2; c1 = 269; c2 = -0.6867; c3 = 0.004759; c4 = -0.00001214; c5 = 0.00000000957;
c0= 239.8790812; c1= 0.00036227; c2 = 5.02763E-07; c3=-4.9019E-10; c4=3.7E-14; %torque vs. rpm
angular_v_wheel = v/radius;
angular_v_motor = angular_v_wheel*gr;
motor_rpm = angular_v_motor*radToRPM;
torque = c0+c1*motor_rpm+c2*(motor_rpm)^2+c3*(motor_rpm)^3+c4*(motor_rpm)^4;
power_motor = torque*angular_v_motor;
if (motor_rpm >= 6500)
    power_motor = -1; %emmergency brake
end
if (power_boost_flag == true)
    power_motor = power_motor*1.1;
end
end

function power_loss = Hysteresis_Model(curr_temp, Preload, deformation, wheel_rad, wheel_height, tread_rad, curr_vel, ratio)
    frequency = curr_vel/(2*pi*wheel_rad);
    hr = ratio*loss_mod_value(curr_temp)/sqrt(loss_mod_value(curr_temp)^2+storage_mod_value(curr_temp)^2);
    Total_Strain_energy_density = 1/2*Preload*deformation/(pi*wheel_rad^2*wheel_height-pi*tread_rad^2*wheel_height);
    Loss_Part = Total_Strain_energy_density*hr;
    power_loss = Loss_Part*frequency;
end

function power_loss = Coefficient_Model(coef_rolling_resistance,Preload,wheel_rad,curr_vel)
    rolling_force = coef_rolling_resistance*Preload/wheel_rad;
    power_loss = rolling_force*curr_vel;
end

function power_loss = Theory_Model(curr_temp, force_friction_prelim, curr_vel,ratio)
    hr = ratio*loss_mod_value(curr_temp)/sqrt(loss_mod_value(curr_temp)^2+storage_mod_value(curr_temp)^2);  
    rolling_force = force_friction_prelim*hr;
    power_loss = rolling_force*curr_vel;
end