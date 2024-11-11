% This script is meant to analyze all the flight datas and simulated ones,
% note that in order to don't repeat the importing of datas, it is nedeed to
% start this first parts before using every other one


%% Starting values
clc; clear; close all;

% some datas that could be implement by coding

alfa_starting_deg = 85;   
burnout_time = 4.5; % in s
after_burnout_time = 155.5; % in s
total_time = burnout_time + after_burnout_time;

initial_mass_rocket = 35.082; % in kg
final_mass_rocket = 24.153; % in kg
fuel_mass = initial_mass_rocket - final_mass_rocket;

variation_fuel_mass = fuel_mass / burnout_time;
burnout_linear_time = linspace(0, burnout_time, 571);

%import datas
EasyMini = readtable("Flight Datas\EASYMINI\PRT14-EASYMINI-DATALOG-EUROC.csv", "VariableNamingRule","preserve");

                                                                                                    


height_EasyMini          = EasyMini.height;
speed_EasyMini           = EasyMini.speed;
time_EasyMini            = EasyMini.time;
acceleration_EasyMini    = EasyMini.acceleration;
state_EasyMini           = EasyMini.state;
pressure_EasyMini        = EasyMini.pressure;
altitude_EasyMini        = EasyMini.altitude;
temperature_EasyMini     = EasyMini.temperature;
drogue_voltage_EasyMini  = EasyMini.drogue_voltage;
main_voltage_EasyMini    = EasyMini.main_voltage;
battery_voltage_EasyMini = EasyMini.battery_voltage;


Vega_baro               = readtable("Flight Datas\VEGA\baro.csv", "VariableNamingRule","preserve");
Vega_voltageinfo        = readtable("Flight Datas\VEGA\voltageinfo.csv", "VariableNamingRule","preserve");
Vega_orientation        = readtable("Flight Datas\VEGA\orientationInfo.csv", "VariableNamingRule","preserve");
Vega_event              = readtable("Flight Datas\VEGA\eventInfo.csv", "VariableNamingRule","preserve");
Vega_filtered_data_info = readtable("Flight Datas\VEGA\filteredDataInfo.csv","VariableNamingRule","preserve");
Vega_flight_info        = readtable("Flight Datas\VEGA\flightInfo.csv", "VariableNamingRule","preserve");
Vega_flight_states      = readtable("Flight Datas\VEGA\flightStates.csv", "VariableNamingRule","preserve");
Vega_gnss_info          = readtable("Flight Datas\VEGA\gnssInfo.csv", "VariableNamingRule","preserve");
Vega_imu                = readtable("Flight Datas\VEGA\imu.csv", "VariableNamingRule","preserve"); 

state_event_Vega                     = Vega_event.event;
state_action_Vega                    = Vega_event.action;
state_argument_Vega                  = Vega_event.argument;
state_time_Vega                      = Vega_event.ts;
altitude_Vega                        = Vega_filtered_data_info.filteredAltitudeAGL;
height_Vega                          = Vega_flight_info.height;
speed_Vega                           = Vega_flight_info.velocity;
acceleration_Vega_filtered_data_info = Vega_filtered_data_info.filteredAcceleration;
acceleration_Vega_flight_info         = Vega_flight_info.acceleration;
time_Vega                            = Vega_filtered_data_info.ts;
pressure_Vega                        = Vega_baro.P;
temperature_Vega                     = Vega_baro.T;
q0_orientation_Vega                  = Vega_orientation.q0_estimated;
q1_orientation_Vega                  = Vega_orientation.q1_estimated;
q2_orientation_Vega                  = Vega_orientation.q2_estimated;
q3_orientation_Vega                  = Vega_orientation.q3_estimated;
voltage_time_Vega                    = Vega_voltageinfo.ts;
voltage_value_Vega                   = Vega_voltageinfo.voltage;
gnss_latitude_Vega                   = Vega_gnss_info.latitude;
gnss_longitude_Vega                  = Vega_gnss_info.longitude;
gnss_time_Vega                       = Vega_gnss_info.ts;
gnss_satellite_Vega                  = Vega_gnss_info.satellites;
ax_imu_Vega                          = Vega_imu.Ax;
ay_imu_Vega                          = Vega_imu.Ay;
az_imu_Vega                          = Vega_imu.Az;
gx_imu_Vega                          = Vega_imu.Gx;
gy_imu_Vega                          = Vega_imu.Gy;
gz_imu_Vega                          = Vega_imu.Gz;

Propulsion = readtable("Flight Datas\PROPULSION\Cesaroni_8088M1790-P.csv");

propulsion_time   = Propulsion.motor_;
propulsion_thrust = Propulsion.Cesaroni8088M1790_P;


Environment = readtable("Flight Datas\ROCKETPY\environment.csv", "VariableNamingRule","preserve");

% OpenRocket = readtable("Flight Datas\OPENROCKET\OpenRocket.csv", "VariableNamingRule","preserve");
% 
% speed_OpenRocket        = OpenRocket.speed;
% height_OpenRocket       = OpenRocket.height;
% time_OpenRocket         = OpenRocket.time;
% altitude_OpenRocket     = OpenRocket.altitude;
% acceleration_OpenRocket = OpenRocket.acceleration;
% state_OpenRocket        = OpenRocket.state;



%Note that Vega's flight states hasn't been used in this script

%% Starting Costants
clc

G       = 6.6742 * 10^-11; % gravitational costant
R_earth = 6371000;
M_earth = 5.972 * 10^24;



%% Starting functions
clc

% gravity as function of height

gravity_function = @(h) (G .* M_earth) ./ (R_earth + h).^2;

% mass of rocket as function of time supposing the fuel is lineary burnt

mass_rocket_burnout = @(t) initial_mass_rocket - variation_fuel_mass * t;


% degree to rad

deg_to_rad = @(x) pi * x / 180;


%% Adjusting and Starting derivative datas
clc 

%Starting derivative datas

gravity             = gravity_function(height_Vega); %gravity has been calculated on Vega's datas
mass_during_burnout = mass_rocket_burnout(burnout_linear_time);

%turning alfa into radians
alfa_starting = deg_to_rad(alfa_starting_deg);

% A - same for Vega's time, adding the fact that we need the same number of
% time's sample for every data

time_Vega = time_Vega(2:end);
time_vega = time_Vega + abs(time_Vega);

% B - deleting the first value from acceleration and altitude for the same
% reason
acceleration_Vega_filtered_data_info = acceleration_Vega_filtered_data_info(2:end);


% creating an unique acceleration for Vega (don't know if filtered is
% trustable)

variation_filtered_acceleration = acceleration_Vega_filtered_data_info - acceleration_Vega_flight_info;
acceleration_Vega               = acceleration_Vega_filtered_data_info; %we are trusting the filtered datas


%creating adjusted vectors
gravity_burnout = gravity(1:571);
acceleration_burnout = acceleration_Vega(1:571);

x                       = 1:614;
x_new                   = 1:0.04317:614; 
gnss_latitude_Vega_res  = interp1(x,gnss_latitude_Vega,x_new); 
gnss_longitude_Vega_res = interp1(x,gnss_longitude_Vega,x_new); 

x                       = 1:14;
x_new                   = 1:0.025:14; 
propulsion_thrust_res   = interp1(x,propulsion_thrust,x_new);

x                       = 1:571;
x_new                   = 1:1.095:571; % Nuovo vettore di ascisse più denso
gravity_burnout         = interp1(x,gravity_burnout,x_new);
acceleration_burnout    = interp1(x,acceleration_burnout,x_new);
mass_burnout            = interp1(x, mass_during_burnout, x_new);
burnout_linear_time_res = interp1(x, burnout_linear_time, x_new);


%% 1 - Changing State comparison
clc;

%EasyMini
actual_event_EasyMini = state_EasyMini(1);
c_state_EasyMini      = [actual_event_EasyMini time_EasyMini(1)];

for a = 1:size(state_EasyMini)
    if state_EasyMini(a) ~= actual_event_EasyMini
        actual_event_EasyMini = state_EasyMini(a);
        c_state_EasyMini = [c_state_EasyMini, [actual_event_EasyMini time_EasyMini(a)]];
    end
end

% Vega, the changes are done cause Vega detected 2 slightly different times
% when the event changed

actual_event_Vega = state_event_Vega(1);
c_state_Vega      = [actual_event_Vega state_time_Vega(1)];

for a = 1:size(state_event_Vega)
    if state_event_Vega(a) ~= actual_event_Vega
        actual_event_Vega = state_event_Vega(a);
        if actual_event_Vega ~= 3
            actual_time_Vega = (state_time_Vega(a) + state_time_Vega(a + 2))/2;
        else
            actual_time_Vega = state_time_Vega(a);
        end
        c_state_Vega = [c_state_Vega, [actual_event_Vega actual_time_Vega]]; %#ok<*AGROW>
    end
end

% It has been noticed that ids don't match completely between EasyMini
% and Vega. There's also to be noticed that Vega hasn't recorded the
% landing state. The differences should be checked in the manuals

% % OpenRocket
% actual_event_OpenRocket = state_OpenRocket(1);
% c_state_OpenRocket = [actual_event_OpenRocket time_OpenRocket(1)];
% 
% for a = 1: size(state_OpenRocket)
%     if state_OpenRocket(a) ~= actual_event_OpenRocket
%         actual_event_OpenRocket = state_OpenRocket(a);
%         c_state_OpenRocket = [c_state_OpenRocket, [actual_event_OpenRocket time_OpenRocket(a)]];
%     end
% end


%% 2 - All datas plots
clc;

%EasyMini
figure(1)
plot(time_EasyMini, height_EasyMini, 'k', "LineWidth", 1.5, "DisplayName", 'Height')
hold on
plot(time_EasyMini, speed_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", 'Speed')
hold on
plot(time_EasyMini, acceleration_EasyMini, 'r', "LineWidth", 1.5, "DisplayName", 'Acceleration')
hold off
grid on
title("EasyMini's Datas")
xlabel('time')
legend('location',"northeast")

%Vega
figure(2)
plot(time_Vega, height_Vega, 'k', 'LineWidth', 1.5, 'DisplayName', 'Height')
hold on
plot(time_Vega, speed_Vega, 'b', 'LineWidth', 1.5, 'DisplayName', 'Speed')
hold on
plot(time_Vega, acceleration_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', 'Acceleration')
hold off
grid on
title("Vega's Datas")
xlabel('time')
legend(Location="northeast")

% %OpenRocket
% figure(3)
% plot(time_OpenRocket, height_OpenRocket, 'k', 'LineWidth', 1.5, 'DisplayName', 'Height')
% hold on
% plot(time_OpenRocket, speed_OpenRocket, 'b', 'LineWidth', 1.5, 'DisplayName', 'Speed')
% hold on
% plot(time_OpenRocket, acceleration_OpenRocket, 'r', 'LineWidth', 1.5, 'DisplayName', 'Acceleration')
% hold off
% grid on
% title("OpenRocket's Datas")
% xlabel('time')
% legend(Location="northeast")


%% 2.1 - Trajectory
clc;

x1 = gnss_latitude_Vega_res(1);
y1 = gnss_longitude_Vega_res(1);
z1 = altitude_Vega(1);

plot3(gnss_latitude_Vega_res, gnss_longitude_Vega_res, altitude_Vega, 'b', "LineWidth", 1.5, "DisplayName", "Trajectory")
hold on
plot(gnss_latitude_Vega_res, gnss_longitude_Vega_res, 'g', 'LineWidth', 1.5, 'DisplayName', 'XY Trajectory')
hold on
plot3(x1, y1, z1, 'ko', "DisplayName", 'Launch Site', 'LineWidth',1.5)
hold off
grid on
title('Trajectory')
xlabel('latitude')
ylabel('longitude')
zlabel('altitude')
legend('Location','northeast')


%% 3 - Height comparison
clc;

% Vega - EasyMini
figure(4)
plot(time_EasyMini, height_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", "EasyMini height")
hold on
plot(time_Vega, height_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', "Vega height")
hold off
grid on
title("Vega - EasyMini height")
xlabel('time')
ylabel('meter')
legend('location', "northeast")

% % Vega - EasyMini - OpenRocket
% figure(5)
% plot(time_EasyMini, height_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", "EasyMini height")
% hold on
% plot(time_Vega, height_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', "Vega height")
% hold on
% plot(time_OpenRocket, height_OpenRocket, 'k', 'LineWidth', 1.5, 'DisplayName', "OpenRocket height")
% hold off
% grid on
% title("Vega - EasyMini - OpenRocket height")
% xlabel('time')
% ylabel('meter')
% legend(Location="northeast")


%% 4 - Speed comparison
clc;

% Vega - EasyMini
figure(6)
plot(time_EasyMini, speed_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", "EasyMini speed")
hold on
plot(time_Vega, speed_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', "Vega speed")
hold off
grid on
title("Vega - EasyMini speed")
xlabel('time')
ylabel('m/s')
legend(Location="northeast")

% % Vega - EasyMini - OpenRocket
% figure(7)
% plot(time_EasyMini, speed_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", "EasyMini speed")
% hold on
% plot(time_Vega, speed_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', "Vega speed")
% hold on
% plot(time_OpenRocket, speed_OpenRocket, 'k', 'LineWidth', 1.5, 'DisplayName', "OpenRocket speed")
% hold off
% grid on
% title("Vega - EasyMini - OpenRocket speed")
% xlabel('time')
% ylabel('m/s')
% legend(Location="northeast")


%% 5 - Acceleration comparison
clc;

% Vega - EasyMini
figure(8)
plot(time_EasyMini, acceleration_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", "EasyMini acceleration")
hold on
plot(time_Vega, acceleration_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', "Vega acceleration")
hold off
grid on
title("Vega - EasyMini acceleration")
xlabel('time')
ylabel('m/s^2')
legend(Location="northeast")

% % Vega - EasyMini - OpenRocket
% figure(9)
% plot(time_EasyMini, acceleration_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", "EasyMini acceleration")
% hold on
% plot(time_Vega, acceleration_Vega, 'r', 'LineWidth', 1.5, 'DisplayName', "Vega acceleration")
% hold on 
% plot(time_OpenRocket, acceleration_OpenRocket, 'k', 'LineWidth', 1.5, 'DisplayName', "OpenRocket acceleration")
% hold off
% grid on
% title("Vega - EasyMini - OpenRocket acceleration")
% xlabel('time')
% ylabel('m/s^2')
% legend(Location="northeast")
% 

%% 5.1 - Acceleration closeup boostphase comparison 
clc;

time_burnout_end_EasyMini = 5.28;
for a = 1 : size(time_EasyMini)
    if time_EasyMini(a) == time_burnout_end_EasyMini
        index_EasyMini = a;
    end
end

time_burnout_end_Vega = 3.79;
for a = 1 : size(time_Vega)
    if time_Vega(a) == time_burnout_end_Vega
        index_Vega = a;
    end
end

% Vega - EasyMini
figure(10)
plot(time_EasyMini(1:index_EasyMini), acceleration_EasyMini(1:index_EasyMini), 'b', "LineWidth", 1.5, "DisplayName", "EasyMini acceleration")
hold on
plot(time_Vega(1:index_Vega), acceleration_Vega(1:index_Vega), 'r', 'LineWidth', 1.5, 'DisplayName', "Vega acceleration")
hold off
grid on
title("Vega - EasyMini acceleration boostphase")
xlabel('time')
ylabel('m/s^2')
legend(Location="northeast")

% there has to be understod how to find index_OpenRocket

% % Vega - EasyMini - OpenRocket
% figure(11)
% plot(time_EasyMini(1:index_EasyMini), acceleration_EasyMini(1:index_EasyMini), 'b', "LineWidth", 1.5, "DisplayName", "EasyMini acceleration")
% hold on
% plot(time_Vega(1:index_Vega), acceleration_Vega(1:index_Vega), 'r', 'LineWidth', 1.5, 'DisplayName', "Vega acceleration")
% hold on
% plot(time_OpenRocket(1:index_OpenRocket), acceleration_OpenRocket(1:index_OpenRocket), 'k', 'LineWidth', 1.5, 'DisplayName', "OpenRocket acceleration")
% hold off
% grid on
% title("Vega - EasyMini - OpenRocket acceleration boostphase")
% xlabel('time')
% ylabel('m/s^2')
% legend(Location="northeast")

%% 5.2 Acceleration Comparison between filtered and non filtered Vega

figure(12)
plot(time_vega(1:2700), acceleration_Vega_filtered_data_info(1:2700), 'r', "LineWidth", 1.5, "DisplayName", 'Filtered')
hold on
plot(time_vega(1:2700), acceleration_Vega_flight_info(1:2700), 'b', 'LineWidth', 1.5, "DisplayName", 'Non Filtered')
hold off
grid on
title('Vegas Accelerations')
xlabel('time')
ylabel('m/s^2')
legend('location', 'northeast')

% due to the barely visible difference, there has to be done a close up,
% even due to the fact that the two accelereation differe from each other
% only before an exact time, after which they both started to find a 0 value
% acceleration


%% 5.3 Ax Ay Az comparison
clc

figure(13)
plot(time_vega, ax_imu_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'x acceleration')
grid on
xlabel('time')
ylabel('m/s^2')
title('X Acceleration')


figure(14)
plot(time_vega, ay_imu_Vega, 'b', "LineWidth", 1.5, "DisplayName", 'y acceleration')
grid on
xlabel('time')
ylabel('m/s^2')
title('Y Acceleration')

figure(15)
plot(time_vega, az_imu_Vega, 'g', "LineWidth", 1.5, "DisplayName", 'z acceleration')
grid on
xlabel('time')
ylabel('m/s^2')
title('Z Acceleration')

%% 5.4 Gx Gy Gz comparison
clc

figure(16)
plot(time_vega, gx_imu_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'x giroscope')
grid on
xlabel('time')
ylabel('rad/s^2')
title('X Omega')

figure(17)
plot(time_vega, gy_imu_Vega, 'b', "LineWidth", 1.5, "DisplayName", 'y giroscope')
grid on
xlabel('time')
ylabel('rad/s^2')
title('Y Omega')

figure(18)
plot(time_vega, gz_imu_Vega, 'g', "LineWidth", 1.5, "DisplayName", 'z giroscope')
grid on
xlabel('time')
ylabel('rad/s^2')
title('Z Omega')


%% 6 - Voltages (singular)
clc;

size(voltage_value_Vega);

%EasyMini
figure(19)
plot(time_EasyMini, drogue_voltage_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", 'Drogue Voltage')
hold on
plot(time_EasyMini, main_voltage_EasyMini, 'r', "LineWidth", 1.5, "DisplayName", "Main Voltage")
hold off
grid on
title('EasyMini Voltage')
legend('location', 'northeast')
xlabel('time')
ylabel('voltage')

% Vega
figure(20)
plot(voltage_time_Vega, voltage_value_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'Vega Datas')
hold off
title('Vega Voltage')
xlabel('time')
ylabel('voltage')
legend('location', 'northeast')
grid on


%% 7 - Temperature + Pressure as functions of height and time
clc

%it should be done an avarage between the datas from the ascend and the
%discent of the rocket

figure(21)
plot( pressure_EasyMini,height_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", 'EasyMini pressure')
hold on
plot(pressure_Vega, height_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'Vega pressure')
hold off
grid on
xlabel('meters')
ylabel('Pascal')
title('Vega - EasyMini Pressure')
legend('location', 'southeast')

figure(22)
plot(temperature_EasyMini, height_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", 'EasyMini temperature')
hold on
plot(temperature_Vega, height_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'Vega temperature')
hold off
grid on
xlabel('meters')
ylabel('Celsius')
title('Vega - EasyMini Temperature')
legend('location', 'northeast')

figure(23)
plot(time_EasyMini, pressure_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", 'EasyMini pressure')
hold on
plot(time_Vega, pressure_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'Vega pressure')
hold off
grid on
xlabel('time')
ylabel('Pascal')
title('Vega - EasyMini Pressure')
legend('location', 'southeast')

figure(24)
plot(time_EasyMini, temperature_EasyMini, 'b', "LineWidth", 1.5, "DisplayName", 'EasyMini temperature')
hold on
plot(time_Vega, temperature_Vega, 'r', "LineWidth", 1.5, "DisplayName", 'Vega temperature')
hold off
grid on
xlabel('time')
ylabel('Celsius')
title('Vega - EasyMini Temperature')
legend('location', 'northeast')


%% 8 - Thrust
clc


% RockSim simulations

figure(25)
plot(propulsion_time, propulsion_thrust, 'r', "LineWidth", 1.5, "DisplayName", 'RockSim Thrust')
hold on
plot(propulsion_time, propulsion_thrust, 'ro', "LineWidth", 2, "DisplayName", 'RockSim Thrust')
hold off
grid on
xlabel('time')
ylabel('Thrust')
title('RockSim Thrust')


%% 9 - Drag 
clc

% let's try to approximate drag force in a 2D flight

% ASCENDING - COASTING 

% the new equation is the sequent:
m = final_mass_rocket;


% mass is costant, acceleration is supposed in 2D, so:

acceleration_2D = sqrt(ax_imu_Vega .^ 2 + ay_imu_Vega .^ 2);

% alfa is the only ' problem', however considering a 2D flight:
time_alfa = Vega_filtered_data_info.ts;
time_alfa_diff = diff(time_alfa);
alfa = [];
alfa(1) = alfa_starting;

%gz datas have to be in degrees, otherwise the rocket is rotating of about
%3 rotation per second



for a = 1:size(gz_imu_Vega)
    alfa(a + 1) = alfa(a) + deg_to_rad(gx_imu_Vega(a) .* time_alfa_diff(a));
end

new_height = zeros(1,1);

for a = 1:sum(size(alfa)) - 2
    new_height(a + 1) = new_height(a) + speed_Vega(a) * sin(alfa(a)) * time_alfa_diff(a);
end


figure(1)
plot(time_alfa, new_height, 'r', "LineWidth", 1.5, "DisplayName", 'Derivative height')
hold on
plot(time_Vega, height_Vega, 'b--', "LineWidth", 1.5, "DisplayName", 'Vega Height')
hold off
grid on
title('Verifying alfa calculation')

%the calculation of the angle of attack matches 

% gravity is already calculated, acceleration is the following:

ay_imu_Vega;

drag_1_coasting = [];
for a = 1:sum(size(alfa)) - 2
    drag_1_coasting(a) = - m .* (ay_imu_Vega(a) + gravity(a) .* sin(alfa(a)));
end

time_drag = time_Vega(455:2687);
drag_1_coasting = drag_1_coasting(455:2687);

figure(2)
plot(time_drag, drag_1_coasting, 'r')
grid on

% Drag = ro * v^2 * Cd /2, so:

Cd_function = @(D, r, v) 2 .* D ./ (r .* v .^2);

% v is known, Drag maybe too, so we only need ro:

Ra = 287.05;
ro_function = @(p,t) p ./ (Ra .* t);
ro_Vega = ro_function(pressure_Vega(455:2687), temperature_Vega(455:2687));

Cd = Cd_function(drag_1_coasting, ro_Vega, speed_Vega(455:2687));

figure(3)
plot(time_drag(1:2000), Cd(1:2000), 'r')
grid on


