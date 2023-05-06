% p_4_1_2_3.m
% Copyright
% University of Florida
% T. Schmitz
% June 1, 2008

clear all
close all
clc

% Simulation specifications
kt = 750;                       % tangential specific force, N/mm^2
kn = 250;                       % normal specific force, N/mm^2
b = 5;                          % axial depth, mm
ft = 0.1;                       % feed per tooth, mm/tooth
phis = 120;                     % cut start angle, deg
phie = 180;                     % cut exit angle, deg
gamma = 0;                      % helix angle, deg
d = 19;                         % cutter diameter, mm
Nt = 4;                         % number of teeth, int
omega = 7500;                   % spindle speed, rpm
steps = 2^12;                   % steps for one cutter revolution, int
dt = 60/(steps*omega);          % time step, s
dphi = 360/steps;               % angular step, deg
if gamma == 0                   % straight teeth
    db = b;                     % discretized axial depth, m
else                            % nonzero helix angle
    db = d*(dphi*pi/180)/2/tan(gamma*pi/180);
end
steps_axial = round(b/db);      % number of steps along tool axis
tooth_angles = 0:360/Nt:(360-360/Nt);  % angles of Nt cutter teeth starting from zero, deg

% Initialize vectors
teeth = round(tooth_angles/dphi) + 1;
phi = linspace(0, (steps-1)*dphi, steps);
Force_x = zeros(1, steps);
Force_y = zeros(1, steps);

% Main program
for cnt1 = 1:10*steps              % time steps, s
    for cnt2 = 1:Nt              
   		teeth(cnt2) = teeth(cnt2) + 1;      % index teeth one position (rotate cutter by dphi)
	    if teeth(cnt2) > steps
	      	teeth(cnt2) = 1;
      	end
    end

    Fx = 0;
    Fy = 0;
    for cnt3 = 1:Nt                         % sum forces over all teeth
        for cnt4 = 1:steps_axial            % sum forces along axial depth of helical endmill
            phi_counter = teeth(cnt3) - (cnt4-1);
            if phi_counter < 1              % helix has wrapped through phi = 0 deg
                phi_counter = phi_counter + steps;
            end
            phia = phi(phi_counter);        % angle for given axial disk, deg
            
            if (phia >= phis) & (phia <= phie)          % verify that tooth angle is in specified range for current disk, deg
                h = ft*sin(phia*pi/180);                % chip thickness, m
                Ft = kt*db*h;                           % tangential force, N
                Fn = kn*db*h;                           % normal force, N
         	else    % tooth angle is outside range bounded by radial immersion
        		Ft = 0;
                Fn = 0;
            end

            Fx = Fx + Fn*sin(phia*pi/180) + Ft*cos(phia*pi/180);    % N
            Fy = Fy - Fn*cos(phia*pi/180) + Ft*sin(phia*pi/180);    % N
        end     % cnt4 loop
    end         % cnt3 loop
    
    Force_x(cnt1) = Fx;
    Force_y(cnt1) = Fy;
    t(cnt1) = (cnt1-1)*dt;
    tooth_angle(cnt1) = (cnt1-1)*dphi;
end             % cnt1 loop

% Calculate resultant force
Force = (Force_x.^2 + Force_y.^2).^0.5; % N
time = tooth_angle/omega*60/360;

figure(1)
plot(tooth_angle, Force_x, 'r:', tooth_angle, Force_y, 'b--', tooth_angle, Force, 'k')
axis([0 360 -150 400])
set(gca,'FontSize', 14)
xlabel('\phi (deg)')
ylabel('Force (N)')
legend('F_x', 'F_y', 'F')

figure(2)
plot(t, Force_x, 'r:', t, Force_y, 'b--', t, Force, 'k')
axis([0 0.008 -150 400])
set(gca,'FontSize', 14)
xlabel('t (s)')
ylabel('Force (N)')
legend('F_x', 'F_y', 'F')

% Calculate FFT of time domain Force signal
N = length(Force);
fs = 1/dt;              % sampling frequency, Hz
F_mean = mean(Force);
Force = Force - F_mean; % remove mean prior to computing FFT
F = fft(Force');
F = F(1:N/2+1);
F = F/(N/2);            % correct amplitude
F(1) = F_mean;          % replace DC value with mean
f = [0:fs/N:(1-1/(2*N))*fs]';
f = f(1:N/2+1);         % frequency, Hz

figure(3)
plot(f, abs(F))
axis([0 5000 0 160])
set(gca,'FontSize', 14)
xlabel('f (Hz)')
ylabel('F (N)')
