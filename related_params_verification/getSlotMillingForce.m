function [Force_x, Force_y, Force, Torque, Power, t, phi] = getSlotMillingForce(Ks, beta, b, feedRate, d, Nt, omega, steps, ifPlot)
%GETSLOTMILLINGFORCE
%   Inputs:
%       Ks: material cutting force per unit area, N/mm^2
%       beta: deg
%       b: axial depth, mm
%       feedRate: feed rate, mm/s
%       d: cutter diameter, mm
%       Nt: number of teeth, int
%       omega: spindle speed, rpm
%       steps: steps for one cutter revolution

%% Simulation specifications
kt = Ks*sin(deg2rad(beta));            % tangential specific force, N/mm^2
kn = Ks*cos(deg2rad(beta));           % normal specific force, N/mm^2

ft = feedRate*60/(omega*Nt);      % feed per tooth, mm/tooth

%slotting operation
phis = 0;                     % cut start angle, deg
phie = 180;                     % cut exit angle, deg
gamma = 0;                      % helix angle, deg (ignored)

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
Torque = zeros(1, steps);
Power = zeros(1, steps);

%% Cutting force,  Torque & Power
for cnt1 = 1:10*steps              % time steps, s
    for cnt2 = 1:Nt              
   		teeth(cnt2) = teeth(cnt2) + 1;      % index teeth one position (rotate cutter by dphi)
	    if teeth(cnt2) > steps
	      	teeth(cnt2) = 1;
        end
    end

    Fx = 0;
    Fy = 0;
    torq = 0;
    for cnt3 = 1:Nt                         % sum forces over all teeth
        for cnt4 = 1:steps_axial            % sum forces along axial depth of helical endmill
            phi_counter = teeth(cnt3) - (cnt4-1);
            if phi_counter < 1              % helix has wrapped through phi = 0 deg
                phi_counter = phi_counter + steps;
            end
            phia = phi(phi_counter);        % angle for given axial disk, deg
            
            if (phia >= phis) && (phia <= phie)          % verify that tooth angle is in specified range for current disk, deg
                h = ft*sin(phia*pi/180);                % chip thickness, m
                Ft = kt*db*h;                           % tangential force, N
                Fn = kn*db*h;                           % normal force, N
         	else    % tooth angle is outside range bounded by radial immersion
        		Ft = 0;
                Fn = 0;
            end
            
            torq = torq + 0.5*d*1e-3*Ft;     %N*m
            Fx = Fx + Fn*sin(phia*pi/180) + Ft*cos(phia*pi/180);    % N
            Fy = Fy - Fn*cos(phia*pi/180) + Ft*sin(phia*pi/180);    % N
        end     % cnt4 loop
    end         % cnt3 loop
    
    Force_x(cnt1) = Fx;
    Force_y(cnt1) = Fy;
    Torque(cnt1) = torq;    %N*m
    Power(cnt1) = torq*omega*2*pi/60;    %W
    t(cnt1) = (cnt1-1)*dt;
    tooth_angle(cnt1) = (cnt1-1)*dphi;
end             % cnt1 loop

% Calculate resultant force
Force = (Force_x.^2 + Force_y.^2).^0.5; % N
% time = tooth_angle/omega*60/360;

%% Plot
if ~exist("ifPlot", "var")
    ifPlot = true;
end

if ifPlot
    figure(1)
    plot(tooth_angle, Force_x, 'r:', tooth_angle, Force_y, 'b--', tooth_angle, Force, 'k')
    % axis([0 360 -150 400])
    xlim([0 360]);
    set(gca,'FontSize', 12)
    xlabel('\phi (deg)')
    ylabel('Force (N)')
    legend('F_x', 'F_y', 'F')
    
    figure(2)
    plot(t, Force_x, 'r:', t, Force_y, 'b--', t, Force, 'k')
    % axis([0 0.008 -150 400])
    xlim([0 0.008]);
    set(gca,'FontSize', 12)
    xlabel('t (s)')
    ylabel('Force (N)')
    legend('F_x', 'F_y', 'F')
    
    % plot Torque & Power
    %x--phi
    figure;
    subplot(2,1,1);
    plot(tooth_angle, Torque, 'k');
    xlim([0 360]);
    set(gca,'FontSize', 12)
    xlabel('\phi (deg)')
    ylabel('Torque (N*m)')
    
    subplot(2,1,2);
    plot(tooth_angle, Power, 'k');
    xlim([0 360]);
    set(gca,'FontSize', 12)
    xlabel('\phi (deg)')
    ylabel('Power (W)')
    
    %x--t
    figure;
    subplot(2,1,1);
    plot(t, Torque, 'r');
    xlim([0 0.008]);
    set(gca,'FontSize', 12)
    xlabel('t (s)')
    ylabel('Torque (N*m)')
    
    subplot(2,1,2);
    plot(t, Power, 'r');
    xlim([0 0.008]);
    set(gca,'FontSize', 12)
    xlabel('t (s)')
    ylabel('Power (W)')
end