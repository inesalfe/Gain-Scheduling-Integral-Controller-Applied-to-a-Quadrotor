%% Choose waypoints

addpath('SLMtools/');

fig1 = figure(1);
hold on
xlim([-10, 10])
ylim([-10, 10])

disp('Select the desired waypoints. After you select all of them, press the space key.');

button = 1;
k = 1;
while button==1
    [way_x(k), way_y(k), button] = ginput(1);
    k = k + 1;
end

% The last value is selected when a button is pressed and it should be
% ignored
way_x(end) = [];
way_y(end) = [];

close all

waypoints = [way_x', way_y'];

fig2 = figure;
scatter(waypoints(:,1), waypoints(:,2), 30);
hold off;

%% Calculate spline

t_vec = linspace(0,1,size(waypoints,1));
spline_x = spline(t_vec,waypoints(:,1)');
spline_y = spline(t_vec,waypoints(:,2)');
path_t = @(t) [ppval(spline_x,t); ppval(spline_y,t)];

n_points = 50;

t_vec = linspace(0,1,n_points);
points = path_t(t_vec)';

fig2 = figure;
scatter(waypoints(:,1), waypoints(:,2), 30);
hold on;
plot(points(:,1), points(:,2));
hold off;

syms t w traj_x(t) traj_y(t) traj_z(t) deriv_traj_x(t) deriv_traj_y(t) traj_phi(t) traject_user;

spline_x_cells = slmpar(spline_x,'symabs');
spline_y_cells = slmpar(spline_y,'symabs');

traj_x(t) = 0;
traj_y(t) = 0;
traj_z(t) = 5;
traj_phi(t) = 0;
for i = 1:size(waypoints,1)-2
    limits = cell2mat(spline_x_cells(1,i));
    traj_x(t) = traj_x(t) + cell2sym(spline_x_cells(2,i))*(t >= limits(1) & t < limits(2));
    traj_y(t) = traj_y(t) + cell2sym(spline_y_cells(2,i))*(t >= limits(1) & t < limits(2));
    deriv_traj_x(t) = diff(cell2sym(spline_x_cells(2,i)),'x');
    deriv_traj_y(t) = diff(cell2sym(spline_y_cells(2,i)),'x');
    
    traj_phi(t) = traj_phi(t) + atan2(deriv_traj_y(t),deriv_traj_x(t)) * (t >= limits(1) & t < limits(2));
end

i = size(waypoints,1)-1;
limits = cell2mat(spline_x_cells(1,i));
traj_x(t) = traj_x(t) + cell2sym(spline_x_cells(2,i))*(t >= limits(1) & t <= limits(2));
traj_y(t) = traj_y(t) + cell2sym(spline_y_cells(2,i))*(t >= limits(1) & t <= limits(2));
deriv_traj_x(t) = diff(cell2sym(spline_x_cells(2,i)),'x');
deriv_traj_y(t) = diff(cell2sym(spline_y_cells(2,i)),'x');    
traj_phi(t) = traj_phi(t) + atan2(deriv_traj_y(t),deriv_traj_x(t)) * (t >= limits(1) & t <= limits(2));

traj_x(t) = subs(traj_x(t), t, t*w);
traj_y(t) = subs(traj_y(t), t, t*w);
traj_phi(t) = subs(traj_phi(t), t, t*w);

traj_x(t) = subs(traj_x(t), 'x', t*w);
traj_y(t) = subs(traj_y(t), 'x', t*w);
traj_phi(t) = subs(traj_phi(t), 'x', t*w);

%% Discretize spline and calculate t_max

prompt = 'T_max:\n';
t_max = input(prompt);

t_init = 0;
t_step = 0.01;
t_vec = t_init:t_step:t_max;

traj_x(t) = subs(traj_x,w,1/t_max);
traj_y(t) = subs(traj_y,w,1/t_max);
traj_phi(t) = subs(traj_phi,w,1/t_max);

x_values=eval(traj_x(t_vec));
y_values=eval(traj_y(t_vec));
phi_values=eval(traj_phi(t_vec));

dist = 0;
for i = 1:size(x_values,2)-1
    dist = dist + norm([x_values(1,i+1);y_values(1,i+1)]-[x_values(1,i);y_values(1,i)]);
end

fig2 = figure;
scatter(waypoints(:,1), waypoints(:,2), 30);
hold on;
plot(x_values, y_values);
hold off;

fig3 = figure;
plot(t_vec, phi_values);
hold off;

traject_user = [ traj_x(t); traj_y(t); traj_z(t) ; traj_phi(t) ];