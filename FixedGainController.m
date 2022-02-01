%%%%%%%%% Part C: Fixed-Gain Controller for the Quadrotor %%%%%%%%%

%% New system variables %%%

syms int_error_x int_error_y int_error_z int_error_psi ...
     ref_x       ref_y       ref_z       ref_psi       ...
     error_x     error_y     error_z     error_psi    

int_error = [ int_error_x ; int_error_y ; int_error_z ; int_error_psi ];
ref       = [ ref_x       ; ref_y       ; ref_z       ; ref_psi       ];
 
%% 1. Augmented x-space representation %%

error = y - ref;

% Augment x

state_aug    = [ x    ; int_error ]; 

% Augment x equation

f_x_u_aug = [ f_x_u ; error ];

%% 2. Design Fixed-Gain Controller  %%

%%% Define the equilibrium point 

x_ss = [ 0 ; 0 ; 5 ; zeros(3, 1)  ; ... 
                 0 ; 0 ; 0 ; zeros(3, 1) ];
u_ss = [ parms.mass * parms.grav  ; zeros(3, 1) ];

%%% Determine linearization of the closed-loop system
%%% around the equilibrium point

A_int = [ A zeros(12, 4) ; C zeros(4, 4)  ];
B_int = [ B   ;   zeros(4, 4)  ];

A_int_stable = eval( subs( A_int, [ x ; u ], [ x_ss ; u_ss ] ) );
B_int_stable = eval( subs( B_int, [ x ; u ], [ x_ss ; u_ss ] ) );

%%% Determine fixed-gain matrix via LQR formulation

Q_state = diag( [ 1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ]     );
                 
Q_error = diag( [ 1  ; 1  ; 1  ; 1 ] );

Q = blkdiag( Q_state, Q_error );
 
R = diag( [ 0.0001 ; 0.0001 ; 0.0001 ; 0.0001 ] );

% Q_state = diag( [ 1  ; 1  ; 1  ;     ... 
%                   1  ; 1  ; 1  ;     ... 
%                   1  ; 1  ; 1  ;     ... 
%                   1  ; 1  ; 1  ]     );
%                  
% Q_error = diag( [ 0.01  ; 0.01  ; 0.01  ; 0.01 ] );
% 
% Q = blkdiag( Q_state, Q_error );
%  
% R = diag( [ 0.1 ; 0.001 ; 0.001 ; 0.001 ] );
 
K = lqr( A_int_stable, B_int_stable, Q , R );

%% 3. Experiments %%%%%%

% Replace generic u with fixed-gain controller

state_eq_control = f_x_u_aug;
state_eq_control = subs( state_eq_control, u, -K(:, 1:12) * x - K(:, 13:16) * int_error);
% x + c(1, 12) .* random(1, 12) %% Maybe just position?

%% Experiment - Take-off
 
%%% Define take-off trajectory

syms take_off_x(t) take_off_y(t) take_off_z(t) take_off_yaw(t)

take_off_x(t)   = 5;
take_off_y(t)   = 5;
take_off_z(t)   = 5;
take_off_yaw(t) = 0;

traject = [ take_off_x(t) ; take_off_y(t) ; take_off_z(t) ; take_off_yaw(t) ];

state_eq_control = subs( state_eq_control, ref, traject );

% Convert symbolic function to Ode45 u

quadrotor_fixedGain = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
odeFun_quadrotor_fixedGain = @( t_sim, state_sim ) quadrotor_fixedGain( t_sim, state_sim );
 
% Define time span and initial x for Ode45

t_min = 0; t_step = 0.01; t_max = 50;
t_span = t_min : t_step : t_max;

state_initial_1 = [ 0  ; 0  ; 0     ; zeros(3, 1)  ; ... 
                    0  ; 0  ; 0     ; zeros(7, 1) ]; 
state_initial_2 = [ 10 ; 10 ; 0     ; zeros(3, 1)  ; ... 
                    0  ; 0  ; -pi/8 ; zeros(7, 1) ]; 
state_initial_3 = [ 0  ; 10 ;  10   ; zeros(3, 1)  ; ... 
                    0  ; 0  ; pi/8  ; zeros(7, 1) ];                 
state_initial_4 = [ 10 ; 0  ;  10   ; zeros(3, 1)  ; ... 
                    0  ; 0  ; -pi/8 ; zeros(7, 1) ];                   
                
[ t_sim_1, state_sim_1 ]  = ode45( @(t_sim_1, state_sim_1) odeFun_quadrotor_fixedGain( t_sim_1, state_sim_1 ), t_span, state_initial_1 ); 
[ t_sim_2, state_sim_2 ]  = ode45( @(t_sim_2, state_sim_2) odeFun_quadrotor_fixedGain( t_sim_2, state_sim_2 ), t_span, state_initial_2 ); 
[ t_sim_3, state_sim_3 ]  = ode45( @(t_sim_3, state_sim_3) odeFun_quadrotor_fixedGain( t_sim_3, state_sim_3 ), t_span, state_initial_3 ); 
[ t_sim_4, state_sim_4 ]  = ode45( @(t_sim_4, state_sim_4) odeFun_quadrotor_fixedGain( t_sim_4, state_sim_4 ), t_span, state_initial_4 ); 
 
%%% Plot results
% tiledlayout(2, 2)
% 
% nexttile
% plot( t_sim_1, ... 
%       state_sim_1(:, 1), ... 
%       'LineWidth', 2, 'DisplayName', 'Trajectory');
% hold on;
% plot( t_sim_2, ... 
%       state_sim_2(:, 1), ... 
%       'LineWidth', 2, 'DisplayName', 'Trajectory');  
% hold on;
% plot( t_sim_3, ... 
%       state_sim_3(:, 1), ... 
%       'LineWidth', 3, 'DisplayName', 'Trajectory');    
% hold on;
% hold on;
% plot( t_sim_4, ... 
%       state_sim_4(:, 1), ... 
%       'LineWidth', 3, 'DisplayName', 'Trajectory'); 
% 
% xlim([ 0   30 ]);
% ylim([ 0   10 ]);
% xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
% ylabel('Position X' , 'interpreter', 'latex', 'fontsize', 15)
% 
% 
% nexttile
% plot( t_sim_1, ... 
%       state_sim_1(:, 2), ... 
%       'LineWidth', 2, 'DisplayName', 'Trajectory');
% hold on;
% plot( t_sim_2, ... 
%       state_sim_2(:, 2), ... 
%       'LineWidth', 2, 'DisplayName', 'Trajectory');  
% hold on;
% plot( t_sim_3, ... 
%       state_sim_3(:, 2), ... 
%       'LineWidth', 3, 'DisplayName', 'Trajectory');    
% hold on;
% hold on;
% plot( t_sim_4, ... 
%       state_sim_4(:, 2), ... 
%       'LineWidth', 3, 'DisplayName', 'Trajectory'); 
% 
% xlim([ 0   30 ]);
% ylim([ 0   10 ]);
% xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
% ylabel('Position Y' , 'interpreter', 'latex', 'fontsize', 15)

tiledlayout(3, 2)

nexttile([2 2])
plot( state_sim_1(:, 1), ... 
      state_sim_1(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( state_sim_2(:, 1), ... 
      state_sim_2(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( state_sim_3(:, 1), ... 
      state_sim_3(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( state_sim_4(:, 1), ... 
      state_sim_4(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');  
  
xlim([ 0  10 ]);
ylim([ 0  10 ]);
xlabel('Position X' , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_1, ... 
      state_sim_1(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( t_sim_2, ... 
      state_sim_2(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');  
hold on;
plot( t_sim_3, ... 
      state_sim_3(:, 3), ... 
      'LineWidth', 3, 'DisplayName', 'Trajectory');    
hold on;
hold on;
plot( t_sim_4, ... 
      state_sim_4(:, 3), ... 
      'LineWidth', 3, 'DisplayName', 'Trajectory'); 

xlim([ 0   30 ]);
ylim([ 0   10 ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Z' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_1, ... 
      state_sim_1(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( t_sim_2, ... 
      state_sim_2(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( t_sim_3, ... 
      state_sim_3(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot( t_sim_4, ... 
      state_sim_4(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');  
hold on;
   
xlim([ 0      30 ]);
ylim([ -pi/2 pi/2 ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Angle Yaw' ,  'interpreter', 'latex', 'fontsize', 15)


%% Experiment - Lemniscate curve with fixed yawn

%%% Define Lemniscate trajectory with fixed yawn and varying altitude
%%% ( See: https://mathworld.wolfram.com/Lemniscate.html )

syms lemniscate_x(t) lemniscate_y(t) lemniscate_z(t) lemniscate_yaw(t)

% Define trajectory foci and frequency of the Lemniscate trajectory

a = 10 / sqrt(2); 
w = pi/40 ;       

lemniscate_x(t)   = a * sqrt(2) .* sin( w * t ) ./ ( ( 1 + cos( w * t )^2 ) ); 
lemniscate_y(t)   = a * sqrt(2) .* cos( w * t ) .* sin( w * t ) ./ ( ( 1 + cos( w * t )^2 ) );
lemniscate_z(t)   = 5;
lemniscate_yaw(t) = 0;

traject = [ lemniscate_x(t) ; lemniscate_y(t) ; lemniscate_z(t) ; lemniscate_yaw(t) ];

state_eq_control = subs( state_eq_control, ref, traject );

% Convert symbolic function to Ode45 u

quadrotor_fixedGain = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
odeFun_quadrotor_fixedGain = @( t_sim, state_sim ) quadrotor_fixedGain( t_sim, state_sim );
 
% Define time span and initial conditions for Ode45

t_min  = 0; t_step = 0.01; t_max = 2*pi/w;
t_span = t_min:t_step:t_max;  
 
state_initial = [ 0 ; 0 ; 5 ; zeros(3, 1)  ; ... 
                  0 ; 0 ; 0 ; zeros(7, 1) ];              
              
% Run Ode45 Solver

[ t_sim, state_sim ]  = ode45( @(t_sim, state_sim) odeFun_quadrotor_fixedGain( t_sim, state_sim ), t_span, state_initial );

%%% Plot results
tiledlayout(3, 2)

nexttile([2 2])
plot( state_sim(:, 1), ... 
      state_sim(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  eval(lemniscate_x (0:0.1:2*pi/w) ), ... 
       eval(lemniscate_y (0:0.1:2*pi/w) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
  
xlim([ -15   15  ]);
ylim([ -7.5  7.5 ]);
xlabel('Position X' , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:0.1:2*pi/w, ... 
       eval(lemniscate_z (0:0.1:2*pi/w) ), ... 
       'LineStyle', '--', 'LineWidth', 2);

xlim([ 0   2*pi/w ]);
ylim([ 0   10     ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Z' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:0.1:2*pi/w, ... 
       eval(lemniscate_yaw (0:0.1:2*pi/w) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
   
xlim([ 0     2*pi/w ]);
ylim([ -2*pi   2*pi ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Angle Yaw' ,  'interpreter', 'latex', 'fontsize', 15)

%% Experiment - Lemniscate trajectory with tangential yawn

%%% Define Lemniscate trajectory with tangential yawn 

syms lemniscate_x(t) lemniscate_y(t) lemniscate_z(t) lemniscate_yaw(t)

% Define trajectory foci and frequency of the Lemniscate trajectory

a = 10 / sqrt(2);  
%w = pi/5 ;       
% w = pi/30 ;       
w = pi/500 ;       

lemniscate_x(t)   = a * sqrt(2) .* sin( w * t ) ./ ( ( 1 + cos( w * t )^2 ) ); 
lemniscate_y(t)   = a * sqrt(2) .* cos( w * t ) .* sin( w * t ) ./ ( ( 1 + cos( w * t )^2 ) );
lemniscate_z(t)   = 5; 
lemniscate_yaw(t) = ( (lemniscate_y(t) == 0) & ( lemniscate_x(t) == 0 ) & (lemniscate_x(t+0.1) > 0) ) * ...
                        ( pi / 4 )        + ...
                    ( (lemniscate_y(t) == 0) & ( lemniscate_x(t) == 0 ) & (lemniscate_x(t+0.1) < 0) ) * ...
                        ( - pi - pi / 4 ) + ...
                    ( (lemniscate_y(t) == 0) & ( lemniscate_x(t) ~= 0 ) ) * ...
                        ( - pi/ 2 )       + ...                   
                    ( ( (lemniscate_x(t) >= 0) & (lemniscate_y(t) > 0) ) | ( (lemniscate_x(t) <  0) & (lemniscate_y(t) <  0) ) ) * ...
                              ( atan( ( lemniscate_x(t) .* (a^2 - lemniscate_x(t).^2 - lemniscate_y(t).^2 ) ) ./ ( ( lemniscate_y(t) + 0.0001 ) * ( a^2 + lemniscate_x(t).^2 + lemniscate_y(t).^2 ) ) ) ) + ...
                    ( ( (lemniscate_x(t) >= 0) & (lemniscate_y(t) < 0) ) | ( (lemniscate_x(t) <  0) & (lemniscate_y(t) >  0) ) ) * ...
                        ( -pi + atan( ( lemniscate_x(t) .* (a^2 - lemniscate_x(t).^2 - lemniscate_y(t).^2 ) ) ./ ( ( lemniscate_y(t) + 0.0001 ) * ( a^2 + lemniscate_x(t).^2 + lemniscate_y(t).^2 ) ) ) );

traject = [ lemniscate_x(t) ; lemniscate_y(t) ; lemniscate_z(t) ; lemniscate_yaw(t) ];

% Replace in x equation generic reference with the trajectory

state_eq_control = subs( state_eq_control, ref, traject );

% Convert symbolic function to Ode45 u

quadrotor_fixedGain = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
odeFun_quadrotor_fixedGain = @( t_sim, state_sim ) quadrotor_fixedGain( t_sim, state_sim );

% Define time span and initial x for Ode45

%t_min  = 0; t_step = 0.01; t_max = 3;
t_min  = 0; t_step = 0.01; t_max = 13.75;
%t_min  = 0; t_step = 0.01; t_max = 31.5;
t_span = t_min:t_step:t_max;  

state_initial = [ 0 ; 0 ; 5    ; zeros(3, 1)  ; ... 
                  0 ; 0 ; pi/4 ; zeros(7, 1) ];     
  
% Run Ode45 Solver
[ t_sim, state_sim ]  = ode45( @(t_sim, state_sim) odeFun_quadrotor_fixedGain( t_sim, state_sim ), t_span, state_initial );

%%% Plot results
tiledlayout(3, 2)

nexttile([2 2])
plot( state_sim(:, 1), ... 
      state_sim(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  eval(lemniscate_x (0:0.1:2*pi/w) ), ... 
       eval(lemniscate_y (0:0.1:2*pi/w) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
  
xlim([ -15   15  ]);
ylim([ -7.5  7.5 ]);
xlabel('Position X' , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:0.1:2*pi/w, ... 
       eval(lemniscate_z (0:0.1:2*pi/w) ), ... 
       'LineStyle', '--', 'LineWidth', 2);

xlim([ 0   2*pi/w ]);
ylim([ 0   10     ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Z' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:0.1:2*pi/w, ... 
       eval(lemniscate_yaw (0:0.1:2*pi/w) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
   
xlim([ 0     2*pi/w ]);
ylim([ -3/2*pi pi/2 ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Angle Yaw' ,  'interpreter', 'latex', 'fontsize', 15)

%% Plot Results

% Plot yaw trajectory and reference


grid on
xlim([ 0    10 ]);
ylim([ 0    10   ]);
xlabel('Time' ,      'interpreter', 'latex', 'fontsize', 15)
ylabel('Angle Yaw' , 'interpreter', 'latex', 'fontsize', 15)
legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
