%%%%%%%%% Part C: Fixed-Gain Controller for the Quadrotor %%%%%%%%%

%% New system variables %%%

syms int_error_x int_error_y int_error_z int_error_psi ...
     ref_x       ref_y       ref_z       ref_psi       ...
     error_x     error_y     error_z     error_psi    

int_error = [ int_error_x ; int_error_y ; int_error_z ; int_error_psi ];
ref       = [ ref_x       ; ref_y       ; ref_z       ; ref_psi       ];
 
%% 1. Augmented state-space representation %%

% Define error

error = y - ref;

% Augment x

state_aug    = [ x    ; int_error ]; 

% Augment x equation

f_x_u_aug = [ f_x_u ; error ];

%% 2. Design Fixed-Gain Controller  %%

%%% Define the equilibrium point 

x_ss = [ 5 ; 10 ; 5 ; zeros(3, 1)  ; ... 
                 0 ; 0 ; 0 ; zeros(3, 1) ];
u_ss = [ parms.mass * parms.grav  ; zeros(3, 1) ];

%%% Determine linearization of the closed-loop system
%%% around the equilibrium point

A_int = [ A zeros(12,4) ; C zeros(4, 4)  ];
B_int = [ B   ;   zeros(4, 4)  ];

A_int_stable = eval( subs( A_int, [ x ; u ], [ x_ss ; u_ss ] ) );
B_int_stable = eval( subs( B_int, [ x ; u ], [ x_ss ; u_ss ] ) );

Q_state = diag( [ 1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ]     );
                 
Q_error = diag( [ 0.01 ; 0.01 ; 0.01 ; 0.01 ] );

Q = blkdiag( Q_state, Q_error );
 
R = diag( [ 0.1 ; 0.001 ; 0.001 ; 0.001 ] );
 
K = lqr( A_int_stable, B_int_stable, Q , R );

%% 3. Experiments %%%%%%

% Replace generic input with fixed-gain controller

state_eq_control = f_x_u_aug;
state_eq_control = subs( state_eq_control, u, -K(:, 1:12) * ( x - transpose( C ) * ref ) - K(:, 13:16) * int_error + u_ss );
% state + c(1, 12) .* random(1, 12) %% Maybe just position?

%% Experiment - Take-off
 
%%% Define take-off trajectory

syms take_off_x(t) take_off_y(t) take_off_z(t) take_off_yaw(t)

take_off_x(t)   = 5;
take_off_y(t)   = 10;
take_off_z(t)   = 5;
take_off_yaw(t) = 0;

traject = [ take_off_x(t) ; take_off_y(t) ; take_off_z(t) ; take_off_yaw(t) ];

state_eq_control = subs( state_eq_control, ref, traject );

% Convert symbolic function to Ode45 input

quadrotor_fixedGain = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
odeFun_quadrotor_fixedGain = @( t_sim, state_sim ) quadrotor_fixedGain( t_sim, state_sim );
 
% Define time span and initial state for Ode45

t_min = 0; t_step = 0.01; t_max = 35;
t_span = t_min : t_step : t_max;

state_initial_1 = [ 7  ; 15  ; 0     ; zeros(3, 1)  ; ... 
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
  
xlim([ 2  15 ]);
ylim([ 2  15 ]);
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
