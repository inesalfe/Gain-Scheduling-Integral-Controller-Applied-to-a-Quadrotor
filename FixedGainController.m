%%%%%%%%% Part C: Fixed-Gain Controller for the Quadrotor %%%%%%%%%

%% New system variables %%%

syms int_error_x int_error_y int_error_z int_error_psi ...
     ref_x       ref_y       ref_z       ref_psi       ...
     error_x     error_y     error_z     error_psi    

int_error = [ int_error_x ; int_error_y ; int_error_z ; int_error_psi ];
ref       = [ ref_x       ; ref_y       ; ref_z       ; ref_psi       ];
 
% 1. Augmented x-space representation %%

error = y - ref;

% Augment x

state_aug    = [ x    ; int_error ]; 

% Augment x equation

f_x_u_aug = [ f_x_u ; error ];

% 2. Design Fixed-Gain Controller  %%

%%% Define the equilibrium point 

x_ss = [ eval(traj_x(0)) ; eval(traj_y(0)) ; eval(traj_z(0)) ; zeros(3, 1)  ; ... 
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
                 
Q_error = diag( [ 1 ; 1 ; 1 ; 1 ] );

Q = blkdiag( Q_state, Q_error );
 
R = diag( [ 0.1 ; 0.1 ; 0.1 ; 0.1 ] );
 
K = lqr( A_int_stable, B_int_stable, Q , R );

% 3. Experiments %%%%%%

% Replace generic u with fixed-gain controller

state_eq_control = f_x_u_aug;
state_eq_control = subs( state_eq_control, u, -K(:, 1:12) * ( x - transpose( C ) * ref ) - K(:, 13:16) * int_error + u_ss);

% Experiment - Lemniscate curve with fixed yaw

syms traject_zero;

traject_zero = [ traj_x(t); traj_y(t); traj_z(t) ; 0 ];

state_eq_control = subs( state_eq_control, ref, traject_zero );

% Convert symbolic function to Ode45 u

quadrotor_fixedGain = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
odeFun_quadrotor_fixedGain = @( t_sim, state_sim ) quadrotor_fixedGain( t_sim, state_sim );

% Define time span and initial conditions for Ode45

t_min  = 0; t_step = 0.01;
t_span = t_min:t_step:t_max;  
 
state_initial = [ eval(traj_x(0)) ; eval(traj_y(0)) ; eval(traj_z(0)) ; zeros(3, 1)  ; ... 
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
plot(  eval(traj_x (0:t_step:t_max) ), ... 
       eval(traj_y (0:t_step:t_max) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
  
xlim([ -10   10  ]);
ylim([ -10  10 ]);
xlabel('Position X (m)' , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y (m)' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:t_step:t_max, ... 
       eval(traj_z (0:t_step:t_max) ), ... 
       'LineStyle', '--', 'LineWidth', 2);

xlim([ 0   t_max ]);
ylim([ 0   10     ]);
xlabel('Time (s)' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Altitude (m)' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:t_step:t_max, ... 
       0*(0:t_step:t_max), ... 
       'LineStyle', '--', 'LineWidth', 2);
   
xlim([ 0     t_max ]);
ylim([ -pi   pi ]);
xlabel('Time (s)' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('$\psi$ (rad)' ,  'interpreter', 'latex', 'fontsize', 15)

%% Experiment - Lemniscate curve with tangential yaw

syms int_error_x int_error_y int_error_z int_error_psi ...
     ref_x       ref_y       ref_z       ref_psi       ...
     error_x     error_y     error_z     error_psi    

int_error = [ int_error_x ; int_error_y ; int_error_z ; int_error_psi ];
ref       = [ ref_x       ; ref_y       ; ref_z       ; ref_psi       ];
 
% 1. Augmented x-space representation %%

error = y - ref;

% Augment x

state_aug    = [ x    ; int_error ]; 

% Augment x equation

f_x_u_aug = [ f_x_u ; error ];

%%% Define the equilibrium point 

x_ss = [ eval(traj_x(0)) ; eval(traj_y(0)) ; eval(traj_z(0)) ; zeros(3, 1)  ; ... 
                 0 ; 0 ; eval(traj_phi(0)) ; zeros(3, 1) ];
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
                 
Q_error = diag( [ 1 ; 1 ; 1 ; 1 ] );

Q = blkdiag( Q_state, Q_error );
 
R = diag( [ 0.1 ; 0.1 ; 0.1 ; 0.1 ] );
 
K = lqr( A_int_stable, B_int_stable, Q , R );

% 3. Experiments %%%%%%

% Replace generic u with fixed-gain controller

state_eq_control = f_x_u_aug;
state_eq_control = subs( state_eq_control, u, -K(:, 1:12) * ( x - transpose( C ) * ref ) - K(:, 13:16) * int_error + u_ss);

state_eq_control = subs( state_eq_control, ref, traject_user );

% Convert symbolic function to Ode45 u

quadrotor_fixedGain = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
odeFun_quadrotor_fixedGain = @( t_sim, state_sim ) quadrotor_fixedGain( t_sim, state_sim );

% Define time span and initial conditions for Ode45

t_max = 30;
t_min  = 0; t_step = 0.01;
t_span = t_min:t_step:t_max;  
 
state_initial = [ eval(traj_x(0)) ; eval(traj_y(0)) ; eval(traj_z(0)) ; zeros(3, 1)  ; ... 
                  0 ; 0 ; eval(traj_phi(0)) ; zeros(7, 1) ];              
              
% Run Ode45 Solver

[ t_sim, state_sim ]  = ode45( @(t_sim, state_sim) odeFun_quadrotor_fixedGain( t_sim, state_sim ), t_span, state_initial );

%%% Plot results
tiledlayout(3, 2)

nexttile([2 2])
plot( state_sim(:, 1), ... 
      state_sim(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  eval(traj_x (0:t_step:t_max) ), ... 
       eval(traj_y (0:t_step:t_max) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
  
xlim([ -10   10  ]);
ylim([ -10  10 ]);
xlabel('Position X (m)' , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y (m)' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:t_step:t_max, ... 
       eval(traj_z (0:t_step:t_max) ), ... 
       'LineStyle', '--', 'LineWidth', 2);

xlim([ 0   t_max ]);
ylim([ 0   10     ]);
xlabel('Time (s)' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Altitude (m)' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim, ... 
      state_sim(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  0:t_step:t_max, ... 
       eval(traj_phi(0:t_step:t_max)), ... 
       'LineStyle', '--', 'LineWidth', 2);
   
xlim([ 0     t_max ]);
ylim([ -pi   pi ]);
xlabel('Time (s)' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('$\psi$ (rad)' ,  'interpreter', 'latex', 'fontsize', 15)