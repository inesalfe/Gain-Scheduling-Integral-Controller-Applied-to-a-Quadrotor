%%%%%%%%% Part D: Gain-Scheduler Controller for the Quadrotor %%%%%%%%%

%% New System Variables %%%

syms mod_int_error_x mod_int_error_y mod_int_error_z mod_int_error_psi ...
     ref_x           ref_y           ref_z           ref_psi           ...

mod_int_error = [ mod_int_error_x ; mod_int_error_y ; mod_int_error_z ; mod_int_error_psi ];
ref           = [ ref_x           ; ref_y           ; ref_z           ; ref_psi           ];

%% Setup Gain-Scheduling Controller %%

global yaw_ref yaw_tol;

%%% Define tolerance upon which yaw angle is switched

yaw_tol = pi/8;

%%% Define the linearization matrices of the closed-loop system
%%% around a generic equilibrium point

A_int = [ A zeros(12,4) ; C zeros(4, 4)];
B_int = [ B   ;   zeros(4, 4)];

%%% Define cost matrices for the LQR formulation

Q_state = diag( [ 1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ;     ... 
                  1  ; 1  ; 1  ]     );
                 
Q_error = diag( [ 1 ; 1 ; 1 ; 1 ] );

Q = blkdiag( Q_state, Q_error );
 
R = diag( [ 0.1 ; 0.1 ; 0.1 ; 0.1 ] );

%% Experiment - Lemniscate curve with tangencial yawn and varying altitude

%%% Define time span and initial x for Ode45

t_min = 0; t_step = 0.01;

t_next        = 0;
state_next    = [ eval(traj_x(0)) ; eval(traj_y(0)) ; eval(traj_z(0)) ; zeros(3, 1) ; ... 
                  0 ; 0 ; eval(traj_phi(0)) ; zeros(7, 1) ];   
              
t_sim_acc     = [];           
state_sim_acc = [];
u_out         = [];

%%% Augment x-space representation

state_aug    = [ x  ; mod_int_error ]; 

u_ss = [ parms.mass * parms.grav  ; zeros(3, 1) ];

while t_next < t_max - t_step
  
    %%%%%% A. Compute gains matrices of the controller %%%%%%
     
    %%% Define new value of scheduling variable 
    
    yaw_ref = state_next(9);
    
    %%% Define new equilibrium point (with disturbance of the nominal mass)

    x_ss = [ state_next(1:3)       ; zeros(3, 1) ; 
                     0 ; 0 ; state_next(9) ; zeros(3, 1) ];
    
    %%% Compute linearization matrices of the closed-loop system at new 
    %%% equilibrium point
    
    A_int_steady = eval( subs( A_int, [ x ; u ], [ x_ss ; u_ss ] ) );
    B_int_steady = eval( subs( B_int, [ x ; u ], [ x_ss ; u_ss ] ) );
    
    %%% Compute new gain matrices via LQR formulation
    
    K = lqr( A_int_steady, B_int_steady, Q , R );
    
    %%%%%% B. Define x-space representation of the open-loop system %%%%%%
    
    %%% Replace in x equation generic u with gain-scheduling controller

    state_eq_control = [ f_x_u ; -K(:, 13:16) * ( C * x - ref ) ];
    state_eq_control = subs( state_eq_control, u, -K(:, 1:12) * ( x - transpose( C ) * ref ) + mod_int_error + u_ss );
    
    %%% Replace in x equation generic reference with the trajectory

    state_eq_control = subs( state_eq_control, ref, traject_user );

    %%%%%% C. Simulate behaviour of the open-loop system %%%%%%
   
    %%% Convert symbolic function representing x equation to Ode45 u

    quadrotor_gainScheduler = matlabFunction( state_eq_control, 'vars', { t, state_aug } );
    odeFun_quadrotor_gainScheduler = @( t_sim, state_sim ) quadrotor_gainScheduler( t_sim, state_sim );
            
    t_span = t_next : t_step : t_max;
    state_initial = state_next;
        
    options              = odeset('Events', @changeController);
    [ t_sim, state_sim ] = ode45( @(t_sim, state_sim) odeFun_quadrotor_gainScheduler( t_sim, state_sim ), t_span, state_initial, options );

    %%% Define initial conditions for next epoch  

    u_out = [ u_out ; (-K(:, 1:12) * ( state_sim(:,1:12)' - transpose( C ) * [eval(traj_x(t_sim))'; eval(traj_y(t_sim))'; eval(traj_z(t_sim))' ; eval(traj_phi(t_sim))'] ) + state_sim(:,13:16)' + u_ss*ones(1,size(t_sim,1)))' ] ;
    
    t_next     = t_sim(end);
    state_next = transpose( state_sim(end, :) );
    
    %%% Save results from current epoch

    t_sim_acc     = [ t_sim_acc     ; t_sim     ];
    state_sim_acc = [ state_sim_acc ; state_sim ];

end    

t_vec = t_min : t_step : t_max;

%% Plot results
tiledlayout(3, 2)

nexttile([2 2])
plot( state_sim_acc(:, 1), ... 
      state_sim_acc(:, 2), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  eval(traj_x (t_vec) ), ... 
       eval(traj_y (t_vec) ), ... 
       'LineStyle', '--', 'LineWidth', 2);
  
xlim([ -11  11  ]);
ylim([ -11  11 ]);
xlabel('Position X' , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_acc, ... 
      state_sim_acc(:, 3), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  t_vec, ... 
       eval(traj_z (t_vec) ), ... 
       'LineStyle', '--', 'LineWidth', 2);

xlim([ 0   t_max ]);
ylim([ 0   10     ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Z' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_acc, ... 
      state_sim_acc(:, 9), ... 
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on;
plot(  t_vec, ... 
       eval(traj_phi (t_vec) ), ... 
       'LineStyle', '--', 'LineWidth', 2);

xlim([ 0     t_max ]);
ylim([ -2*pi   2*pi ]);
xlabel('Time' ,       'interpreter', 'latex', 'fontsize', 15)
ylabel('$\psi$' ,  'interpreter', 'latex', 'fontsize', 15)
hold off;

fig1 = figure;
plot(u_out);

%% Miscellaneous

function rmsq = RootMeanSquaredError( y, ref )

    rmsq = sqrt( sumsqr( y - ref ) / ( size(y, 1) * size(y, 2) ) );
    
end

function [value, isterminal, direction] = changeController( ~, state_sim )

    global yaw_ref  yaw_tol;
        
    value      = ( abs( state_sim(9) - yaw_ref ) > yaw_tol );
    isterminal = 1;  
    direction  = 0;
end