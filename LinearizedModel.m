%%%%%%%%% Part B. Linearized Model of the Quadrotor %%%%%%%%%

%% Equilibrium points %%

x_ss = [ pos_x ; pos_y ; pos_z ; zeros(3, 1)
                 0 ; 0 ; psi ; zeros(3, 1) ];
u_ss = [ parms.mass * parms.grav ; zeros(3, 1) ];

% The subtitution of the x with the stable x and the u with the
% stable u must give f = 0; Function "all" testes if all values are 1.
% So assert will not give an error if the x and u are indeed stable
assert( ~ all(subs(f_x_u, [x ; u], [x_ss ; u_ss] ) ) )

%% Linearization around an equilibrium point %%

%%% Compute the linearization matrices 

A = jacobian( f_x_u, x );
B = jacobian( f_x_u, u );
C = jacobian( y,   x );

%%% Compute the linearization around a generic equilibrium point 

x_ss = [ pos_x ; pos_y ; pos_z ; zeros(3, 1) ; 0 ; 0 ; psi ; zeros(3, 1) ];
u_ss = [ parms.mass * parms.grav ; zeros(3, 1) ];
  
A_stable = subs( A, [ x ; u ], [ x_ss ; u_ss ] );
B_stable = subs( B, [ x ; u ], [ x_ss ; u_ss ] );
 
disp( A_stable );
disp( B_stable );

%% Stability of the equilibrium points %%

%%% Assess the eigenvalues of the linearization matrix A around a generic 
%%% equilibrium point 

disp( eig(A_stable) );

%%% We observe that the all eigenvalues are zero. In this case, linearization 
%%% fails to determine the stability of the system and must proceed by simulating
%%% the dynamic behaviour of the quadrotor.

% Replace in x equation generic u with stable u

state_eq_control = subs( f_x_u, u, u_ss );

% Convert symbolic function to Ode45 u

quadrotor = matlabFunction( state_eq_control, 'vars', { x } );
%quadrotor = matlabFunction( state_eq_control, 'vars', { t, x } );
odeFun_quadrotor = @( t_sim, state_sim ) quadrotor( state_sim );

% Define time span and initial x for Ode45

t_min  = 0; t_step = 0.01; t_max = 20;
t_span = t_min:t_step:t_max;  

state_initial_1 = [ 0    ; 0   ; 5  ; zeros(3, 1) ; ...
                    0    ; 0   ; 0  ; zeros(3, 1) ];

state_initial_2 = [ 0    ; 0   ; 5  ; zeros(3, 1) ; ...
                    pi/3 ; pi/4 ; 0  ; zeros(3, 1) ];

state_initial_3 = [ 0    ; 0   ; 5  ;  1 ; -3 ; -1  ; ...
                    0    ; 0   ; 0  ; zeros(3, 1) ];

state_initial_4 = [ 0    ; 0   ; 5  ; zeros(3, 1) ; ...
                    0    ; 0   ; 0  ; pi/10 ; -pi/5 ; pi/ 3 ];                
                
% Run Ode45 Solver
[ t_sim_1, state_sim_1 ]  = ode45( @(t_sim_1, state_sim_1) odeFun_quadrotor( t_sim_1, state_sim_1 ), t_span, state_initial_1 );
[ t_sim_2, state_sim_2 ]  = ode45( @(t_sim_2, state_sim_2) odeFun_quadrotor( t_sim_2, state_sim_2 ), t_span, state_initial_2 );
[ t_sim_3, state_sim_3 ]  = ode45( @(t_sim_3, state_sim_3) odeFun_quadrotor( t_sim_3, state_sim_3 ), t_span, state_initial_3 );
[ t_sim_4, state_sim_4 ]  = ode45( @(t_sim_4, state_sim_4) odeFun_quadrotor( t_sim_4, state_sim_4 ), t_span, state_initial_4 );

%%% Plot position trajectory 

f1 = figure('Name', 'Stability of the equilibrium points');

tiledlayout(2,2)

nexttile
plot( t_sim_1,           ...
      state_sim_1(:, 1), ...  
      'LineWidth', 2 );
hold on
plot( t_sim_2,           ...
      state_sim_2(:, 1), ...  
      'LineWidth', 2 );
hold on
plot( t_sim_3,           ...
      state_sim_3(:, 1), ...  
      'LineWidth', 2 );
hold on
plot( t_sim_4,           ...
      state_sim_4(:, 1), ...  
      'LineWidth', 2 );
  
grid on
xlim([ 0   10 ]);
ylim([ -5   5 ]);
xlabel('Time '          , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position X '    , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_1,           ...
      state_sim_1(:, 2), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_2,           ...
      state_sim_2(:, 2), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_3,           ...
      state_sim_3(:, 2), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_4,           ...
      state_sim_4(:, 2), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');  

grid on
xlim([  0  10 ]);
ylim([  -5  5 ]);
xlabel('Time'           , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Y'     , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_1,           ...
      state_sim_1(:, 3), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_2,           ...
      state_sim_2(:, 3), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_3,           ...
      state_sim_3(:, 3), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_4,           ...
      state_sim_4(:, 3), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');  

grid on
xlim([ 0   10 ]);
ylim([ 0   10 ]);
xlabel('Time'        , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Position Z ' , 'interpreter', 'latex', 'fontsize', 15)

nexttile
plot( t_sim_1,           ...
      state_sim_1(:, 9), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_2,           ...
      state_sim_2(:, 9), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_3,           ...
      state_sim_3(:, 9), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
hold on
plot( t_sim_4,           ...
      state_sim_4(:, 9), ...  
      'LineWidth', 2, 'DisplayName', 'Trajectory');
  
grid on
xlim([ 0       10 ]);
ylim([ -2*pi 2*pi ]);
xlabel('Time'      , 'interpreter', 'latex', 'fontsize', 15)
ylabel('Angle Yaw' , 'interpreter', 'latex', 'fontsize', 15)
