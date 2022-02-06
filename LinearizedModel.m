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