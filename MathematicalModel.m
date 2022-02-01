close all; clear; clc 

%%%%%%%%% Part A. Mathematical Model of the Quadrotor %%%%%%%%%

%% System variables and parameters %%

syms t                                                     ...
     pos_x                 pos_y                 pos_z                 ... 
     v_x_b   v_y_b   v_z_b   ...
     theta               phi               psi               ...  
     w_theta_b w_phi_b w_psi_b ...
     f            tor_theta           tor_phi                tor_psi 

pos     = [pos_x   ; pos_y   ; pos_z];
ang_pos = [phi ; theta ; psi];
v_b     = [v_x_b   ; v_y_b   ; v_z_b];
w_b = [w_phi_b ; w_theta_b ; w_psi_b];

parms.grav   = 9.8;
parms.mass   = 0.8;
parms.inertia_x = 0.005; 
parms.inertia_y = 0.005;
parms.inertia_z = 0.009;

%% Dynamic Behavior %%

% Velocity in the inertial frame 

vel_x = cos( theta ) * cos( psi ) * v_x_b + ( sin( phi ) * sin( theta ) * cos( psi ) - cos( phi ) * sin( psi ) ) * v_y_b + ( cos( phi ) * sin( theta ) * cos( psi ) + sin( phi ) * sin( psi ) ) * v_z_b;
vel_y = cos( theta ) * sin( psi ) * v_x_b + ( sin( phi ) * sin( theta ) * sin( psi ) + cos( phi ) * cos( psi ) ) * v_y_b + ( cos( phi ) * sin( theta ) * sin( psi ) - sin( phi ) * cos( psi ) ) * v_z_b;
vel_z =              sin( theta ) * v_x_b                                        - ( sin( phi ) * cos( theta ) ) * v_y_b                                        - ( cos( phi ) * cos( theta ) ) * v_z_b;
vel = [ vel_x ; vel_y ; vel_z ];

% Aceleration in the body frame 

ace_x_bodyframe = w_psi_b * v_y_b - w_theta_b * v_z_b - parms.grav * sin( theta );
ace_y_bodyframe = w_phi_b * v_z_b - w_psi_b * v_x_b + parms.grav * cos( theta ) * sin( phi );
ace_z_bodyframe = w_theta_b * v_x_b - w_phi_b * v_y_b + parms.grav * cos( theta ) * cos( phi ) - 1 / parms.mass * f;
ace_bodyframe   = [ ace_x_bodyframe ; ace_y_bodyframe ; ace_z_bodyframe ];

% Angular velocity in the body frame 

vel_phi = w_phi_b + sin( phi ) * tan( theta ) * w_theta_b + cos( phi ) * tan( theta ) * w_psi_b;
vel_theta =                                  cos( phi ) * w_theta_b              - sin( phi ) * w_psi_b;
vel_psi =                     sin( phi ) / cos( theta ) * w_theta_b + cos( phi ) / cos( theta ) * w_psi_b;
ang_vel = [ vel_phi ; vel_theta ; vel_psi ];
 
% Angular acelaration in the body frame 

ace_phi_bodyframe = (parms.inertia_y - parms.inertia_z) / parms.inertia_x * w_theta_b * w_psi_b + 1 / parms.inertia_x * tor_phi;
ace_theta_bodyframe = (parms.inertia_z - parms.inertia_x) / parms.inertia_y * w_phi_b * w_psi_b + 1 / parms.inertia_y * tor_theta;
ace_psi_bodyframe = (parms.inertia_x - parms.inertia_y) / parms.inertia_z * w_phi_b * w_theta_b + 1 / parms.inertia_z * tor_psi;
ang_ace_bodyframe = [ ace_phi_bodyframe ; ace_theta_bodyframe ; ace_psi_bodyframe ];

%% State-space representation %%

% Define state, input, and output

x     = [ pos    ; v_b ; ang_pos ; w_b ];
u     = [ f ; tor_phi       ; tor_theta ; tor_psi ];
y    = [ pos    ; psi ];

% Define state equation

f_x_u  = [ vel    ; ace_bodyframe ; ang_vel ; ang_ace_bodyframe ];
