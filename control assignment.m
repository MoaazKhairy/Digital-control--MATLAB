% Asseignment Control 
% Name: Moaz Khairy Hussein 
% sec : 2
% BN : 22

% define given state space representation
G = [-0.64 0.9 0.58 ; -0.32 0.2 3.04 ; -0.32 0.2 1.04]
H = [2 ; 1 ; 1]
C = [2 0.2 -2.2]
D = []
sampling_period = 0.1;

%% request (a) 
% get Controllability matrix and inverse
Cont_Matrix = [H G*H G*G*H];
Cont_Matrix_inverse = inv(Cont_Matrix);
f3 = Cont_Matrix_inverse(3 , :) ;  % last row in inverse controllability matrix 
fprintf('get P and P_inverse : \n')
P_inverse = [ f3 ; f3*G ; f3*G*G ]   %required
P = inv(P_inverse)                   %required
G_hat = P_inverse * G * P ;
H_hat = P_inverse * H ;
C_hat = C * P ;
D_hat = D ;
% get standard controllable form
sys_ss = ss(G_hat , H_hat , C_hat , D_hat , sampling_period)
fprintf('Open Loop pulse Transfer Function : \n')
sys_tf = tf(sys_ss)  %open loop Transfer Function (required)
[open_loop_tf_num , open_loop_tf_den] = tfdata(sys_tf , 'v')

%% request (b)
poles = [ 0.3 0.4+0.5i 0.4-0.5i ];
closed_loop_charac_eq_coeff = poly(poles)
syms z
closed_loop_charac_eq = poly2sym(closed_loop_charac_eq_coeff , z)
K_hat = place(G_hat , H_hat , poles)
fprintf('get K matrix : \n')
K = K_hat * P_inverse

%% request (c)
%get closed loop pulse transfer function
closed_loop_tf_num = open_loop_tf_num ;
closed_loop_tf_den = closed_loop_charac_eq_coeff ;
fprintf('closed Loop pulse Transfer Function : \n')
closed_loop_tf = tf(closed_loop_tf_num , closed_loop_tf_den , sampling_period)  %required
%get step response and plot it
ss_value = dcgain(closed_loop_tf)
new_closed_loop_tf = closed_loop_tf / ss_value % to ensure steady state value =  1
step(new_closed_loop_tf)   %required
ss_value_new = dcgain(new_closed_loop_tf)  %required
%get step-response characteristics
step_info = stepinfo(new_closed_loop_tf)  %required

%% request (d) repeat (b) and (c) again
%choice one
poles_1= [ 0.1 0.2 0.3 ] ;
closed_loop_charac_eq_coeff_1 = poly(poles_1)
syms z
closed_loop_charac_eq_1 = poly2sym(closed_loop_charac_eq_coeff_1 , z)
K_hat_1 = place(G_hat , H_hat , poles_1)
K_1 = K_hat_1 * P_inverse
%get closed loop pulse transfer function
closed_loop_tf_num_1 = open_loop_tf_num ;
closed_loop_tf_den_1 = closed_loop_charac_eq_coeff_1 ;
closed_loop_tf_1 = tf(closed_loop_tf_num_1 , closed_loop_tf_den_1 , sampling_period)  %required
%get step response and plot it
ss_value_1 = dcgain(closed_loop_tf_1)
new_closed_loop_tf_1 = closed_loop_tf_1 / ss_value_1 % to ensure steady state value =  1
ss_value_new_1 = dcgain(new_closed_loop_tf_1)  %required
%get step-response characteristics
step_info_1 = stepinfo(new_closed_loop_tf_1)  %required

%choice two
poles_2= [ 0.3 0.6 0.41 ] ;
closed_loop_charac_eq_coeff_2 = poly(poles_2)
syms z
closed_loop_charac_eq_2 = poly2sym(closed_loop_charac_eq_coeff_2 , z)
K_hat_2 = place(G_hat , H_hat , poles_2)
K_2 = K_hat_2 * P_inverse
%get closed loop pulse transfer function
closed_loop_tf_num_2 = open_loop_tf_num ;
closed_loop_tf_den_2 = closed_loop_charac_eq_coeff_2 ;
closed_loop_tf_2 = tf(closed_loop_tf_num_2 , closed_loop_tf_den_2 , sampling_period)  %required
%get step response and plot it
ss_value_2 = dcgain(closed_loop_tf_2)
new_closed_loop_tf_2 = closed_loop_tf_2 / ss_value_2 % to ensure steady state value =  1
ss_value_new_2 = dcgain(new_closed_loop_tf_2)  %required
%get step-response characteristics
step_info_2 = stepinfo(new_closed_loop_tf_2)  %required

%choice three
poles_3= [ 0.5 0.2+0.5i 0.2-0.5i ] ;
closed_loop_charac_eq_coeff_3 = poly(poles_3)
syms z
closed_loop_charac_eq_3 = poly2sym(closed_loop_charac_eq_coeff_3 , z)
K_hat_3 = place(G_hat , H_hat , poles_3)
K_3 = K_hat_3 * P_inverse
%get closed loop pulse transfer function
closed_loop_tf_num_3 = open_loop_tf_num ;
closed_loop_tf_den_3 = closed_loop_charac_eq_coeff_3 ;
closed_loop_tf_3 = tf(closed_loop_tf_num_3 , closed_loop_tf_den_3 , sampling_period)  %required
%get step response and plot it
ss_value_3 = dcgain(closed_loop_tf_3)
new_closed_loop_tf_3 = closed_loop_tf_3 / ss_value_3 % to ensure steady state value =  1
ss_value_new_3 = dcgain(new_closed_loop_tf_3)  %required
%get step-response characteristics
step_info_3 = stepinfo(new_closed_loop_tf_3)  %required

%choice four
poles_4= [ 0.36 0.5+0.3i 0.5-0.3i ] ;
closed_loop_charac_eq_coeff_4 = poly(poles_4)
syms z
closed_loop_charac_eq_4 = poly2sym(closed_loop_charac_eq_coeff_4 , z)
K_hat_4 = place(G_hat , H_hat , poles_4)
K_4 = K_hat_4 * P_inverse
%get closed loop pulse transfer function
closed_loop_tf_num_4 = open_loop_tf_num ;
closed_loop_tf_den_4 = closed_loop_charac_eq_coeff_4 ;
closed_loop_tf_4 = tf(closed_loop_tf_num_4 , closed_loop_tf_den_4 , sampling_period)  %required
%get step response and plot it
ss_value_4 = dcgain(closed_loop_tf_4)
new_closed_loop_tf_4 = closed_loop_tf_4 / ss_value_4 % to ensure steady state value =  1
ss_value_new_4 = dcgain(new_closed_loop_tf_4)  %required
%get step-response characteristics
step_info_4 = stepinfo(new_closed_loop_tf_4)  %required

%choice five
poles_5= [ 0.8 0.6+0.4i 0.6-0.4i ] ;
closed_loop_charac_eq_coeff_5 = poly(poles_5)
syms z
closed_loop_charac_eq_5 = poly2sym(closed_loop_charac_eq_coeff_5 , z)
K_hat_5 = place(G_hat , H_hat , poles_5)
K_5 = K_hat_5 * P_inverse
%get closed loop pulse transfer function
closed_loop_tf_num_5 = open_loop_tf_num ;
closed_loop_tf_den_5 = closed_loop_charac_eq_coeff_5 ;
closed_loop_tf_5 = tf(closed_loop_tf_num_5 , closed_loop_tf_den_5 , sampling_period)  %required
%get step response and plot it
ss_value_5 = dcgain(closed_loop_tf_5)
new_closed_loop_tf_5 = closed_loop_tf_5 / ss_value_5 % to ensure steady state value =  1
ss_value_new_5 = dcgain(new_closed_loop_tf_5)  %required
%get step-response characteristics
step_info_5 = stepinfo(new_closed_loop_tf_5)  %required

% draw five step responses on single graph with different colors
step(new_closed_loop_tf_1 , new_closed_loop_tf_2 , new_closed_loop_tf_3 , new_closed_loop_tf_4 , new_closed_loop_tf_5)
legend('System 1','Sysem 2','System 3','System 4','System 5')
