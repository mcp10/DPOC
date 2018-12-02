%check differences in the results obtained with the three methods

[ J_opt_VI, u_opt_ind_VI ] = ValueIteration( P, G );

[ J_opt_PI, u_opt_ind_PI ] = PolicyIteration( P, G );

[ J_opt_LP, u_opt_ind_LP ] = LinearProgramming( P, G )

J_opt_VI_PI = norm(J_opt_VI - J_opt_PI)
u_opt_difference_VI_PI = norm(u_opt_ind_VI - u_opt_ind_PI)

J_opt_VI_LP = norm(J_opt_VI - J_opt_LP)
u_opt_difference_VI_LP = norm(u_opt_ind_VI - u_opt_ind_LP)

J_opt_LP_PI = norm(J_opt_LP - J_opt_PI)
u_opt_difference_LP_PI = norm(u_opt_ind_LP - u_opt_ind_PI)
