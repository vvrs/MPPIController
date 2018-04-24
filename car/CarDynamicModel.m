function X_dot = CarModel(t,x,u)

	% State (x)
	% vx,vy,phi_dot(angular velocity)

	vx = x(1);
	vy = x(2);
	phi_dot = x(3);

	% % input (u)
	% % Frx, delta	
	Frx = u(1);
	delta = u(2);

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Autonomous Drift Cornering with Mixed Open-loop and Closed-loop Control
	% F.Zhang ∗ J. Gonzales ∗∗ K.Li ∗ F. Borrelli ∗∗

	% Parameters

	m = 1.95;
	Iz = 0.24;
	lf = 0.125;
	lr = 0.125;
	Cf = 1.76;
	Cr = 1.76;

	alpha_f = delta - ((vy + lf*phi_dot)/vx);
	alpha_r = (lr*phi_dot - vy)/vx;

	Fyf = Cf*alpha_f;
	Fyr = Cr*alpha_r;

	vx_dot = (1/m)*Frx;
	vy_dot = (1/m)*(Fyf + Fyr) - vx*phi_dot;
	phi_dot_dot = (1/Iz)*(lf*Fyf - lr*Fyr);

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% From ETH Thesis
	% % parameters
	% theta_z = 2.78*10^-5 % kgm^2
	% m = 40.1
	% lr = 0.029
	% lf = 0.033

	% Bf = 4.1
	% Cf = 1.1
	% Df = 0.22

	% Br = 3.8609
	% Cr = 1.4
	% Dr = 0.1643

	% alpha_f = -atan((phi_dot*lf + vy)/vx) + delta;
	% alpha_r = atan((phi_dot*lr-vy)/vx);

	% Ffy = Df*sin(Cf*atan(Bf*alpha_f));
	% Fry = Dr*sin(Cr*atan(Br*alpha_r));

	% vx_dot = (1/m)*(Frx - Ffy*sin(delta)+m*vy*phi_dot);
	% vy_dot = (1/m)*(Fry - Ffx*cos(delta)-m*vx*phi_dot);
	% phi_dot_dot = (1/theta_z)*(Ffy*lf*cos(delta)-Fry*lr);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% return
	X_dot = [vx_dot;
			 vy_dot;
			 phi_dot_dot];




end
