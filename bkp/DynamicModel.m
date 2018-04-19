function x_dot = DynamicModel(t, x ,u, dW)

	% x - current state of the system
	% dW - uncertainty (modeled as normal distribution)
	% u - current input

	% System parameters
	m = .5;  % pendulum mass (kg)
	M = 1;   % car mass  (kg)
	g = 9.81;   % gravity force (m/s^2)
	l = 0.5;    % pendulum length (m)


	lin_acc = (l*m*sin(x(3))*x(4)^2 + u(1)  - g*m*cos(x(3))*sin(x(3)))/(M + m*(sin(x(3))^2));
	angular_acc = (-m*l*x(4)^2*sin(x(3))*cos(x(3))-u(1)*cos(x(3))+(g*sin(x(3)))*(m+M))/((sin(x(3))^2)*m*l + M*l);

	x_dot = [x(2) + dW(1);
	         lin_acc + dW(2);
	         x(4) + dW(3);
	         angular_acc + dW(4)]; 
end