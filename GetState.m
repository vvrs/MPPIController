%------------------------------------------------------------------------------------------------------%
% Author: Rajnish Tiwari, Vishnu Rudrasamudram, Ameya Wagh
% Description: function to fetch current states of the system
%------------------------------------------------------------------------------------------------------%
function X = GetState(velocity,current_state,dt)

	vx = velocity(1);
	vy = velocity(2);
	phi_dot = velocity(3);

	current_x = current_state(1);
	current_y = current_state(2);
	current_phi = current_state(3);

	phi = current_phi + phi_dot*dt;

	x_dot = vx*cos(phi) - vy*sin(phi);
	y_dot = vx*sin(phi) + vy*cos(phi);


	x = current_x + x_dot*dt;
	y = current_y + y_dot*dt;

	X = [x;
		 y;
		 phi]
end