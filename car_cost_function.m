%------------------------------------------------------------------------------------------------------%
% Author: Rajnish Tiwari, Vishnu Rudrasamudram, Ameya Wagh
% Description: cost function for car using projected distance on path.
%------------------------------------------------------------------------------------------------------%
function cost = car_cost_function(x,v)
	d = abs((x(1)/13)^2 + (x(2)/6)^2-1);
	cost = 100*d^2+(v-7)^2;
end