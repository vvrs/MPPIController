function cost = CostFunction_car(X,V,v_desired)
	% X - from GetState
	% V - from CarDynamicModel
	d = abs((X(1)/13)^2 + (X(2)/6)^2 - 1);

	cost = 100*d^2 + (V(1)-v_desired)^2;
end