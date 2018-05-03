%------------------------------------------------------------------------------------------------------%
% Author: Rajnish Tiwari, Vishnu Rudrasamudram, Ameya Wagh
% Description: entropy function to be added to input sequence
%------------------------------------------------------------------------------------------------------%
function [entropy] = totalEntropy(Sk , del_uk)

    n = length(Sk);
    lambda = 10;
    sum1 = 0;
    sum2 = 0;
    for i = 1:n
        sum1 = sum1 + exp(-(1/lambda)*Sk(i))*del_uk(i);
        sum2 = sum2 + exp(-(1/lambda)*Sk(i));
    end
    entropy = sum1/sum2;

end