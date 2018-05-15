function [entropy] = totalEntropy(Sk , del_uk, param)
    % Calculation of expectency over trajectory 
    
    % Normalization of cost function 
%     Sk = Sk./sum(Sk);


    n = length(Sk);
    lambda = param.lambda;
    sum1 = 0;
    sum2 = 0;
    for i = 1:n
        sum1 = sum1 + exp(-(1/lambda)*Sk(i))*del_uk(i);
        sum2 = sum2 + exp(-(1/lambda)*Sk(i));
    end
    entropy = sum1/sum2;

end