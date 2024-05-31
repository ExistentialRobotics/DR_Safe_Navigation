function [ datam ] = cells2meters( datac,dim_min,res)
%[ datam ] = cells2meters( datac,dim_min,res)
%   Detailed explanation goes here

if length(res) > 1
    datam = bsxfun(@plus,bsxfun(@times,(datac-0.5),res),dim_min);
else
    datam = bsxfun(@plus,(datac-0.5)*res,dim_min);
end

end