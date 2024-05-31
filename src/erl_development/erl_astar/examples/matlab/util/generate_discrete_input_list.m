function ulist = generate_discrete_input_list(u_vals,nd)
% ulist = generate_dicrete_input_list(u_vals,nd)
%
% gets points within the Linf norm unit ball
% scales the corresponding vectors by u_vals
% returns the list of vectors
%
% @INPUT:
%   u_vals = list of vector norms for the output controls 
%            (e.g. u_vals = [1;2])
%   nd = number of dimensions (e.g., nd = 3)
%
% @OUTPUT:
%   ulist = num_u x nd = list of vectors
%

if(nargin < 2)
    nd = 3;
end
         
u_res = get_Linf_ball__(1,nd);
u_res( ceil(size(u_res,1)/2), :) = []; % remove origin
u_res_norm = sqrt(sum(u_res.^2,2));   % compute norm
u_res = bsxfun(@rdivide, u_res, u_res_norm); % normalize

num_u_res = size(u_res,1);

u_vals = transpose(u_vals(:));
u_vals = u_vals(ones(num_u_res,1),:);

ulist = [zeros(1,nd); reshape(bsxfun(@times, u_vals, reshape(u_res,num_u_res,1,[])),[],nd)];

end


function vals = get_Linf_ball__(rad,nd)
% Generates a list of voxels (tuples) that comprise an L-inf
% ball of given radius.
%
% @param radius: the radius value to use (inclusive; discrete)
% @nd: number of dimensions (default: 3)
% @vals: [] x nd = list of tuples (discrete coordinates)
%
if( nargin < 2 )
    nd = 3;
end

lin = transpose(-rad:rad);
num_lin = length(lin);
dim_vec = transpose(1:nd);

vals = zeros(num_lin^nd,nd);
A = reshape(lin(:,ones(num_lin^(nd-1),1)),num_lin*ones(1,nd));
vals(:,1) = A(:);
for dim = 2:nd
    vals(:,dim) = reshape(permute(A,circshift(dim_vec,[dim-1,0])),[],1);
end

end
