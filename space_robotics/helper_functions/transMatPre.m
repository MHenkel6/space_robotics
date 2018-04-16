function [ transMatTotal ] = transMatPre(transMats)
% pre-multiply transformation matrices in a cell array
transMatTotal = transMats{1};
for ii = 2:numel(transMats)
    transMatTotal = transMats{ii} * transMatTotal;
end

end

