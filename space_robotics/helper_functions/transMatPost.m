function [ transMatTotal ] = transMatPost(transMats)
% post multiply transformation matrices in a cell array
transMatTotal = transMats{1};
for ii = 2:numel(transMats)
    transMatTotal =  transMatTotal * transMats{ii};
end

end

