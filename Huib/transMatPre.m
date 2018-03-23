function [ transMatTotal ] = transMatPre(transMats)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
transMatTotal = transMats{1};
for ii = 2:numel(transMats)
    transMatTotal = transMats{ii} * transMatTotal;
end

end

