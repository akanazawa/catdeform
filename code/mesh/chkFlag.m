function t = chkFlag(name)
t = ~isempty(whos('global',name));
end

