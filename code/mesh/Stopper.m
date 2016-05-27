classdef Stopper
    properties
        t_start;
        t_end;
    end
    
    methods
        function obj=Stopper(varargin)
            obj.t_start = tic;
            if length(varargin)>=1,
                fprintf(varargin{:});
            end;
        end
        function t_end=stop(obj,varargin)
            t_end = toc(obj.t_start);
            obj.t_end = t_end;
            if length(varargin)==0,
                fprintf('(%.3g secs)\n',t_end);
            end;
        end
    end
end

