classdef clientBufferState
    properties (SetAccess = public)
        o = 0 
        l = 0 
        t = 1
        v = []
        tt = 0;
        st = 0;
        state = 0;
    end
    methods 
        function CB = clientBufferState(o,l,t,v,tt)
            CB.o = o;
            CB.l = l;
            CB.t = t;
            CB.v = v;
            CB.tt = tt;
        end
        function CB = segmentin(CB, tt, state)
            %CB.o = CB.o + CB.t;
            CB.o = CB.t*max(sum(state>0,2));
            CB.l = CB.l + 1;
            %CB.v(CB.l) = rate;
            CB.tt = tt;
        end
        function CB = segmentout(CB, tt)
            if CB.o>0, CB.o = CB.o - CB.t; 
            else display('Empty buffer'); CB.st = CB.st + 1;
            end
            if CB.l>0, CB.l = CB.l - 1; end
            %temp = CB.v(2:CB.l+1);
            %CB.v = 0;
            %CB.v(1:CB.l) = temp;
            CB.tt = tt;
        end
    end
end