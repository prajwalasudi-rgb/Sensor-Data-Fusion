firstparam = [1, 2, 3.3, 3.7, 8, 21];  %list of places to search for first parameter
secondparam = linspace(0,1,20);        %list of places to search for second parameter
[F,S] = ndgrid(firstparam, secondparam);
fitresult = arrayfun(@(p1,p2) fittingfunction(p1,p2), F, S); %run a fitting on every pair fittingfunction(F(J,K), S(J,K))
[minval, minidx] = min(fitresult);
bestFirst = F(minidx);
bestSecond = S(minidx);