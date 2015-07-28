function [ minimum, fval, total ] = GenerateSaSolution( hValues, paths,cellAdjacencies,startingAdjacencies,startingCoords,goalCoord, initCost )
    cool = @(T) (.90*T);
    %newSol = @()  rand(nnz(paths),1);
    InitTemp = 1000;
    StopTemp = 1e-8;
    MaxTries = 4000;
    MaxSuccess = 20;
    MaxCosecRejects = 1000;
    unoptimizedHValues = hValues;
    k = 1; %Boltzman's Constant
    y = 1000; %Energy Difference Constant 
    
    iterations = 0;
    total = 0;
    finished = 0;
    success = 0;
    consec = 0;
    T = InitTemp;
    oldEnergy = GetTotalPathCost(hValues, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    current = unoptimizedHValues;
    prevSol = unoptimizedHValues;

    while ~finished;
        iterations = iterations+1;
        
        if iterations >= MaxTries || success >= MaxSuccess || consec >= MaxCosecRejects;
            if T < StopTemp || consec >= MaxCosecRejects;
                finished = 1;
                total = total + iterations;
                break;
            else
                T = cool(T);
                total = total + iterations;
                iterations = 0;
                success = 0;
            end
        end
        
        current = newSol(current);
        newEnergy = GetTotalPathCost(current, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
        if (newEnergy < oldEnergy);
            prevSol = current;
            oldEnergy = newEnergy;
            success = success + 1;
            cosec = 0;
        else
            if (rand < exp((oldEnergy - newEnergy)/(k*T)));
               prevSol = current;
               oldEnergy = newEnergy;
               success = success + 1;
               cosec = 0;
            else
                cosec = cosec + 1;
            end
        end
        
    end

    minimum = prevSol;
    fval = oldEnergy;

end

function [newVal] = newSol(hValues)
    value_count = numel(hValues);
    newVal = hValues;
    i = randi(value_count);
    newVal(i) = rand;
end