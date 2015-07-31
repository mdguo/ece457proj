function [ minimum, fval, total, costGraph ] = GenerateSaSolution( hValues, paths,cellAdjacencies,startingAdjacencies,startingCoords,goalCoord, initCost )

def = struct(...
        'CoolSched',@(T) (.8*T),...
        'Generator',@(X)  rand(numel(X),1),...
        'InitTemp',5,...
        'MaxConsRej',100*numel(hValues),...
        'MaxSuccess',2*numel(hValues),...
        'MaxTries',5*numel(hValues),...
        'StopTemp',1e-4,...
        'StopVal',-Inf,...
        'Verbosity',0);
    
    fs = {'CoolSched','Generator','InitTemp','MaxConsRej',...
        'MaxSuccess','MaxTries','StopTemp','StopVal','Verbosity'};
    for nm=1:length(fs)
        options.(fs{nm}) = def.(fs{nm});
    end 
    
% main settings
loss = @(X) GetTotalPathCost(X, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
parent = hValues;
newsol = options.Generator;      % neighborhood space function
Tinit = options.InitTemp;        % initial temp
minT = options.StopTemp;         % stopping temp
cool = options.CoolSched;        % annealing schedule
minF = options.StopVal;
max_consec_rejections = options.MaxConsRej;
max_try = options.MaxTries;
max_success = options.MaxSuccess;
report = options.Verbosity;
k = 1;                           % boltzmann constant

% counters etc
itry = 0;
success = 0;
finished = 0;
consec = 0;
T = Tinit;
initenergy = loss(parent);
oldenergy = initenergy;
total = 0;
if report==2, fprintf(1,'\n  T = %7.5f, loss = %10.5f\n',T,oldenergy); end

while ~finished;
    itry = itry+1; % just an iteration counter
    total = total + 1;
    costGraph(total)=oldenergy;
    current = parent; 
    
    % % Stop / decrement T criteria
    if itry >= max_try || success >= max_success;
        if T < minT || consec >= max_consec_rejections;
            finished = 1;
            %total = total + itry;
            break;
        else
            T = cool(T);  % decrease T according to cooling schedule
            max_try = max_try * 1.1; % Increases number of tries as temperature cools
            if report==2, % output
                fprintf(1,'  T = %7.5f, loss = %10.5f\n',T,oldenergy);
            end
            %total = total + itry;
            itry = 1;
            success = 1;
        end
    end
    
    newparam = newSol(current);
    %newparam = newsol(current);
    newenergy = loss(newparam);
    
    if (newenergy < minF),
        parent = newparam; 
        oldenergy = newenergy;
        break
    end
    
    if (oldenergy-newenergy > 1e-6)
        parent = newparam;
        oldenergy = newenergy;
        success = success+1;
        consec = 0;
    else
        if (rand < exp( (oldenergy-newenergy)/(k*T) ));
            parent = newparam;
            oldenergy = newenergy;
            success = success+1;
        else
            consec = consec+1;
        end
    end
end

minimum = parent;
fval = oldenergy;
figure
plot(costGraph)
    
end

function [newVal] = newSol(hValues)
    value_count = numel(hValues);
    newVal = hValues;
    i = randi(value_count);
    newVal(i) = rand;
end