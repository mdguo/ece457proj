function [hValues] = AntColonySystem(iniCoords, iniSol, paths, cellAdjacencies, startingAdjacencies)
    % final return value is a set of hValues equal to number of paths
    % hValues = rand(nnz(paths),1);
    
    % initialize ACO parameters
    numAnts = 1;        % number of ants
    maxIteration = 5;   % max number of iterations
    numChoices = 5;     % number of choices to take towards next path
    spread = 0.5;       % degree of spread from initial solution
    layers = nnz(paths);% number of nodes each ant needs to travel
    phmone = ones(numChoices, layers) * (0.1) .* rand(1);
    decay = 0.7;        % pheromone delay param
    
    % generate possible solution matrix on each edge
    pathVector = zeros(numChoices, layers);
    for i=1:layers
        firstPath = iniSol(i);                  % point from initial sol
        lob = max(0.01, firstPath - spread);    % lower bound
        upb = min(0.99, firstPath + spread);    % upper bound
        pathVector(:,i) = lob + (upb-lob) .* rand(numChoices, 1);
    end
    
    phmone
    pathVector
    
    decisionVector = calculateDecisonVector(iniCoords(1), pathVector(:,1), phmone(:,1));
    
end

function [decisionVec] = calculateDecisionVector(iniCoords, nodeHeights, phmone)
    from = iniCoords(1,:);
    to = 
end

function [distance] = calcDistance(p1, p2)
    distance = sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
end

