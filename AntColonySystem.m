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
    alpha = 1;          % distance param
    beta = 1;           % pheromone param
    
    hValues = zeros(1, layers);
    
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
    
    currPos = iniCoords(1,:);
    
    [boundary, distVector] = calculateDistanceVector(numChoices, currPos, pathVector(:,1), paths, cellAdjacencies, startingAdjacencies);
    boundary
    distVector
    
    decisionVec = zeros(numChoices, 1);
    for pathNum = 1:numChoices
        decisionVec(pathNum) = calcProbability(pathNum, distVector, alpha, phmone(:,1), beta);
    end
    
    decisionVec
    
    % choose a node to take and update the current position
    [maxProb, maxIndex] = max(decisionVec);
    % should I implement generating a random value and check if maxProb>r ?
    
    chosen = pathVector(maxIndex, 1);
    hValues(1) = chosen;
    currPos = updateCurrPos(boundary, chosen);
    
    
end

function[pos] = updateCurrPos(boundary, chosen)
    pos(1) = boundary(1);
    pos(2) = boundary(2) + (boundary(3) - boundary(2)) * chosen;
end

function [boundary, distVec] = calculateDistanceVector(numChoices, fromCoord, nodeHeights, paths, cellAdjacencies, startingAdjacencies)
    boundary = cellAdjacencies(:,startingAdjacencies(1),paths(1));
    from = fromCoord;
    distVec = zeros(numChoices, 1);
    
    for hIndex=1:numChoices
        to = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*nodeHeights(hIndex)];
        distVec(hIndex) = calcDistance(from, to);
    end
end

function [distance] = calcDistance(p1, p2)
    distance = sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
end

function [probability] = calcProbability(pathNum, distVec, distParam, phmoneVal, phmoneParam)
    numer = phmoneVal(pathNum)^phmoneParam / distVec(pathNum)^distParam;
    denom = sum( phmoneVal .^phmoneParam ./ distVec .^distParam );
    probability = numer / denom;
end
