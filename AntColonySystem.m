function [cost, bestValues] = AntColonySystem(iniCoords, goalCoord, iniSol, paths, cellAdjacencies, startingAdjacencies)
    % final return value is a set of hValues equal to number of paths
    % hValues = rand(nnz(paths),1);
    
    % initialize ACO parameters
    numAnts = 10;        % number of ants
    maxIteration = 120;   % max number of iterations
    numChoices = 20;     % number of choices to take towards next path
    spread = 0.8;       % degree of spread from initial solution
    layers = nnz(paths);% number of nodes each ant needs to travel
    phmone = ones(numChoices, layers) * (0.3) .* rand(1);
    ph_o = 0.05;        % pheromone deposition for local update
    p = 0.8;            % pheromone delay param
    beta = 1;           % relative param phmone/visibility
    r_o = 0.5;          % probability exploration param    
    
    % possible hValue solution matrix on each edge
    pathMatrix = zeros(numChoices, layers);
    % index of hValues taken by all ants
    % for updating of pheromone
    hIndex = zeros(numAnts, layers);
    hValues = zeros(numAnts, layers);
    bestValues = zeros(1, layers);
    for i=1:layers
        firstPath = iniSol(i);                  % point from initial sol
        lob = max(0.01, firstPath - spread);    % lower bound
        upb = min(0.99, firstPath + spread);    % upper bound
        pathMatrix(:,i) = lob + (upb-lob) .* rand(numChoices, 1);
    end
    
    %phmone
    %pathMatrix
    currSol = iniSol;
    for it = 1 : maxIteration
        for antNum = 1: numAnts
            % 3 stages
            % first stage, from iniCoord to path(1)
            
            currPos = iniCoords(1,:);
            fromCell = startingAdjacencies(1);
            toCell = paths(1);
            [boundary, distVector] = calcDistanceVector(numChoices, currPos, pathMatrix(:,1), cellAdjacencies, fromCell, toCell);    
            rouletteVec = calcRoulette(numChoices, distVector, phmone(:,1), beta);
        
            if(rand()<r_o)
                % choose best local solution
                [maxProb, maxIndex] = max(rouletteVec);
            else
                % choose a random solution
                maxIndex = ceil(rand()*numChoices);
            end

            chosen = pathMatrix(maxIndex, 1);
            hIndex(antNum, 1) = maxIndex;
            hValues(antNum, 1) = chosen;
            currPos = updateCurrPos(boundary, chosen);
            
            % local pheromone update
            phmone(maxIndex,1) = (1-p)*phmone(maxIndex,1) + p*ph_o;

            % stage 2, from path(1) to path(end)
            for j = 2:size(paths, 2)
                % roulette wheel method - closest distance
                fromCell = paths(j-1);
                toCell = paths(j);
                [boundary, distVector] = calcDistanceVector(numChoices, currPos, pathMatrix(:,j), cellAdjacencies, fromCell, toCell);
                rouletteVec = calcRoulette(numChoices, distVector, phmone(:,j), beta);
                
                if(rand()<r_o)
                    [maxProb, maxIndex] = max(rouletteVec);
                else
                    maxIndex = ceil(rand()*numChoices);
                end
                
                chosen = pathMatrix(maxIndex, j);
                hIndex(antNum, j) = maxIndex;
                hValues(antNum, j) = chosen;
                currPos = updateCurrPos(boundary, chosen);
                
                % local pheromone update
                phmone(maxIndex,j) = (1-p)*phmone(maxIndex,j) + p*ph_o;
            end
        end    % end ant
        
        % getting result from best ant
        totCost = zeros(1, numAnts);
        for k = 1:numAnts
            totCost(k) = GetTotalPathCost(hValues(k,:), paths, cellAdjacencies, startingAdjacencies, iniCoords, goalCoord);
        end
        
        [bestCost, bestAntNum] = min(totCost);
        pathIndex = hIndex(bestAntNum,:);

        % pheromone reinforcement on iteration best route
        phmone = (1-p).* phmone;
        for l=1:layers
            phmone( pathIndex(l), l) = p * 1/bestCost;
        end
        
        cost = bestCost;
        bestValues = hValues(bestAntNum,:);
        currSol = bestValues;
    end    % end iteration
    
    bestValues
end

function [rouletteVec] = calcRoulette(numChoices, distVector, phmoneVal, beta)
    for pathNum = 1:numChoices
        rouletteVec(pathNum) = calcProbability(pathNum, distVector, phmoneVal, beta);
    end
end

function [probability] = calcProbability(pathNum, distVec, phmoneVal, phmoneParam)
    numer = phmoneVal(pathNum)^phmoneParam / distVec(pathNum);
    denom = sum( phmoneVal .^phmoneParam ./ distVec );
    probability = numer / denom;
end

function [boundary, distVec] = calcDistanceVector(numChoices, fromCoord, nodeHeights, cellAdjacencies, fromCell, toCell)
    boundary = cellAdjacencies(:, fromCell, toCell);
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

function [pos] = updateCurrPos(boundary, chosen)
    pos(1) = boundary(1);
    pos(2) = boundary(2) + (boundary(3) - boundary(2)) * chosen;
end