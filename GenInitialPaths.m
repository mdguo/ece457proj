% Author: William Mayo
%assumptions
%no overlapping obstacles (two obstacles cannot share a vertex on the same
%point
%no vertex on the edge of map

function [paths cellAdjacencies startingAdjacencies] = GenInitialPaths(fieldDimensions, startingCoords, goalCoord, obstacles)

    % sort all the obstical vertices by x position
    sortedVertices = [];
    for i = 1:size(obstacles,3)
        sortedVertices = [sortedVertices; obstacles(:,:,i)];
    end
    sortedVertices = sort(sortedVertices,1);
    
    % find all the vertical lines and add the last one
    verticalIntercepts = [0 sortedVertices(1,1)];
    for i = 2:size(sortedVertices, 1)
        if (verticalIntercepts(end) ~= sortedVertices(i,1) && ~isnan(sortedVertices(i,1)))
            verticalIntercepts = [verticalIntercepts sortedVertices(i,1)];
        end
    end
    verticalIntercepts = [verticalIntercepts fieldDimensions(1)];
    
    %Generate a list of all edges in obstacles
    obstacleEdges = [0 0 fieldDimensions(1) 0];
    for i = 1:size(obstacles,3)
        lastIndex = 1;
        
        for j = 2:size(obstacles,1)
            if isnan(obstacles(j,:,i))
                break;
            end
            lastIndex = lastIndex + 1;
            obstacleEdges = [obstacleEdges ; obstacles(j-1,:,i) obstacles(j,:,i)]; 
        end
        
        % make sure to include the edge that wraps around
        obstacleEdges = [obstacleEdges ; obstacles(1,:,i) obstacles(lastIndex,:,i)];
    end
    obstacleEdges = [obstacleEdges; 0 fieldDimensions(2) fieldDimensions(1) fieldDimensions(2)];
    
    %Sort edges by y coordinate of first point. Can do this since no two
    %edge can overlap with each other
    obstacleEdges = sortrows(obstacleEdges, [2 4]);
    
    %starting from the second vertical line, for each vertical line
    %interval move up vertically and add cells as boundaries are
    %encountered
    cellAdjacencies = zeros(3,4,4); %2 (y1,y2) by #cells by $cells
    startingAdjacencies = zeros(size(startingCoords, 3)) * NaN;
    finishingAdjacency = NaN;
    prevCellRanges = []; %y boundary ranges of previous vertical region's right boundary
    cellCount = 0;
    prevCellCount = 0;
    previousRegionCellCount = 0;
    
    for i = 2:size(verticalIntercepts,2)
        intA = verticalIntercepts(i-1);
        intB = verticalIntercepts(i);
        
        edgeFoundInRegion = 0;
        insideBoundary = 1; % we start off the map
        prevEdge = 0; %keeps track of the previous edge
        currentCellRanges = [];
        
        for j = 1:size(obstacleEdges,1)
            edge = obstacleEdges(j,:);
            
            %if this edge crosses the current vertical boundary region
            if ((edge(1) <= intA && edge(3) >= intB) || (edge(3) <= intA && edge(1) >= intB))        
                if (~insideBoundary) 
                    %this boundary to the previous is a cell region
                    cellCount = cellCount + 1; %cell count is the id of the current cell
                    
                    %add cell to adjacency matrix
                    %determine upper and lower boundaries

                    pLower = polyfit([prevEdge(1) prevEdge(3)],[prevEdge(2) prevEdge(4)],1);
                    pUpper = polyfit([edge(1) edge(3)],[edge(2) edge(4)],1);
                    
                    %store y1 and y2 intercepts in matrix
                    %cellAdjacencies(:,1,1) = [pLower(1)*intA+pLower(2); pUpper(1)*intA+pUpper(2)];
                    cellLeftMinY = pLower(1)*intA+pLower(2);
                    cellLeftMaxY = pUpper(1)*intA+pUpper(2);
                    cellRightMinY = pLower(1)*intB+pLower(2);
                    cellRightMaxY = pUpper(1)*intB+pUpper(2);
                    
                    %store the left ymin and ymax in the cellAdjacencies
                    %connected to itself
                    cellAdjacencies(:,cellCount,cellCount) = [intA cellLeftMinY cellLeftMaxY];
                    
                    %store the lower right edge v boundaries for next
                    %region processin
                    currentCellRanges = [currentCellRanges cellRightMinY cellRightMaxY];
                    
                    %determine if the starting or ending vertext is located
                    %in the current cell
                    for k=1:size(startingCoords,1)
                        if startingCoords(k,1) >= intA && startingCoords(k,1) <= intB && ...
                            startingCoords(k,2) >= pLower(1)*startingCoords(k,1)+pLower(2) && ...
                            startingCoords(k,2) <= pUpper(1)*startingCoords(k,1)+pUpper(2)
                            startingAdjacencies(k) = cellCount;
                        end
                    end
                    
                    %determine if finishing coordinate is in the current
                    %cell
                    if goalCoord(1) >= intA && goalCoord(1) <= intB && ...
                        goalCoord(2) >= pLower(1)*goalCoord(1)+pLower(2) && ...
                        goalCoord(2) <= pUpper(1)*goalCoord(1)+pUpper(2)
                        finishingAdjacency = cellCount;
                    end
                    
                    % find all connections from cells in previous region to
                    % current cell in this region
                    if (size(prevCellRanges,2) > 1)
                        for k=1:2:size(prevCellRanges,2)
                            range = [prevCellRanges(k) prevCellRanges(k+1)];
                            if cellLeftMinY <= range(2) && range(1) <= cellLeftMaxY
                                %add edge to graph
                                cellA = prevCellCount-previousRegionCellCount + 1 + floor(k/2);
                                cellB = cellCount;
                                yMin = max([cellLeftMinY range(1)]);
                                yMax = min([cellLeftMaxY range(2)]);
                                
                                cellAdjacencies(:,cellA,cellB) = [intA yMin yMax];
                                cellAdjacencies(:,cellB,cellA) = [intA yMin yMax];
                                %disp(['connection betwwen ', num2str(cellA), ' and ', num2str(cellB), ' over ', num2str(yMin), ' ', num2str(yMax)]);
                            end
                        end
                    end
                end
                
                prevEdge = edge;
                insideBoundary = ~insideBoundary;
            end
        end
        previousRegionCellCount = cellCount - prevCellCount;
        prevCellCount = cellCount;
        prevCellRanges = currentCellRanges;
    end
    
    %useful variables to debug with
    %cellCount
    %cellAdjacencies
    %finishingAdjacency
    %startingAdjacencies
    
    %because of the sub-optimality of Dijkstra's. Instead of running it
    %once on the goal we run it once from each starting position
    paths = zeros(1,cellCount); %max length of a path is all the cells though likely less
    
    for i = 1:size(startingAdjacencies,2)

        [previous] = Dijkstra(cellAdjacencies, finishingAdjacency, goalCoord, startingAdjacencies(i), startingCoords(i,:));
        nextPath = getPathFromPreviousDistanceVectors(previous, finishingAdjacency);
         
        if size(nextPath,2) < cellCount
            temp = zeros(1,cellCount);
            temp(1,1:size(nextPath,2)) = nextPath;
            nextPath = temp;
        end
        
        paths(i,:) = nextPath;
    end
    
    %more useful debug vars
    %previous
    %distance
    %startingPosDistance
    %startingPosPrevious 
    
    
end

function [previous goalPosPrevious] = Dijkstra(cellAdjacencies, finishingAdjacency, goalCoord, startingAdjacency, startingCoords)
    nCells = size(cellAdjacencies, 3);

    goalPosDistance = Inf;
    goalPosPrevious = NaN;
    distance = Inf(1, nCells);
    previous = NaN(1, nCells);
    visited = ones(1, nCells); %keeps track of nodes already visited
    
    %Start by finding the distance from the goalCoord in finishingAdjacency
    %to all other neighbors. Can do this because it is a leaf
    row = cellAdjacencies(:,:,startingAdjacency)';
    for j = 1:nCells
        if j ~= startingAdjacency && row(j,3)-row(j,2) > 0
            from = startingCoords;
            to = [row(j,1), (row(j,2) + (row(j,3)-row(j,2))/2)];

            distance(j) = calcDistance(from, to);
            previous(j) = startingAdjacency;
        end
    end
    
    visited(startingAdjacency) = Inf;
    %now run dijkstra on the whole graph
    
    for i = 1:nCells-1
        [minV minI] = min(distance.*visited);
        visited(minI) = Inf; %only visit each node once
        
        %for each adjacent node
        row = cellAdjacencies(:,:,minI)';
        for j = 1:nCells 
            if j ~= startingAdjacency && j ~= minI && j ~= previous(minI) && ...
                    row(j,3)-row(j,2) > 0
                
                from = [];
                to = [];
                commingFrom = previous(minI); %the cell in the path before minI
                
                if row(j,1) == cellAdjacencies(1,commingFrom,commingFrom)
                    %if the path goes left then right or right then left
                    %(move back into same verticle cell region)
                    from = [row(j,1), row(j,2) + (row(j,3)-row(j,2))/2];
                    to = [row(commingFrom,1), row(commingFrom,2) + (row(commingFrom,3)-row(commingFrom,2))/2];
                elseif row(minI,1) < row(j,1)
                    from = [row(commingFrom,1), row(commingFrom,2) + (row(commingFrom,3)-row(commingFrom,2))/2];
                    to = [row(j,1), row(j,2) + (row(j,3)-row(j,2))/2];
                else
                    from = [row(commingFrom,1), row(commingFrom,2) + (row(commingFrom,3)-row(commingFrom,2))/2];
                    to = [row(j,1), row(j,2) + (row(j,3)-row(j,2))/2];
                end

                %disp(['from ', num2str(minI), ' to ', num2str(j), ' distance ', num2str(calcDistance(from, to))]);

                alt = distance(minI) + calcDistance(from, to);
                
                if alt < distance(j)
                    distance(j) = alt;
                    previous(j) = minI;
                end
                
                %if ending point is in this cell determin its distance
                if j == finishingAdjacency
                    from = [row(j,1), row(j,2) + (row(j,3)-row(j,2))/2];
                    to = goalCoord;
                    
                    goalPointAlt = alt + calcDistance(from, to);
                    
                    if goalPointAlt < goalPosDistance
                        goalPosDistance = goalPointAlt;
                        goalPosPrevious = minI;
                    end
                end
            end
        end
    end
end

function [distance] = calcDistance(p1, p2)
    distance = sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
end

function [path] = getPathFromPreviousDistanceVectors(previous, finishingAdjacency)
    nCells = size(previous,2);
    
    path = [];
    path(1) = finishingAdjacency;
    next = path(1);

    for j=1:nCells
        if ~isnan(previous(next))
            path(j) = previous(next);
            next = path(j);
        else 
            break;
        end
    end
    
    %just flip so that its from start to finish
    path = fliplr(path(1:end-1));
end

