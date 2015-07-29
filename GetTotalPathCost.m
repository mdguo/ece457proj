% Author: William Mayo

function [cost] = GetTotalPathCost(hValues, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    cost = 0;
    hIndex = 1;
    
    for i=1:size(paths,1)
        %note i is the index of the goal coord
        path = paths(i,:);
        
        %Add distance from starting i to firrst adjacency
        boundary = cellAdjacencies(:,path(1),startingAdjacencies(i));
        from = startingCoords(i,:);
        to = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*hValues(hIndex)];
        hIndex = hIndex + 1;
        cost = cost + calcDistance(from, to);
        
        %Now for each cell adjacency along the path add their costs
        prevBoundary = boundary;
        for j = 2:size(path,2)
            if path(j) == 0
                break;
            end
            
            boundary = cellAdjacencies(:,path(j),path(j-1));
            from = [prevBoundary(1), prevBoundary(2) + (prevBoundary(3) - prevBoundary(2))*hValues(hIndex-1)];
            to = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*hValues(hIndex)];
            hIndex = hIndex + 1;
            cost = cost + calcDistance(from, to);
            
            prevBoundary = boundary;
        end
        
        %calculate the distance from the last cell boundary to the finish
        %point

        from = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*hValues(hIndex-1)];
        to = goalCoord;
        cost = cost + calcDistance(from, to);
    end
end

function [distance] = calcDistance(p1, p2)
    distance = sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
end