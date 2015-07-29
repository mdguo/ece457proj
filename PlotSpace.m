% Author: William Mayo

function PlotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, hValues, cellAdjacencies, startingAdjacencies)
    figure();
    h = plot(1:fieldDimensions(1),0);
    axis([0 fieldDimensions(1) 0 fieldDimensions(2)]);
    hold on;
    set(h,'linewidth',2);

    % plot each obstacle
    for i = 1:size(obstacles,3)
        lastIndex = 1;
        for j = 2:size(obstacles,1)
            if isnan(obstacles(j,:,i))
                break;
            end
            lastIndex = lastIndex + 1;
            plot([obstacles(j-1,1,i) obstacles(j,1,i)], [obstacles(j-1,2,i) obstacles(j,2,i)], '-r');
        end
        % make sure to include the edge that wraps around
        plot([obstacles(1,1,i) obstacles(lastIndex,1,i)], [obstacles(1,2,i) obstacles(lastIndex,2,i)], '-r');
    end
    
    %add markers for starting and ending positions
    plot(goalCoord(1), goalCoord(2), 'Xg');
    for i=1:size(startingCoords,1)
        plot(startingCoords(i,1), startingCoords(i,2), 'og');
    end
    
    %now plot each path
    hIndex = 1;
    
    for i=1:size(paths,1)
        %note i is the index of the goal coord
        path = paths(i,:);
        
        %Add distance from starting i to firrst adjacency
        boundary = cellAdjacencies(:,path(1),startingAdjacencies(i));
        from = startingCoords(i,:);
        to = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*hValues(hIndex)];
        plot([from(1) to(1)], [from(2) to(2)], '-+');
        hIndex = hIndex + 1;
        
        %Now for each cell adjacency along the path add their costs
        prevBoundary = boundary;
        for j = 2:size(path,2)
            if path(j) == 0
                break;
            end
            
            boundary = cellAdjacencies(:,path(j),path(j-1));
            from = [prevBoundary(1), prevBoundary(2) + (prevBoundary(3) - prevBoundary(2))*hValues(hIndex-1)];
            to = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*hValues(hIndex)];
            plot([from(1) to(1)], [from(2) to(2)], '-+');
            hIndex = hIndex + 1;
            
            prevBoundary = boundary;
        end
        
        %calculate the distance from the last cell boundary to the finish
        %point

        from = [boundary(1), boundary(2) + (boundary(3) - boundary(2))*hValues(hIndex-1)];
        to = goalCoord;
        plot([from(1) to(1)], [from(2) to(2)], '-+');
    end
end
