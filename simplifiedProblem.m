    fieldDimensions = [10 10];
    startingCoords = [ 2 3; 8 3];
    goalCoord = [ 4 9 ];
    
    obstacles = zeros(4, 2, 1) * NaN;    % 3 obstacles, max 4 vertices in each obstacle
    obstacles(:,:,1) = [ 5 8; 3 5; 7 5; NaN NaN ];
    
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, [], [], [], []);
    
    [paths cellAdjacencies startingAdjacencies] = GenInitialPaths(fieldDimensions, startingCoords, goalCoord, obstacles);
    
    hValues = ones(nnz(paths),1)*0.5;
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, hValues, cellAdjacencies, startingAdjacencies);
    
    [gMinOpt nIterations gMin] = PSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, gMinOpt, cellAdjacencies, startingAdjacencies);
    nIterations
    gMin