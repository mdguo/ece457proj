    fieldDimensions = [10 10];
    startingCoords = [ 1 1; 9 9; 2 9; 9 3];
    goalCoord = [ 6 6 ];
    
    obstacles = zeros(4, 2, 3) * NaN;    % 3 obstacles, max 4 vertices in each obstacle
    obstacles(:,:,1) = [ 3 9; 2 7 ; 4 4 ; NaN NaN];
    obstacles(:,:,2) = [ 6 1; 6 5 ; 8 5 ; 8 1];
    obstacles(:,:,3) = [ 7 6; 8 9 ; 9 5 ; NaN NaN];
    
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, [], [], [], []);
    
    [paths cellAdjacencies startingAdjacencies] = GenInitialPaths(fieldDimensions, startingCoords, goalCoord, obstacles);
    
    hValues = ones(nnz(paths),1)*0.5;
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, hValues, cellAdjacencies, startingAdjacencies);
    
    [gMinOpt nIterations gMin] = PSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, gMinOpt, cellAdjacencies, startingAdjacencies);
    nIterations
    gMin