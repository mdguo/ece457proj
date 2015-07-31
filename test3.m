    fieldDimensions = [12 14];
    startingCoords = [ 1 1; 1 9; 10 2.5; 11 8; 6 11 ];
    goalCoord = [ 6.5 4.5 ];
    
    obstacles = zeros(16, 2, 6) * NaN;    % 3 obstacles, max 4 vertices in each obstacle
    obstacles(:,:,1) = [ 1 3; 1.2 3.2; 3.2 1.2; 3 1; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN ];
    obstacles(:,:,2) = [ 2 7; 2 8; 3 8; 3 7; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN ];
    obstacles(:,:,3) = [ 4 2.5; 4 6.5; 6 4.5; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN ];
    obstacles(:,:,4) = [ 5 2; 5 2.5; 7 2.5; 7 6.5; 5 6.5; 5 7; 8 7; 8 2; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN ];
    obstacles(:,:,5) = [ 9 1; 9 4.5; 10 1; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN ];
    obstacles(:,:,6) = [ 9 6; 9 8; 9.5 8; 9.5 6.5; 11 6.5; 11 6; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN; NaN NaN ];
    obstacles(:,:,6) = [ 4 13; 4 9; 8 9; 8 13; 5 13; 5 10; 7 10; 7 12; 6.5 12; 6.5 10.5; 5.5 10.5; 5.5 12.5; 7.5 12.5; 7.5 9.5; 4.5 9.5; 4.5 13 ];
    
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, [], [], [], []);
    
    [paths cellAdjacencies startingAdjacencies] = GenInitialPaths(fieldDimensions, startingCoords, goalCoord, obstacles);
     
    hValues = ones(nnz(paths),1)*0.5;
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, hValues, cellAdjacencies, startingAdjacencies);
    
    %[gMinOpt nIterations gMin] = PSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    %plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, gMinOpt, cellAdjacencies, startingAdjacencies);
    %nIterations
    %gMin
    
    [gMinOpt nIterations gMin scores] = CPSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    plotSpace(fieldDimensions, obstacles, startingCoords, goalCoord, paths, gMinOpt, cellAdjacencies, startingAdjacencies);
    
    figure()
    plot(1:nIterations, scores);