% Implementation of concurrent PSO

function [gMinOpt nIterations gMin] = PSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    %End condition is in terms of the number of iterations with no
    %improvment
    nNIInterations = 200;
    nIterations = 0;
    gMin = Inf;
    
    %For concurrent PSO we generate two separate populatsion. One
    %population will be more social and the other will be more cognitive
    %based on their values of C1 and C2. Populations exchange their global
    %optimas every 7 iterations
    
    %useful vars
    nVars = nnz(paths);
    
    %generate population 1 (social population)
    pop1 = struct();
    pop1.C1 = 1;
    pop1.C2 = 1.6;
    pop1.w = 0.792;
    pop1.size = 50;
    pop1.gMin = Inf;
    pop1.gMinOpt = NaN(nVars,1); %global optimum hValues
    pop1.populationSolutions = rand(nVars,pop1.size);
    pop1.populationPBest = pop1.populationSolutions;
    pop1.populationCosts = zeros(pop1.size,1);
    pop1.populationMinCost = Inf(pop1.size,1);
    pop1.populationVelocities = zeros(nVars,pop1.size); %velocities are initialized to zero
    pop1 = initializePopulation(pop1, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    
    nNoImprovement = 0;
    
    while nNoImprovement < nNIInterations
        nIterations = nIterations + 1;
        
        pop1 = performIteration(pop1, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
        

        if pop1.gMin < gMin
            nNoImprovement = 0;
            gMin = pop1.gMin;
            gMinOpt = pop1.gMinOpt;
        else
            nNoImprovement = nNoImprovement + 1;
        end
    end
end

function [pop] = initializePopulation(pop, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    for i =1:pop.size
        pop.populationCosts(i) = GetTotalPathCost(pop.populationSolutions(:,i), paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
        pop.populationMinCost(i) = pop.populationCosts(i);
        
        if pop.populationCosts(i) < pop.gMin
            pop.gMin = pop.populationCosts(i);
            pop.gMinOpt = pop.populationSolutions(:,i);
        end
    end
end

function [pop] = performIteration(pop, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    nVars = nnz(paths);

    for i = 1:pop.size
        pop.populationVelocities(:,i) = pop.w.*pop.populationVelocities(:,i) + ... 
            pop.C1*rand(nVars,1).*(pop.populationPBest(:,i) - pop.populationSolutions(:,i)) + ...
            pop.C2*rand(nVars,1).*(pop.gMinOpt - pop.populationSolutions(:,i));

        pop.populationSolutions(:,i) = pop.populationSolutions(:,i) + pop.populationVelocities(:,i);

        %make sure no solution has gone over or under the limit
        over = pop.populationSolutions(:,i) > 1;
        under = pop.populationSolutions(:,i) < 0;

        pop.populationSolutions(:,i) = pop.populationSolutions(:,i).*not(over)+over;
        pop.populationSolutions(:,i) = pop.populationSolutions(:,i).*not(under);

        %calculate new particle cost
        pop.populationCosts(i) = GetTotalPathCost(pop.populationSolutions(:,i), paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);

        %update best local min
        if pop.populationCosts(i) < pop.populationMinCost(i)
            pop.populationMinCost(i) = pop.populationCosts(i);
            pop.populationPBest(i) = pop.populationSolutions(i);
        end

        %check for new gmin
        if pop.populationCosts(i) < pop.gMin
            pop.gMin = pop.populationCosts(i);
            pop.gMinOpt = pop.populationSolutions(:,i);
        end
    end
end