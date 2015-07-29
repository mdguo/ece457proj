% Implementation of concurrent PSO

function [gMinOpt nIterations gMin] = PSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    %End condition is in terms of the number of iterations with no
    %improvment
    nNIInterations = 10;
    nIterations = 0;
    
    %For concurrent PSO we generate two separate populatsion. One
    %population will be more social and the other will be more cognitive
    %based on their values of C1 and C2. Populations exchange their global
    %optimas every 7 iterations
    
    %useful vars
    nVars = nnz(paths);
    
    %generate population 1 (social population)
    pop1 = struct();
    pop1.C1 = 1;
    pop1.C2 = 1.4944;
    pop1.w = 0.792;
    pop1.size = 10;
    pop1.gMin = Inf;
    pop1.gMinOpt = NaN(nVars,1); %global optimum hValues
    pop1.populationSolutions = rand(nVars,pop1.size);
    pop1.populationPBest = pop1.populationSolutions;
    pop1.populationCosts = zeros(pop1.size,1);
    pop1.populationMinCost = Inf(pop1.size,1);
    pop1.populationVelocities = zeros(nVars,pop1.size); %velocities are initialized to zero
    pop1 = initializePopulation(pop1, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    
    %generate population 2 (cognitive population)
    pop2 = struct();
    pop2.C1 = 1.4944;
    pop2.C2 = 1;
    pop2.w = 0.792;
    pop2.size = 10;
    pop2.gMin = Inf;
    pop2.gMinOpt = NaN(nVars,1); %global optimum hValues
    pop2.populationSolutions = rand(nVars,pop2.size);
    pop2.populationPBest = pop2.populationSolutions;
    pop2.populationCosts = zeros(pop2.size,1);
    pop2.populationMinCost = Inf(pop2.size,1);
    pop2.populationVelocities = zeros(nVars,pop2.size); %velocities are initialized to zero
    pop2 = initializePopulation(pop2, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    
    nNoImprovement = 0;
    
    while nNoImprovement < nNIInterations
        newGMinFound = 0;
        nIterations = nIterations + 1;
        
        pop1 = performIteration(pop1, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
        pop2 = performIteration(pop2, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);

        if mod(nIterations,nNIInterations) == 0
            %every nNIInterations exchange optimal values between
            %populations
            if pop1.gMin < pop2.gMin
                nNoImprovement = 0;
                pop2.gMin = pop1.gMin;
                pop2.gMinOpt = pop1.gMinOpt;
            elseif pop1.gMin > pop2.gMin
                nNoImprovement = 0;
                pop1.gMin = pop2.gMin;
                pop1.gMinOpt = pop2.gMinOpt;
            else
                nNoImprovement = nNoImprovement + 1;
            end
            
            gMinOpt = pop2.gMinOpt;
            gMin = pop2.gMin;
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