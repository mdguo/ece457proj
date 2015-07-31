% Implementation of concurrent PSO

function [gBestSol nIterations gBest scores] = CPSO(paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    %End condition is in terms of the number of iterations with no
    %improvment
    nNIInterations = 100;
    nIterations = 0;
    nExchangeRate = 15;
    gBest = Inf;
    gBestSol = [];
    scores = [];
    
    %For concurrent PSO we generate two separate populatsion. One
    %population will be more social and the other will be more cognitive
    %based on their values of C1 and C2. Populations exchange their global
    %optimas every 7 iterations
    
    %useful vars
    nVars = nnz(paths);
    
    %generate population 1 (social population)
    pop1 = struct();
    pop1.C1 = 0.5;
    pop1.C2 = 2;
    pop1.w = 0.792;
    pop1.size = 20;
    pop1.gBest = Inf;
    pop1.gBestSol = NaN(nVars,1); %global optimum hValues
    pop1.solutions = rand(nVars,pop1.size);
    pop1.pBest = pop1.solutions;
    pop1.pBestCost = Inf(pop1.size,1);
    pop1.velocities = zeros(nVars,pop1.size); %velocities are initialized to zero
    pop1 = initializePopulation(pop1, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    
    %generate population 2 (cognitive population)
    pop2 = struct();
    pop2.C1 = 1.4944;
    pop2.C2 = 1.4944;
    pop2.w = 0.792;
    pop2.size = 20;
    pop2.gBest = Inf;
    pop2.gBestSol = NaN(nVars,1); %global optimum hValues
    pop2.solutions = rand(nVars,pop2.size);
    pop2.pBest = pop2.solutions;
    pop2.pBestCost = Inf(pop2.size,1);
    pop2.velocities = zeros(nVars,pop2.size); %velocities are initialized to zero
    pop2 = initializePopulation(pop2, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
    
    if gBest > pop2.gBest
        gBestSol = pop2.gBestSol;
        gBest = pop2.gBest;
    elseif gBest > pop1.gBest
        gBestSol = pop1.gBestSol;
        gBest = pop1.gBest;
    end
    
    nNoImprovement = 0;
    
    while nNoImprovement < nNIInterations
        newgBestFound = 0;
        nIterations = nIterations + 1;
        
        pop1 = performIteration(pop1, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
        pop2 = performIteration(pop2, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);

        if mod(nIterations,nExchangeRate) == 0
            %every nNIInterations exchange optimal values between
            %populations
            if pop1.gBest < pop2.gBest
                pop2.gBest = pop1.gBest;
                pop2.gBestSol = pop1.gBestSol;
            elseif pop1.gBest > pop2.gBest
                pop1.gBest = pop2.gBest;
                pop1.gBestSol = pop2.gBestSol;
            end
        end
        
        if gBest > pop2.gBest
            nNoImprovement = 0;
            gBestSol = pop2.gBestSol;
            gBest = pop2.gBest;
        elseif gBest > pop1.gBest
            nNoImprovement = 0;
            gBestSol = pop1.gBestSol;
            gBest = pop1.gBest;
        else
            nNoImprovement = nNoImprovement + 1;
        end
        
        scores = [scores gBest];
    end
end 

function [pop] = initializePopulation(pop, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    for i =1:pop.size
        cost = GetTotalPathCost(pop.solutions(:,i), paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);
        
        if cost < pop.gBest
            pop.gBest = cost;
            pop.gBestSol = pop.solutions(:,i);
        end
    end
end

function [pop] = performIteration(pop, paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord)
    nVars = nnz(paths);

    for i = 1:pop.size
        pop.velocities(:,i) = pop.w.*pop.velocities(:,i) + ... 
            pop.C1*rand(nVars,1).*(pop.pBest(:,i) - pop.solutions(:,i)) + ...
            pop.C2*rand(nVars,1).*(pop.gBestSol - pop.solutions(:,i));

        pop.solutions(:,i) = pop.solutions(:,i) + pop.velocities(:,i);

        %make sure no solution has gone over or under the limit
        over = pop.solutions(:,i) > 1;
        under = pop.solutions(:,i) < 0;

        pop.solutions(:,i) = pop.solutions(:,i).*not(over)+over;
        pop.solutions(:,i) = pop.solutions(:,i).*not(under);

        %calculate new particle cost
        cost = GetTotalPathCost(pop.solutions(:,i), paths, cellAdjacencies, startingAdjacencies, startingCoords, goalCoord);

        %update best local min
        if cost < pop.pBestCost(i)
            pop.pBestCost(i) = cost;
            pop.pBest(i) = pop.solutions(i);
        end

        %check for new gBest
        if cost < pop.gBest
            pop.gBest = cost;
            pop.gBestSol = pop.solutions(:,i);
        end
        
    end
end