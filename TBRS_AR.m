function stateOut = TBRS_AR(BW, I, Q, U, C, state)
%% Tile-based rate selection algorithm (Convex Hull)
% parameters
x = zeros(I,Q); % selection parameter
u = zeros(I,Q); % marginal utility

stateIn = state;
% current state
for ii=1:I
    x(ii,1:state(ii)) = 1;
end

% initial lambda
lamb = zeros(I,Q); 
for ii=1:I
    if state(ii)==0
        for qq=1:Q
            lamb(ii,qq) = U(ii,qq)/C(ii,qq);
            marU(ii,qq) = U(ii,qq);
            marC(ii,qq) = C(ii,qq); 
        end
    else
        for qq=state(ii)+1:Q
            lamb(ii,qq) = (U(ii,qq)-U(ii,state(ii)))/(C(ii,qq)-C(ii,state(ii))); 
            marU(ii,qq) = U(ii,qq)-U(ii,state(ii));
            marC(ii,qq) = C(ii,qq); 
        end 
    end
end 

currentBW = 0; accumU = 0; cc = 1;
% Iteration for bit allocation
while currentBW(cc) <= BW
    % Intitial value
    Uvector = reshape(marU,[1 I*Q]);
    Cvector = reshape(marC,[1 I*Q]);
    xvector = reshape(x,[1 I*Q]);
    lamb_vector = reshape(lamb,[1 I*Q]);
    [lambSort(cc) lambIndex(cc)] = max(lamb_vector);

    currentBW(cc+1) = currentBW(cc) + Cvector(lambIndex(cc));
    accumU(cc+1) = accumU(cc) + Uvector(lambIndex(cc));
    xvector(lambIndex(cc)) = 1;

    x = reshape(xvector, [I Q]);
    if mod(lambIndex(cc),I) == 0
        state(I) = ceil(lambIndex(cc)/I);
        updateI = I;
    else
        state(mod(lambIndex(cc),I)) = ceil(lambIndex(cc)/I);
        updateI = mod(lambIndex(cc),I);
    end
    % if statement for break the while loop when Quality reach to max
    
    lamb(updateI,:) = zeros(1,Q);
    % update lambda
    for qq=state(updateI)+1:Q
        lamb(updateI,qq) = (U(updateI,qq)-U(updateI,state(updateI)))/(C(updateI,qq)-C(updateI,state(updateI))); 
        marU(updateI,qq) = U(updateI,qq)-U(updateI,state(updateI));
        marC(updateI,qq) = C(updateI,qq)-C(updateI,state(updateI)); 
    end 
    
    cc = cc + 1;
end
stateOut = state;

% figure(1); hold; 
% plot(C(1,:)',U(1,:)','*');
% [a b] = sort(U.*x); c = sum(a>0);
% plot(C(b(end-c+1:end)),U(b(end-c+1:end))'.*x(b(end-c+1:end))','r-');

end